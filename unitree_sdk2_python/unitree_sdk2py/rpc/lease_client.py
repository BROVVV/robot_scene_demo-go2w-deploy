import time
import socket
import os
import json

from threading import Thread, Lock

from .client_base import ClientBase
from .internal import *


"""
" class LeaseContext
"""
class LeaseContext:
    def __init__(self):
        self.id = 0
        self.term = RPC_LEASE_TERM

    def Update(self, id, term):
        self.id = id
        self.term = term

    def Reset(self):
        self.id = 0
        self.term = RPC_LEASE_TERM

    def Valid(self):
        return self.id != 0


"""
" class LeaseClient
"""
class LeaseClient(ClientBase):
    def __init__(self, name: str):
        self.__name = name + "_lease"
        self.__contextName = socket.gethostname() + "/" + name + "/" + str(os.getpid())
        self.__context = LeaseContext()
        self.__thread = None
        self.__lock = Lock()
        super().__init__(self.__name)
        print("[LeaseClient] lease name:", self.__name, ", context name:", self.__contextName)
    
    def Init(self):
        # Go2-W grants a one-second Sport lease.  A one-second RPC timeout plus
        # the old 0.3-second sleep guaranteed expiry after a single delayed
        # renewal on the loaded Foxy/Jetson runtime.  Fail fast and retry while
        # there is still time left in the current lease term.
        self.SetTimeout(0.3)
        self.__thread = Thread(target=self.__ThreadFunc, name=self.__name, daemon=True)
        self.__thread.start()

    def WaitApplied(self):
        while True:
            with self.__lock:
                if self.__context.Valid():
                    break
            time.sleep(0.1)            
    
    def GetId(self):
            with self.__lock:
                return self.__context.id
    
    def Applied(self):
            with self.__lock:
                return self.__context.Valid()

    def Invalidate(self):
        with self.__lock:
            self.__context.Reset()
    
    def __Apply(self):
        parameter = {}
        parameter["name"] = self.__contextName
        p = json.dumps(parameter)

        c, d = self._CallBase(RPC_API_ID_LEASE_APPLY, p)
        if c != 0:
            print("[LeaseClient] apply lease error. code:", c)
            return False

        data = json.loads(d)
        
        id = data["id"]
        term = data["term"]

        print("[LeaseClient] lease applied id:", id, ", term:", term)

        with self.__lock:
            self.__context.Update(id, float(term/1000000))
        return True
    
    def __Renewal(self):
        parameter = {}
        p = json.dumps(parameter)

        c, d = self._CallBase(RPC_API_ID_LEASE_RENEWAL, p, 0, self.__context.id)
        if c != 0:
            print("[LeaseClient] renewal lease error. code:", c)
            if c == RPC_ERR_SERVER_LEASE_NOT_EXIST:
                with self.__lock:
                    self.__context.Reset()
            return False
        return True

    def __RenewalNoReply(self):
        # A lease renewal is idempotent and its useful effect happens when the
        # server consumes the request, not when this client receives the
        # acknowledgement.  On the loaded Go2-W, synchronous response
        # callbacks can be delayed beyond the one-second lease term even
        # though reliable DDS requests are delivered.  Keep the lease alive
        # with lightweight no-reply heartbeats and use __Renewal only as a
        # periodic health probe.
        with self.__lock:
            lease_id = self.__context.id
        code = self._CallNoReplyBase(
            RPC_API_ID_LEASE_RENEWAL, "{}", 0, lease_id
        )
        if code != 0:
            print("[LeaseClient] renewal send error. code:", code)
            return False
        return True
    
    def __GetWaitSec(self):
        waitsec = 0.0
        if self.__context.Valid():
            waitsec = self.__context.term

        if waitsec <= 0:
            waitsec = RPC_LEASE_TERM

        return waitsec

    def __ThreadFunc(self):
        renewals_since_probe = 0
        while True:
            if self.__context.Valid():
                succeeded = self.__RenewalNoReply()
                if succeeded:
                    renewals_since_probe += 1
                    if renewals_since_probe >= 20:
                        # A response timeout does not invalidate the lease:
                        # the preceding no-reply heartbeats continue renewing
                        # it.  A definitive 3206 response does reset the
                        # context inside __Renewal and causes a fresh Apply.
                        self.__Renewal()
                        renewals_since_probe = 0
            else:
                succeeded = self.__Apply()
                renewals_since_probe = 0
            # Ten small reliable heartbeats per second leave ample scheduling
            # margin inside the Go2-W server's fixed one-second lease term.
            time.sleep(self.__GetWaitSec() * 0.10 if succeeded else 0.05)
