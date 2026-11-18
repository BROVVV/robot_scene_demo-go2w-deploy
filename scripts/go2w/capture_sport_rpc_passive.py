#!/usr/bin/env python3
"""Passive SDK DDS request/response evidence; creates readers only, no clients."""
import argparse
import json
import os
from pathlib import Path
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize,ChannelSubscriber
from unitree_sdk2py.idl.unitree_api.msg.dds_ import Request_,Response_


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--interface',required=True);p.add_argument('--duration',type=float,default=7)
    p.add_argument('--json',required=True);args=p.parse_args()
    rows=[];subscribers=[]
    ChannelFactoryInitialize(0,os.getenv('GO2W_ROBOT_HOST_IP',args.interface))
    def callback(message,service,kind):
        h=message.header
        row={'received':time.time(),'service':service,'kind':kind,
             'request_id':int(h.identity.id),'api_id':int(h.identity.api_id)}
        if kind=='response':row.update(status=int(h.status.code),data=message.data)
        else:
            row.update(lease_id=int(h.lease.id),no_reply=bool(h.policy.noreply))
            if service=='sport' and row['api_id']==1008:
                try:
                    parameter=json.loads(message.parameter)
                    row['move_parameter']={k:parameter.get(k) for k in ('x','y','z')}
                except (ValueError,TypeError,AttributeError):
                    row['move_parameter_unreadable']=True
        rows.append(row)
    for service in ('sport_lease','sport'):
        for kind,typ in [('request',Request_),('response',Response_)]:
            sub=ChannelSubscriber('rt/api/'+service+'/'+kind,typ)
            sub.Init(lambda m,s=service,k=kind:callback(m,s,k),100);subscribers.append(sub)
    end=time.monotonic()+args.duration
    while time.monotonic()<end:time.sleep(.05)
    for sub in subscribers:sub.Close()
    requests={r['request_id']:r for r in rows if r['kind']=='request'}
    paired=[{**r,'request':requests.get(r['request_id'])} for r in rows if r['kind']=='response']
    result={'read_only':True,'sdk_clients_created':False,'rows':rows,'responses':paired}
    Path(args.json).write_text(json.dumps(result,indent=2))
    print(json.dumps({'rows':len(rows),'responses':paired}))


if __name__=='__main__':main()
