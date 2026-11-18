#!/usr/bin/env python3
"""Announce only this Wi-Fi interface's own IPv4/MAC to refresh LAN ARP caches.

This bypasses a missing ARP-response path; it does not repair device firmware.
No addresses, routes, Wi-Fi profiles, motion endpoints or other hosts are changed.
"""
import argparse
import ipaddress
import json
import re
import signal
import socket
import struct
import subprocess
import threading
from pathlib import Path


def announcement(mac, address, operation):
    if not re.fullmatch(r'(?:[0-9a-fA-F]{2}:){5}[0-9a-fA-F]{2}', mac):
        raise ValueError('invalid interface MAC')
    hardware = bytes.fromhex(mac.replace(':', ''))
    if hardware[0] & 1 or hardware == bytes(6):
        raise ValueError('interface MAC must be unicast')
    ip = ipaddress.IPv4Address(address)
    if ip.is_unspecified or ip.is_multicast or ip.is_loopback or int(ip) == 0xffffffff:
        raise ValueError('invalid interface IPv4')
    if operation not in (1, 2):
        raise ValueError('invalid ARP operation')
    return (b'\xff' * 6 + hardware + b'\x08\x06' +
            struct.pack('!HHBBH6s4s6s4s', 1, 0x800, 6, 4, operation,
                        hardware, ip.packed, bytes(6), ip.packed))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--interface', default='wlan0')
    parser.add_argument('--interval', type=float, default=5.)
    args = parser.parse_args()
    if not re.fullmatch(r'[a-zA-Z0-9_.-]{1,15}', args.interface):
        parser.error('invalid interface name')
    if not 2 <= args.interval <= 60:
        parser.error('interval must be between 2 and 60 seconds')
    stopped = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopped.set())
    previous = None
    while not stopped.is_set():
        state = 'waiting for interface IPv4'
        try:
            result = subprocess.run(['ip', '-j', 'address', 'show', 'dev', args.interface],
                                    capture_output=True, text=True, timeout=2)
            interfaces = json.loads(result.stdout or '[]')
            addresses = [a['local'] for interface in interfaces
                         if 'UP' in interface.get('flags', [])
                         for a in interface.get('addr_info', [])
                         if a.get('family') == 'inet' and a.get('scope') == 'global']
            if addresses:
                mac = (Path('/sys/class/net') / args.interface / 'address').read_text().strip()
                with socket.socket(socket.AF_PACKET, socket.SOCK_RAW,
                                   socket.htons(0x0806)) as stream:
                    stream.bind((args.interface, 0))
                    for address in addresses:
                        for operation in (1, 2):
                            stream.send(announcement(mac, address, operation))
                state = 'announcing %s on %s (%s)' % (','.join(addresses), args.interface, mac)
        except Exception as error:
            state = type(error).__name__ + ': ' + str(error)
        if state != previous:
            print(state, flush=True)
            previous = state
        stopped.wait(args.interval)


if __name__ == '__main__':
    main()
