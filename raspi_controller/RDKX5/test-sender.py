#!/usr/bin/env python3
"""Bench-test protocol-v2 RDK bridge over UDP.

This is NOT the final Raspberry Pi controller sender. It only generates a
heartbeat and a changing ABS_X event so the RDK state machine can be tested.
Ctrl-C stops traffic; observe 200 ms neutral and 1 s virtual-device removal.
"""
import argparse, os, random, socket, struct, time
HEADER=struct.Struct('!2sBBII')
EVENT=struct.Struct('@llHHi')
MAGIC=b'RC'; VERSION=2; EVENT_KIND=0; HEARTBEAT=1

def iev(t,c,v):
    n=time.time_ns(); return EVENT.pack(n//1_000_000_000,(n%1_000_000_000)//1000,t,c,v)

def main():
    p=argparse.ArgumentParser(); p.add_argument('host'); p.add_argument('--port',type=int,default=5000)
    a=p.parse_args(); s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    session=random.getrandbits(32); seq=0; x=128; d=12; next_event=time.monotonic()
    print(f'session=0x{session:08x}; Ctrl-C to simulate total link loss')
    while True:
        now=time.monotonic()
        payload=b'\0'*EVENT.size
        s.sendto(HEADER.pack(MAGIC,VERSION,HEARTBEAT,session,seq)+payload,(a.host,a.port))
        if now>=next_event:
            x+=d
            if x>=220 or x<=35: d=-d
            pkt=HEADER.pack(MAGIC,VERSION,EVENT_KIND,session,seq)+iev(3,0,x)  # EV_ABS/ABS_X
            s.sendto(pkt,(a.host,a.port)); seq=(seq+1)&0xffffffff; next_event=now+0.2
        time.sleep(0.05)
if __name__=='__main__': main()
