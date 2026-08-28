#!/usr/bin/env python3
"""Small protocol-v2 UDP decoder for bench testing the Raspberry Pi sender."""
import argparse
import socket
import struct

HEADER = struct.Struct("!2sBBII")
INPUT_EVENT = struct.Struct("@llHHi")
SIZE = HEADER.size + INPUT_EVENT.size

p = argparse.ArgumentParser()
p.add_argument("--port", type=int, default=5000)
a = p.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", a.port))
print(f"listening UDP :{a.port}, record={SIZE} B")
while True:
    data, addr = s.recvfrom(65535)
    if len(data) % SIZE:
        print("malformed", addr, len(data))
        continue
    for off in range(0, len(data), SIZE):
        rec = data[off:off+SIZE]
        magic, version, kind, session, seq = HEADER.unpack_from(rec)
        if kind == 0:
            sec, usec, etype, code, value = INPUT_EVENT.unpack(rec[HEADER.size:])
            print(addr, magic, version, f"session={session:08x}", f"seq={seq}", f"type={etype} code={code} value={value}")
        else:
            print(addr, magic, version, f"session={session:08x}", "heartbeat")
