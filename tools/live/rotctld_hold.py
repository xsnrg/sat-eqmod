#!/usr/bin/env python3
"""Tiny robust rotctld client: hold connection, push target, hold, report.
Usage: rotctld_hold.py AZ EL [hold_seconds]
Sends \\dump_state, P AZ EL, polls p until within 0.05 deg or hold expires,
prints final az el, then disconnects (daemon drops target on last drop)."""
import socket
import sys
import time

az = sys.argv[1]
el = sys.argv[2]
hold = float(sys.argv[3]) if len(sys.argv) > 3 else 45.0

s = socket.create_connection(("localhost", 4533), timeout=5)
s.settimeout(3)


def cmd(c, want=1):
    s.sendall(c.encode() + b"\n")
    buf = b""
    while buf.count(b"\n") < want:
        try:
            b2 = s.recv(4096)
        except Exception:
            break
        if not b2:
            break
        buf += b2
    return buf.decode().strip()


cmd("\\dump_state", 2)
print("P ->", cmd("P {} {}".format(az, el)))
t0 = time.time()
last = "?"
while time.time() - t0 < hold:
    reply = cmd("p", 2).replace("\n", " ")
    parts = reply.split()
    if len(parts) >= 2:
        last = reply
        if abs(float(parts[0]) - float(az)) < 0.05 and abs(float(parts[1]) - float(el)) < 0.05:
            print("settled: {} after {:.1f}s".format(reply, time.time() - t0))
            break
    time.sleep(0.5)
else:
    print("timeout after {:.0f}s, final: {}".format(hold, last))
s.close()
