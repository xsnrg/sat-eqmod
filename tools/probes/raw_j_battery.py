#!/usr/bin/env python3
"""Raw :j stream battery: is the counter/serial sane at rest and in motion?

Logs RAW reply strings (no parsing) so corruption is visible.
Motion test is deliberately slow/visible: 10x sidereal, 20 s, RA axis
(~0.84 deg total at the mount). Stop with :K.
"""
import serial, time, sys

PORT = "/dev/serial/by-id/usb-FTDI_TTL232R_FTA67MMH-if00-port0"
AX = sys.argv[1] if len(sys.argv) > 1 else "1"

s = serial.Serial(PORT, 9600, timeout=0.4)
def revu24(v):
    h = "%06X" % v          # wire format = byte-swapped, like Board::revu24_format
    return h[4:6] + h[2:4] + h[0:2]

def cmd(frame, wait=True):
    s.reset_input_buffer()
    s.write((frame + "\r").encode())
    if not wait: return ""
    buf = b""
    while not buf.endswith(b"\r"):
        c = s.read(1)
        if not c: break
        buf += c
    return buf.decode(errors="replace").strip()

print("mode:", cmd(":e" + AX))
print("period :D:", cmd(":D" + AX))
print("status :f:", cmd(":f" + AX))
print("## 10 s at rest, raw :j twice/sec")
for i in range(20):
    print(f"{i*0.5:5.1f}", repr(cmd(":j" + AX)), repr(cmd(":f" + AX)))
print("## arm + 10x forward, raw :j every 0.25 s for 20 s")
print("F:", cmd(":F" + AX))
print("G:", cmd(":G" + AX + "10"))
print("I:", cmd(":I%s%s" % (AX, revu24(62))))   # 62 us = 10.0x sidereal, swapped
print("J:", cmd(":J" + AX))
t0 = time.monotonic()
while time.monotonic() - t0 < 20.0:
    print(f"{time.monotonic()-t0:6.2f}", repr(cmd(":j" + AX)), repr(cmd(":f" + AX)))
    time.sleep(0.05)
print("K:", cmd(":K" + AX))
time.sleep(1.5)
print("after K:", repr(cmd(":j" + AX)), repr(cmd(":f" + AX)))
print("period now :D:", cmd(":D" + AX))
