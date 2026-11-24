#!/usr/bin/env python3
"""NAK-code-2 hunt: what command/state transitions does the live Sirius NAK?
All motion RA-only, LOW-speed 10x (~0.15 deg/s), runs 1.0-1.5 s, always :K'd.
Consent: standing motor-test consent 2026-11-16.
"""
import serial, time

PORT = "/dev/serial/by-id/usb-FTDI_TTL232R_FTA67MMH-if00-port0"


def revu(v):
    return f"{v & 0xFF:02X}{(v >> 8) & 0xFF:02X}{(v >> 16) & 0xFF:02X}"


def rp(h):
    return int(h[4:6] + h[2:4] + h[0:2], 16)


s = serial.Serial(PORT, 9600, timeout=2)


def cmd(f, quiet=False):
    s.reset_input_buffer()
    s.write((f + "\r").encode())
    buf = b""
    while not buf.endswith(b"\r"):
        c = s.read(1)
        if not c:
            break
        buf += c
    r = buf.decode(errors="replace").strip("\r")
    if not quiet:
        print(f"  {f:12s} -> {r!r}")
    time.sleep(0.03)
    return r


def rate(tag, period_us, seconds=1.2):
    """Run forward low-speed at period, measure steps/s while running."""
    cmd(f":G110", quiet=True)
    cmd(f":I1{revu(period_us)}", quiet=True)
    cmd(":J1", quiet=True)
    p0 = rp(cmd(":j1", quiet=True)[1:])
    time.sleep(seconds)
    p1 = rp(cmd(":j1", quiet=True)[1:])
    print(f"   [{tag}] {(p1 - p0) / seconds:.0f} steps/s over {seconds}s")
    return p1 - p0


print("== state ==")
cmd(":f1")

print("== Q1/Q2/Q3: what does a RUNNING (low-speed) axis NAK? ==")
# start low-speed fwd at 10x (62 us)
cmd(":G110")
cmd(":I1" + revu(62))
cmd(":J1")
time.sleep(0.3)
print("  axis now running; f1 =", cmd(":f1"))
q1 = cmd(":I1" + revu(50))          # change period while running LOW
q2 = cmd(":G110")                   # same mode+dir while running
q3 = cmd(":G131")                   # switch to high-speed-back while running (this NAK'd in daemon)
q4 = cmd(":J1")                     # re-:J while running
cmd(":K1")
time.sleep(0.4)
print(f"  verdict: :I={q1} :Gsame={q2} :Gswitch={q3} :Jagain={q4}")

print("== Q5: does an accepted :I change speed WHILE running? ==")
# 1.5x (400us ~ 130 sps-ish measured earlier 1.25x=137/s), then jump to 10x (62us) mid-run
cmd(":G110")
cmd(":I1" + revu(496))
cmd(":J1")
p0 = rp(cmd(":j1", quiet=True)[1:])
time.sleep(1.0)
p1 = rp(cmd(":j1", quiet=True)[1:])
r1 = (p1 - p0) / 1.0
cmd(":I1" + revu(62), quiet=True)   # accepted? and effective?
time.sleep(0.5)
p2 = rp(cmd(":j1", quiet=True)[1:])
r2 = (p2 - p1) / 0.5
cmd(":K1")
time.sleep(0.4)
p3 = rp(cmd(":j1", quiet=True)[1:])
print(f"   phase1 ~496us: {r1:.0f} sps | phase2 after :I62 mid-run: {r2:.0f} sps | coast={p3 - p2}")
print(f"   (if r2 >> r1: :I is live-effective while running; board has no forced ramp)")

print("== Q6: :I change while running HIGH-speed ==")
# high-speed 130x: mode :G130, period per vendor: high-speed period = target/16? 
# use 20 us -> high band. Short run only.
cmd(":G130")
cmd(":I1" + revu(20))
cmd(":J1")
time.sleep(0.4)
f_ = cmd(":f1")
q6 = cmd(":I1" + revu(25))
q7 = cmd(":G130")
cmd(":K1")
time.sleep(1.2)  # high-speed decel tail ~1 s
print(f"   running high f={f_} :Ichange={q6} :Gsame={q7}")

print("== Q7: after NAK, is the axis still healthy? re-run 10x 1s, measure ==")
rate("post-NAK", 62, 1.0)
cmd(":K1")
time.sleep(0.3)

print("== rewind to park-ish (we drifted fwd; count and reverse) ==")
p = rp(cmd(":j1", quiet=True)[1:])
park = 0x800011
drift = p - park
print(f"   counter={p:06X} park={park:06X} drift={drift} steps")
if abs(drift) > 200:
    # reverse low-speed 10x ~1050 sps; stop early by coast budget
    cmd(":G111", quiet=True)
    cmd(":I1" + revu(62), quiet=True)
    cmd(":J1", quiet=True)
    lead = 1200 if drift > 0 else -1200
    while True:
        p = rp(cmd(":j1", quiet=True)[1:])
        d = p - park
        if (drift > 0 and d <= lead) or (drift < 0 and d >= lead):
            break
        time.sleep(0.1)
    cmd(":K1", quiet=True)
    time.sleep(0.4)
    p = rp(cmd(":j1", quiet=True)[1:])
    d = p - park
    print(f"   after coast: {p:06X} residual={d}")
    rem = p - park
    if abs(rem) > 60:
        cmd(":G111" if rem > 0 else ":G110", quiet=True)
        cmd(":I1" + revu(620), quiet=True)  # 1x creep
        cmd(":J1", quiet=True)
        while True:
            p = rp(cmd(":j1", quiet=True)[1:])
            d = p - park
            if abs(d) <= 40:
                break
            time.sleep(0.05)
        cmd(":K1", quiet=True)
        time.sleep(0.25)
        p = rp(cmd(":j1", quiet=True)[1:])
        print(f"   creep-stopped at {p:06X} residual={p - park}")
print("final: f1 =", cmd(":f1"), " j1 =", cmd(":j1"))
print("DONE")
