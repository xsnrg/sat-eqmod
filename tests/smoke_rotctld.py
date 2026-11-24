#!/usr/bin/env python3
"""Drive sateqd --sim over real TCP with the rotctld wire protocol.

POLAR-session smoke: launch the daemon with a polar profile/home so the
target sweep (az 180+t, el 60+0.5t, home az=180 el=60) is well-conditioned:

  sateqd --sim heq5 --lat 44.95 --lon -93.28 --home-az 180 --home-el 60 \
      --rotctld-port 14533

(For an altaz session, which starts tube-level EAST, use smoke_readonly.sh.)
"""
import socket, sys, time

HOST, PORT = "127.0.0.1", 14533

def conn():
    s = socket.create_connection((HOST, PORT), timeout=5)
    s.settimeout(5)
    return s

def readline_parts(s, until, timeout=5.0):
    s.settimeout(timeout)
    buf = b""
    while True:
        buf += s.recv(4096)
        if until in buf:
            return buf.decode()

def main():
    s = conn()
    # NOTE: escaped backslash in a NON-raw literal — rb"...\n" would send a
    # literal backslash-n and the daemon would wait forever for the line end.
    s.sendall(b"\\dump_state\n")
    ds = readline_parts(s, b"done\n")
    assert ds.startswith("1\n1\n") and "rot_type=AzEl" in ds, ds[:80]
    print("dump_state OK")

    # initial position read
    s.sendall(b"p\n"); az, el = readline_parts(s, b"\n", 2).split()[:2]
    print(f"initial pos: az={az} el={el}")

    # static target at home: should not move (allow tiny)
    s.sendall(b"P 180 60\n"); assert b"RPRT 0" in s.recv(64)
    time.sleep(1.0)
    s.sendall(b"p\n"); a1, e1 = readline_parts(s, b"\n", 2).split()[:2]
    print(f"after 1s at home: az={a1} el={e1}")

    # realistic LEO-like sweep starting exactly at current pointing:
    # az +1 deg/s, el +0.5 deg/s for 12 s (worst-case zenith pass ~1 deg/s).
    # NOTE: evaluate the trajectory at the moment the position is polled,
    # not when the P command was sent, else polling delay counts as error.
    t0 = time.time()
    err_max = 0.0
    last_upd = -1.0
    while True:
        t = time.time() - t0
        if t >= 12.0:
            break
        if t - last_upd >= 0.25:  # GPredict-ish ~4 Hz target refresh
            s.sendall(f"P {180.0 + t:.4f} {60.0 + 0.5 * t:.4f}\n".encode())
            r = s.recv(64); assert b"RPRT 0" in r, r
            last_upd = t
        time.sleep(0.1)
        s.sendall(b"p\n")
        a, e = readline_parts(s, b"\n", 2).split()[:2]
        tp = time.time() - t0  # position reflects ~now
        err = ((float(a) - (180.0 + tp)) ** 2 + (float(e) - (60.0 + 0.5 * tp)) ** 2) ** 0.5
        if tp > 3.0:  # steady state after acquisition transient
            err_max = max(err_max, err)
    print(f"sweep tracking: steady-state max error {err_max:.3f} deg")
    assert err_max < 0.15, "steady-state tracking error too large"

    # below-horizon command must be refused (motors must not chase)
    s.sendall(b"P 90 -40\n"); assert b"RPRT 0" in s.recv(64)  # accepted, gated internally
    time.sleep(1.0)

    # stop, then reset
    s.sendall(b"S\n"); assert b"RPRT 0" in s.recv(64)
    s.sendall(b"\\stop\n"); assert b"RPRT 0" in s.recv(64)
    s.sendall(b"R\n"); assert b"RPRT 0" in s.recv(64)
    s.sendall(b"\\get_info\n")
    info = s.recv(128)
    assert b"sat-eqmod" in info, info
    print("stop/reset/get_info OK")

    s.sendall(b"q\n")
    time.sleep(0.2)
    print("SMOKE PASS")

if __name__ == "__main__":
    sys.exit(main() or 0)
