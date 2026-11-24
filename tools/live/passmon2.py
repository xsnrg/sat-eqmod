#!/usr/bin/env python3
"""Pass monitor v2: READ-ONLY rotctld poller (never pushes targets).

Every POLL_S seconds sends `p` (get position) to the sateqd rotctld port,
logs reply latency plus the reported az/el, and flags any spike over 0.3 s.
CSV columns: utc_mono, p_lat_ms, az, el, err_flag.
Stdout: one line per poll, plus per-60 s latency summaries.
Usage: passmon2.py [host] [duration_s] [port]
"""
import socket
import sys
import threading
import time

def main():
    args = sys.argv[1:]
    HOST = args[0] if args else "127.0.0.1"
    dur = float(args[1]) if len(args) > 1 else 4200.0
    port = int(args[2]) if len(args) > 2 else 4533
    print("az,el,utc_mono,p_lat_ms", flush=True)
    end = time.monotonic() + dur
    spikes = 0
    polls = 0
    lat_sum = 0.0
    lat_max = 0.0
    win_start = time.monotonic()
    last_line = None
    while time.monotonic() < end:
        t0 = time.monotonic()
        try:
            s = socket.create_connection((HOST, port), timeout=1.0)
            s.settimeout(1.0)
            tc = time.monotonic()
            s.sendall(b"p\n")
            r = b""
            lines = 0
            while lines < 2:
                ch = s.recv(1)
                if not ch:
                    break
                r += ch
                if ch == b"\n":
                    lines += 1
            s.close()
        except Exception as e:
            print(f"conn-fail {e}", file=sys.stderr, flush=True)
            time.sleep(0.2)
            continue
        t1 = time.monotonic()
        lat = (t1 - tc) * 1000.0   # server response time (post-connect)
        polls += 1
        lat_sum += lat
        lat_max = max(lat_max, lat)
        txt = r.decode(errors="replace").strip()
        parts = txt.split()
        if len(parts) >= 2:
            line = f"{parts[0]},{parts[1]},{time.time():.3f},{lat:.1f}"
            if line != last_line:      # dense-change logging
                print(line, flush=True)
                last_line = line
        else:
            print(f"bad-reply:{txt!r},{time.time():.3f},{lat:.1f}", flush=True)
        if lat > 300.0:
            spikes += 1
            print(f"SPIKE {lat:.0f}ms at {time.time():.3f}", file=sys.stderr,
                  flush=True)
        if time.monotonic() - win_start >= 60.0:
            print(f"SUMMARY win_start={win_start:.0f} polls={polls} "
                  f"avg={lat_sum/max(polls,1):.1f}ms max={lat_max:.0f}ms "
                  f"spikes_total={spikes}", flush=True)
            win_start = time.monotonic()
            polls = 0
            lat_sum = 0.0
            lat_max = 0.0
        time.sleep(0.1)

main()
