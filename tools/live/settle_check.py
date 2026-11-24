import socket, time

S = socket.create_connection(("127.0.0.1", 4533), timeout=3)
buf = b""

def send(cmd):
    S.sendall((cmd + "\n").encode())

def read_line(tmo=2.0):
    global buf
    S.settimeout(tmo)
    while True:
        idx = buf.find(b"\n")
        if idx >= 0:
            line = buf[:idx].decode()
            buf = buf[idx+1:]
            return line
        chunk = S.recv(4096)
        if not chunk:
            raise RuntimeError("closed")
        buf += chunk

def read_pos():
    # rotctld 'p' reply = two lines az\nel, NO RPRT line
    send("p")
    return float(read_line()), float(read_line())

def run(tag, target_el, dur=70.0):
    print(f"=== {tag}: target (90, {target_el})", flush=True)
    t0 = time.monotonic()
    inside_t = None
    samples = []
    lastint = None
    while time.monotonic() - t0 < dur:
        t = time.monotonic() - t0
        send(f"P 90 {target_el}")
        read_line()                       # consume RPRT 0
        time.sleep(0.3)
        az, el = read_pos()
        err = abs(el - target_el)
        samples.append((t, err))
        if inside_t is None and err < 0.05:
            inside_t = t
        if inside_t is not None and t - inside_t > 25:
            break
        ti = int(t)
        if ti != lastint:
            print(f"t={t:5.1f} el={el:7.3f} err={err:.4f}", flush=True)
            lastint = ti
    if inside_t is not None:
        tail = [s for s in samples if s[0] > inside_t]
        if len(tail) > 10:
            print(f"tail after band-entry: err {tail[0][1]:.4f} -> {tail[-1][1]:.4f} "
                  f"over {tail[-1][0]-tail[0][0]:.0f} s")
        print(f"{tag}: entered 0.05 deg band at t={inside_t:.1f} s; "
              f"final err {samples[-1][1]:.4f}", flush=True)
    else:
        print(f"{tag}: NEVER entered 0.05 band! final err {samples[-1][1]:.4f}", flush=True)

run("descend", 0.0)
time.sleep(2)
run("ascend", 10.0)
send("q")
S.close()
