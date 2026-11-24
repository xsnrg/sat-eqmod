import socket, time
s = socket.create_connection(("127.0.0.1", 4533), timeout=3)
t0 = time.monotonic()
while time.monotonic() - t0 < 75:
    try:
        s.sendall(b"P 90 0\n"); s.recv(64)
        time.sleep(0.9)
        s.sendall(b"p\n")
        r = s.recv(64).decode().strip().replace("\n", " ")
        print("%5.1f %s" % (time.monotonic() - t0, r), flush=True)
    except Exception as e:
        print("%5.1f STALL %s" % (time.monotonic() - t0, e), flush=True)
        break
s.sendall(b"q\n")
