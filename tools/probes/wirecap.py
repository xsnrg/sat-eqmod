import socket, threading
srv = socket.socket(); srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(("127.0.0.1", 4601)); srv.listen(1)
c, a = srv.accept()
c.settimeout(3)
try:
    while True:
        b = c.recv(4096)
        if not b: break
        print("RECV:", repr(b))
except Exception as e:
    print("done", e)
