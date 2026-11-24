#!/bin/bash
# Sync test: camera watch vs real mount motion through the live daemon.
cd ~/sateq-cam
EYES_ROI=0:0.52:0:1 timeout 22 python3 -u eyes_cv.py watch 20 2>/dev/null > /tmp/watch.log &
WATCH=$!
sleep 2
python3 - <<'PY' 2>&1 | sed 's/^/client: /'
import socket, time
s = socket.create_connection(("localhost", 4533), timeout=5)
s.settimeout(3)
def cmd(c):
    s.sendall((c + "\n").encode())
    try:
        return s.recv(4096).decode().strip()
    except Exception as e:
        return f"<{e}>"
print("dump ok:", bool(cmd("\\dump_state")))
print("P 90 4 ->", cmd("P 90 4"))
t0 = time.time()
while time.time() - t0 < 8:
    print("p:", cmd("p")); time.sleep(0.5)
print("P 90 0 ->", cmd("P 90 0"))
t0 = time.time()
while time.time() - t0 < 8:
    print("p:", cmd("p")); time.sleep(0.5)
s.close()
print("client done")
PY
wait $WATCH
echo "=== watch lines around motion windows ==="
grep -c MOVED /tmp/watch.log
awk 'NR%8==1' /tmp/watch.log
