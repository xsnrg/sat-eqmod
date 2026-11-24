#!/bin/bash
# End-to-end --mount auto smoke with NO hardware: two socat pty pairs stand
# in for the two FTDIs; a python fake board answers the read-only identity
# query with the LIVE Sirius golden (=020401, HEQ5 0x01) on one end while the
# other end stays silent (dead mount). sateqd --mount auto must pick sirius,
# connect, and calibrate. Isolated HOME; nothing real is touched.
set -u
cd "$(dirname "$0")/.."
BIN=./build/sateqd
H=$(mktemp -d /tmp/sateq-ad.XXXXXX)
trap 'rm -rf "$H"' EXIT
export HOME="$H"
mkdir -p "$H/.config/sateq/mounts"
fail() { echo "FAIL: $*"; pkill -f "sateq-ad" 2>/dev/null; exit 1; }

command -v socat >/dev/null || { echo "SKIP: socat missing"; exit 0; }

printf 'lat = 44.95\nlon = -93.28\n' > "$H/.config/sateq/station.conf"
printf 'port = %s/live\nmount_code = 0x01\norientation = altaz\n' "$H" \
    > "$H/.config/sateq/mounts/sirius.conf"
printf 'port = %s/dead\nmount_code = 0x04\norientation = altaz\n' "$H" \
    > "$H/.config/sateq/mounts/hdx.conf"

# PTY pairs: fake board talks to $1-side links, daemon opens the other side.
socat -d "pty,raw,echo=0,link=$H/liveA" "pty,raw,echo=0,link=$H/live" \
    2> /dev/null & P1=$!
socat -d "pty,raw,echo=0,link=$H/deadB" "pty,raw,echo=0,link=$H/dead" \
    2> /dev/null & P2=$!
for i in $(seq 1 40); do [ -e "$H/live" ] && [ -e "$H/dead" ] && break; sleep 0.1; done
[ -e "$H/live" ] || fail "socat pty never appeared"

# Fake Sirius: CR-terminated SkyWatcher frames, read-only goldens from the
# live bench (see AGENTS command library); anything else ACKs "=".
python3 - "$H/liveA" <<'PYEOF' &
import os, sys, termios, tty
path = sys.argv[1]
fd = os.open(path, os.O_RDWR | os.O_NOCTTY)
tty.setraw(fd)
GOLD = {b'e': b'020401', b'a': b'00B289', b'b': b'A7FD00', b'g': b'10',
        b's': b'1C0501', b'f': b'100', b'j': b'000080', b'D': b'6C0200'}
buf = b''
while True:
    try:
        d = os.read(fd, 64)
    except OSError:
        break
    if not d:
        break
    buf += d
    while b'\r' in buf:
        frame, buf = buf.split(b'\r', 1)
        if not frame.startswith(b':') or len(frame) < 2:
            continue
        cmd = frame[1:2]
        if cmd == b'q':
            os.write(fd, b'!0\r')            # no EQ8 features (live Sirius)
        elif cmd in GOLD:
            os.write(fd, b'=' + GOLD[cmd] + b'\r')
        else:
            os.write(fd, b'=\r')
PYEOF
FAKE=$!
sleep 0.5

P=14700
$BIN --mount auto --rotctld-port $P > /tmp/sateqd_ad.log 2>&1 &
PID=$!
sleep 3
kill -0 $PID 2>/dev/null || { cat /tmp/sateqd_ad.log; fail "auto daemon died"; }
grep -q "mount auto: selected profile 'sirius'" /tmp/sateqd_ad.log \
    || { cat /tmp/sateqd_ad.log; fail "auto did not select sirius"; }
grep -q "calibrated: reference pointing" /tmp/sateqd_ad.log \
    || fail "auto session did not calibrate"
kill -INT $PID; sleep 1.5
kill $P1 $P2 $FAKE 2>/dev/null
echo "ok: auto selected the profile whose fake board answered, refused the silent port"
echo "PASS: auto-discovery smoke (pty fake board, no hardware)"
