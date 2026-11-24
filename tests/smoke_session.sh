#!/bin/bash
# End-to-end session-layer smoke: --mount NAME resolves station + profile,
# wrong hardware is refused, session state persists and resumes across a
# daemon restart. Uses an isolated HOME so the real ~/.config is untouched,
# and the built-in simulator (no physical motion ever possible).
set -u
cd "$(dirname "$0")/.."
BIN=./build/sateqd
H=$(mktemp -d /tmp/sateq-smoke.XXXXXX)
trap 'rm -rf "$H"' EXIT
export HOME="$H"
mkdir -p "$H/.config/sateq/mounts"

BASE_PORT=${BASE_PORT:-14640}
fail() { echo "FAIL: $*"; exit 1; }

cat > "$H/.config/sateq/station.conf" <<EOF
lat = 44.95
lon = -93.28
EOF

cat > "$H/.config/sateq/mounts/testheq5.conf" <<EOF
port = /dev/serial/by-id/usb-FTDI_TTL232R_TEST-if00-port0
mount_code = 0x01
orientation = altaz
EOF

cat > "$H/.config/sateq/mounts/testeq8.conf" <<EOF
port = /dev/serial/by-id/usb-FTDI_TTL232R_TEST-if00-port0
mount_code = 0x04
orientation = altaz
EOF

# ---- 1. profile resolution + state save ------------------------------------
P=$((BASE_PORT))
$BIN --mount testheq5 --sim heq5 --rotctld-port $P > /tmp/sateqd_s1.log 2>&1 &
PID=$!
sleep 1
printf 'P 120 40\nq\n' | timeout 3 nc 127.0.0.1 $P > /dev/null
sleep 1
kill -INT $PID; sleep 1.5
kill -0 $PID 2>/dev/null && fail "daemon 1 survived SIGINT"
grep -q "mount profile 'testheq5'" /tmp/sateqd_s1.log || fail "profile not resolved"
grep -q "mount code 0x01" /tmp/sateqd_s1.log || fail "sim mount code wrong"
grep -q "saved session state" /tmp/sateqd_s1.log || fail "state not saved"
test -f "$H/.local/state/sateq/testheq5.state" || fail "state file missing"
echo "1 ok: profile + state save"

# ---- 2. resume: counters unchanged (fresh sim, never moved) -----------------
P=$((BASE_PORT+1))
$BIN --mount testheq5 --sim heq5 --rotctld-port $P > /tmp/sateqd_s2.log 2>&1 &
PID=$!
sleep 1
kill -INT $PID; sleep 1.5
grep -q "resumed session: counters" /tmp/sateqd_s2.log || { tail -12 /tmp/sateqd_s2.log; fail "did not resume"; }
grep -q "resumed: reference pointing" /tmp/sateqd_s2.log || fail "no resumed log line"
echo "2 ok: resume continues budget"

# ---- 3. --no-resume ignores state -------------------------------------------
P=$((BASE_PORT+2))
$BIN --mount testheq5 --no-resume --sim heq5 --rotctld-port $P > /tmp/sateqd_s3.log 2>&1 &
PID=$!
sleep 1
kill -INT $PID; sleep 1.5
grep -q "calibrated: reference pointing" /tmp/sateqd_s3.log || fail "--no-resume still resumed"
grep -q "resumed session" /tmp/sateqd_s3.log && fail "--no-resume resumed anyway"
echo "3 ok: --no-resume starts fresh"

# ---- 4. wrong hardware refused (profile expects 0x01, sim says 0x04) --------
P=$((BASE_PORT+3))
$BIN --mount testeq8 --sim heq5 --rotctld-port $P > /tmp/sateqd_s4.log 2>&1 &
PID=$!
sleep 1.5
kill -0 $PID 2>/dev/null && fail "daemon ran despite mount code mismatch"
grep -q "refusing to move it" /tmp/sateqd_s4.log || { cat /tmp/sateqd_s4.log; fail "no refusal message"; }
echo "4 ok: wrong mount code refused"

# ---- 5. read-only session never writes state ---------------------------------
rm -f "$H/.local/state/sateq/testheq5.state"
P=$((BASE_PORT+4))
$BIN --mount testheq5 --read-only --sim heq5 --rotctld-port $P > /tmp/sateqd_s5.log 2>&1 &
PID=$!
sleep 1
kill -INT $PID; sleep 1.5
test -f "$H/.local/state/sateq/testheq5.state" && fail "read-only wrote a state file"
grep -q "READ-ONLY session" /tmp/sateqd_s5.log || fail "no read-only banner"
echo "5 ok: read-only leaves state untouched"

# NOTE: the re-anchor save fix (state stores the LIVE counters + pose at
# exit, not the calibration pair — live bug 2026-11-18: restart after a dive
# fresh-calibrated at home) is NOT smoke-testable against --sim: a fresh
# simulator process resets counters to 0x800000, which is the same false
# "mount moved while down" a real power-cycle correctly refuses. Real
# validation = daemon restart mid-sky with mount powered (resume expected).
echo "PASS: session smoke (profile/resume/no-resume/mismatch/read-only)"
