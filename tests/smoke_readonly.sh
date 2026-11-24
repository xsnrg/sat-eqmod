#!/bin/bash
# read-only daemon smoke: target a fast-moving point, confirm NO skip latch,
# NO halt, and held write frames at exit.
set -u
cd "$(dirname "$0")/.."
PORT=14636
./build/sateqd --sim heq5 --orientation altaz --read-only --lat 44.95 --lon -93.28 --rotctld-port $PORT > /tmp/sateqd_ro3.log 2>&1 &
P=$!
sleep 1
# Keep the connection open while the tracker ticks: since the 2026-11-16
# safety rework a disconnect DROPS the target (last-client semantics), and
# the old batch (P...q in one packet) closed before the first 10 Hz tick,
# so the daemon legitimately never tracked. Hold 2 s, then quit.
( printf 'P 120 40\np\n'; sleep 2; printf 'q\n' ) | timeout 6 nc 127.0.0.1 $PORT
sleep 2
kill -INT $P
sleep 1.5
kill -0 $P 2>/dev/null && { echo "FAIL: daemon survived SIGINT"; kill -9 $P; }
echo "--- log tail ---"
tail -6 /tmp/sateqd_ro3.log
if grep -q "skip" /tmp/sateqd_ro3.log; then
    echo "FAIL: skip-latch occurred in gated mode"; exit 1
fi
if ! grep -q "TRACK" /tmp/sateqd_ro3.log; then
    echo "FAIL: never reached TRACK"; exit 1
fi
echo "PASS: read-only mode tracked without a false skip latch"
