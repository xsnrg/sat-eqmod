#!/bin/bash
# watch_events.sh LOG [MAXMIN] [STORM_PER_MIN] — event-driven daemon-log
# monitor (the real-time watch pattern adopted 2026-11-20: run in the
# background from the agent side; the agent is woken by the shell-completion
# notification, reports/acts, and relaunches — no polling sleeps, and the
# conversation stays unblocked between events).
#
# v2 (same night): routine "wire stop/restart" census lines do NOT wake —
# at 5-10 :K/min during normal passes they paged the agent constantly.
# Instead a STORM watchdog: if the combined TICK srk= totals grow by more
# than STORM_PER_MIN (default 20, the documented storm bar) between 60 s
# checks, it wakes and prints the census. Real events (state transitions,
# handover/re-home narratives, latches, watchdog, client changes) always
# wake, printed verbatim.
#
# Filters: TICK rows (10 Hz tracing) and the periodic 600 s "status:"
# summaries; "state:" transition lines DO pass. Byte-offset incremental
# tail for the event delta; grep of only the last TICK row for srk.
LOG="${1:?usage: watch_events.sh LOG [MAXMIN] [STORM_PER_MIN]}"
MAXMIN="${2:-45}"; STORM_PER_MIN="${3:-20}"
base=$(wc -c < "$LOG")
end=$(( $(date +%s) + MAXMIN*60 ))
# srk totals as one scalar sum (RA+DEC); awk does the arithmetic so the
# parser can't hand bash a malformed $(( )).
srk_sum() { grep "TICK" "$LOG" | tail -1 | grep -oE "srk=[0-9]+,[0-9]+" \
            | head -1 | cut -d= -f2 | awk -F, '{print $1+$2+0}'; }
srk_prev=$(srk_sum); [ -n "$srk_prev" ] || srk_prev=0
last_check=$(date +%s)
while [ "$(date +%s)" -lt "$end" ]; do
  sleep 5
  now=$(date +%s)
  cur=$(srk_sum)
  if [ -n "$cur" ] && [ $((now - last_check)) -ge 60 ]; then
    d=$(( cur - srk_prev ))
    rate=$(( d * 60 / (now - last_check) ))
    if [ "$rate" -gt "$STORM_PER_MIN" ]; then
      echo "STORM: srk +$d in $((now-last_check))s (~$rate/min):"
      grep -vE "\[TICK\]| status: " "$LOG" | tail -3 | cut -c1-200
      exit 0
    fi
    srk_prev="$cur"; last_check=$now
  fi
  out=$(tail -c +$((base+1)) "$LOG" 2>/dev/null | grep -v TICK | grep -v " status: " | grep -v "wire stop/restart" | grep -v "^$")
  if [ -n "$out" ]; then
    echo "NEW-EVENTS:"; echo "$out" | tail -8 | cut -c1-220
    exit 0
  fi
done
echo "TIMEOUT ${MAXMIN}m: quiet (no real events, no storm)"
