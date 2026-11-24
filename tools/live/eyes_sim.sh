#!/bin/bash
# scripted oracle for the eqtool backlash wizard exercise (simulator only).
# Wizard phase pattern per (axis,dir): reset#odd=run-in window (one query,
# must confirm motion), reset#even=b window (1st burst still, 2nd moved).
D=/tmp/opencode/bl_smoke
case "$1" in
  reset)
    r=$(( $(cat "$D/r" 2>/dev/null || echo 0) + 1 )); echo $r > "$D/r"
    echo 0 > "$D/q"; exit 0 ;;
  query)
    r=$(cat "$D/r" 2>/dev/null || echo 0); q=$(( $(cat "$D/q") + 1 )); echo $q > "$D/q"
    if [ $((r % 2)) -eq 1 ]; then exit 0; fi   # run-in confirm: MOVED
    [ "$q" -eq 1 ] && exit 1                    # first burst: riding on slack
    exit 0                                      # later burst: MOVED
    ;;
esac
exit 2
