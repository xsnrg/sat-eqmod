# GT2 pass session 2026-11-20 05:16-06:00Z — RS-44 (daemon e009afd, FIRST live pass of the band-edge mgate fix)

`rs44-2026-11-20-pass.csv` — 253 target pushes recorded by sateqd --record
(daemon log archived gzipped in ../../evidence/daemon-1120f.log.gz;
pre-:R latch snapshot in daemon-1120f-latch.log.gz).

Daemon flags: --mount sirius --orientation altaz --debug --record.
Shipped law under test: high_gate_moving_x=90 default (band-edge relay fix),
cold-HIGH skip blind, coast gate 0.3. No profile overrides.

## What this dataset is evidence FOR
1. **Band-edge relay KILLED (fix validated live)**: steady pass-phase
   3-5 :K/min (storm bar 23-26). Pre-aim dive = 4 events total (era prior:
   100+). Operator verdict: "remarkable improvement".
2. **NEW FALSE-LATCH, the other side of the same fix**: DEC parked at
   +100x = top of the LOW slow-ramp band (mgate 90 kept it LOW, correct
   under the new law) -> vintage board phase-losses *sustained* there
   (~0.8 floor, does NOT converge; contradicts the Phase-1 short-run
   "converges in 15-25 s" model) -> skip deficit grew past the 25 s grace
   (ramp_age 48 s) -> latched at el 68 mid-pass. R cleared it; 52 deg
   catch-up dive converged. Members: 11-17 sweep latch, LUCA dwell, today.
   Fix candidates (under research): sustained-deficit band-aware skip
   expectation, LOW demand clamp below ~55x (delivered 1.0 for certain),
   or auto-R on self-consistency.
3. **LOS branch-flip oscillation (NEW BUG CLASS)**: flip transitions at
   05:28:23 (0->1 pre-aim, correct), **05:53:00 flip=1->0 at el ~1.3
   DESCENDING** = select_branch nearest-branch firing mid-descent through
   the DEC fusion: setpoint rewritten ~180 deg in axis space while GT2
   pushed -> visible back-and-forth. Also a 05:59:58 0->1 blip on a stale
   tick. select_branch needs hysteresis (or branch lock while moving).
4. GT2 push tail near LOS: el clamped 0 keeps pushes coming after real
   LOS (its follow gate treats el-0 as alive) — delays stale-halt;
   GT-team report item (with the double-push pairs: recheck this file).

## Census
Per-minute :K events: pre-aim 4; AOS dive 05:34-35 = 32+22 (legitimate
catch-up); steady pass 3-5/min; latch window 05:43-45; post-R descent+
LOS wiggle 05:52=43, 05:53=16, 05:54=36 (the branch-flip pumping).
Total 186 for the whole night.
