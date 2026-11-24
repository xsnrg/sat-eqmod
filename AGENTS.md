# sat-eqmod — engineering knowledge base

Standalone C++ daemon that satellite-tracks with Synta/SkyWatcher EQ-class mounts
over the motor-board "handset port" protocol, and exposes a **Hamlib `rotctld`
TCP server** so stock tracker apps (GPredict, GridTracker2, any rotctld client)
can drive it.

This file distills the project's protocol knowledge, board-behavior measurements,
control laws, and standing lessons — everything a contributor or the next
session needs to change the code safely. (A separate private bench journal holds
the full day-by-day incident forensics and machine state; nothing needed to
build, test, or understand the laws lives only there.)

## Goals & requirements

- Control mount RA/DEC motors directly (no hand controller in the loop).
- Mount must stay drivable by existing standard trackers → speak rotctld to
  the app side and the SkyWatcher native protocol to the mount side.
- First target: satellite (LEO) tracking with a Yagi; AZ/EL → HA/DEC steering
  at ~10 Hz, park/home + e-stop + watchdogs.

## Hardware inventory

- **Orion/SkyWatcher Sirius EQ-G** — EQ5-class SynScan motor board. HC firmware
  v4 series, ver 4.37.03 (leave HC firmware alone; the project never talks to HC).
- **Orion HDX-110 EQ-G** (original, no R/v2) — SynScan-based, reports motor
  board code **0x04 = "EQ8"** in indi-eqmod's mount table.
- FTDI TTL232R / FT232R breakouts are the known-good cables (resolve by
  `/dev/serial/by-id/...` — ttyUSB numbers swap with plug order). A USB2EQ5
  (FTDI inside, RJ45 to handset jack) needs a custom Windows VCP driver; on
  Linux bind via `ftdi_sio/new_id` if no tty appears.
- A counterfeit PL2303 "PC cable" exists in the ecosystem and has dead TX on
  Linux — do not trust PL2303 dongles as a reference device.
- If `gpsd` is installed (socket-activated on some distros), it will
  snatch/reconfigure `/dev/ttyUSB*` between scans. Stop and mask it before
  serial bring-up.

## The protocol — THE critical learning

The handset-port wire protocol is the **SkyWatcher native motor-board protocol**,
NOT anything LX200-ish:

- Frame: `:` + cmd-char + axis-digit (`1`=RA/AZ, `2`=DEC/ALT) + optional arg + **CR (0x0d)**.
- Responses are **CR-terminated**, first char `=` = OK (then data),
  `!`+code+CR = error NAK.
- 9600 8N1, no handshake lines needed (FTDI TX/RX/GND only).
- **`#` never appears as a terminator.** A board asked an LX200-style `:GR#`
  sees an incomplete command and says nothing — total silence with a healthy
  cable + mount. Do not conclude "mount is dead" from LX200 probes.

Command set (from indi-eqmod `skywatcher.h`, official SkyWatcher protocol):

| char | name / meaning |
|------|----------------|
| `F`  | Initialize (zero position counters) — also ARMS the axis for `:J` |
| `e`  | Inquire motor board version → lower byte of BCD-swapped answer = mount code |
| `a`  | Grids (steps) per revolution (per axis) |
| `b`  | Timer interrupt frequency (Hz) |
| `g`  | High-speed ratio (MCU ticks per high-speed step, per axis) |
| `s`  | PEC period (units vendor-unclear; treat as opaque payload) |
| `L`  | Instant axis stop |
| `K`  | Not-instant axis stop |
| `E`  | Set axis position (step count) |
| `j`  | Get axis position (step count) |
| `f`  | Get axis status (accel/decel/slew phases) |
| `O`  | Set snap port (EQ8/AZEQ6/AZEQ5/EQ6-R only) |
| `G`  | Set motion mode |
| `H`  | Set GOTO target increment (per-step table) |
| `M`  | Set break-point increment |
| `S`  | Set GOTO target (absolute step count) |
| `U`  | Set break step |
| `I`  | **Set step period** (how slew/tracking speed is set: µs between steps) |
| `J`  | **Start motion** (direction + speed-mode arg) |
| `D`  | Get step period (µs, revu24) — confirmed live |
| `B`  | Activate motor (eq6direct); `:Bn1` is SLEEP per vendor doc — never sent |
| `P`  | Set ST4 guide rate |
| `d`  | Get home position encoder count (EQ8 aux encoder) |
| `W`  | Set feature bits (EQ8/AZEQ6/AZEQ5) |
| `q`  | Get feature bits (EQ8/AZEQ6/AZEQ5) |
| `V`  | Set polar-scope LED |

NAK catalog (vendor "motor controller command set" PDF, mirrored in
`protocol.h`): 0 unknown/not-on-model, 1 length error, 2 motor not stopped,
3 non-uppercase-hex char, 4 not initialized, 5 driver sleeping, 7 PEC training,
8 no PEC data. Also `:Q1 55AA` = **run bootloader (no response)** — a foot-gun;
it stays behind the write gate forever.

indi-eqmod specifics worth copying:

- Handshake: `:Ge1` + CR; version parsed as BCD, byte-swapped
  (`(v&0xFF)<<16 | v&0xFF00 | (v&0xFF0000)>>16`), `MountCode = MCVersion & 0xFF`.
- Mount codes: `0x00 EQ6, 0x01 HEQ5, 0x02 EQ5, 0x03 EQ3, 0x04 EQ8, 0x05 AZEQ6,
  0x06 AZEQ5, 0x0A Star Adventurer, 0x0C SA-GTi, 0x20 EQ8-R Pro, 0x22 AZEQ6 Pro,
  0x23 EQ6-R Pro, 0x24 EQ6 Pro, 0x25 CQ350 Pro, 0x31 EQ5 Pro, 0x80 GT, 0x81 MF,
  0x90 DOB, 0xA5 AZ-GTi, 0xF0 GEEHALEL`.
- EQ8-class feature set: code in {0x04,0x05,0x06,0x0A,0x0C,0x23,0x24,0xA5}.
- Position/RA/DEC are **derived from step counts** — the board only counts
  steps. Grids-per-rev (`:Ga1/2`), timer freq (`:Gb1/2`) and high-speed ratio
  (`:Gg1/2`) give arcsec/step; the daemon does the astronomy.
- indi-eqmod retries dispatches and reads until CR; every write is preceded by
  `tcflush(TCIOFLUSH)`.

## Daemon handshake recipe (validated on both boards)

Send `:e1\r`; MountCode = low byte of the 6-hex-digit payload (HDX 04=EQ8,
Sirius 01=HEQ5). Then probe `:q1010000\r` — ACK ⇒ EQ8-class features available
(snap/aux encoder/feature bits), `!0` NAK ⇒ classic path.

## Verified electrical facts (Sirius EQ-G handset jack)

RJ45 socket, pins facing you, tab at bottom (Cat5B colors through a breakout,
OW = pin 1):

| pin | color | V   | function |
|-----|-------|-----|----------|
| 1   | OW    | 0   | GND |
| 2   | O     | 0   | — |
| 3   | GW    | 0   | — |
| 4   | Bl    | 0   | GND |
| 5   | Bl-W  | **5.0** | mount TX (idle high — mount CPU UART alive) |
| 6   | G     | 2.4 | mount RX (biased input) |
| 7   | WtBr  | **12** | +12 V |
| 8   | Br    | **12** | +12 V |

Modern SkyWatcher convention confirmed. **Never connect pins 7/8 to FTDI.**
FTDI wiring: TX→pin 6, RX→pin 5, GND→1/4. The HDX-110 jack is the same pin
pattern at **3.3 V logic** — use a 3.3 V FTDI and/or ~1 kΩ series resistors on
PC→mount TX there; Sirius is native 5 V. The jack's 12 V rail dies with the
mount power switch, which powers interface breakouts — check the mount lamp
before believing any silence.

## Safety lessons (hard-won)

- **Do not poke the hand-controller's PC port with unknown bytes.** A
  `00000000` probe tripped a SynScan HC firmware-update bootloader once; the HC
  front panel latched until a power cycle.
- Old mounts: **feel-test both axes before high-rate (40–100× sidereal) runs.**
  A stiff/cable-flex RA axis can look exactly like motor death.
- **Two processes must never own the serial port.** A daemon-restart race once
  corrupted one `:j` reply and `terminate()`d the daemon on the uncaught
  exception. Guards: `Board::position` retries short ACKs; rotctld handlers
  catch and answer `RPRT -1`; restart procedure = SIGTERM old daemon, WAIT for
  the 4533 listener to drop, THEN start the new one.
- Never run `eqtool query` against a port a live daemon owns — read through
  rotctld instead.
- pkill patterns over ssh must not appear literally in the same command line
  (including log filenames) — they match and kill your own shell. Use bracketed
  patterns or kill by PID.
- Before ANY daemon kill on a live bench, check the last tick rows for MOVING
  counters; if moving, wait for park/stale-halt or have the client drop the
  target first. `calibrate` means "the tube IS at stated home" — it trusts,
  never verifies.

## Ecosystem / references

- **indi-eqmod source**: `github.com/indilib/indi-3rdparty` → `indi-eqmod/`:
  `skywatcher.cpp` protocol impl, `skywatcher-simulator.cpp` is an excellent
  exact-response reference (our own emulator mirrors it).
- **Hamlib** has a native SkyWatcher rotator backend: `rotctl -m 2801` —
  speaks this same motor-board protocol; good independent comms verification.
- EQModLX (eq-mod.sourceforge.net) = LX200 emulator with satellite-mode docs —
  reference for how GPredict-style apps expect tracking to behave.
- **Sky-Watcher official motor-controller command doc** (vendor PDF,
  definitive for 2.xx boards: NAK codes, `:f` bit decode, high-speed = ratio
  steps/interrupt):
  inter-static.skywatcher.com/downloads/skywatcher_motor_controller_command_set.pdf.
- **GSServer** (ASCOM SkyWatcher driver, github.com/rmorgan001/GSServer) and
  **tianwen** GitHub issues (live AZ-GTi measurements) cross-confirm these wire
  findings.
- Cloudy Nights is 403 to plain fetch; use search snippets.

## READ-ONLY COMMAND LIBRARY — live Sirius goldens

`eqtool query` (write gate forced ON) against the real board:

| cmd | reply | notes |
|-----|-------|-------|
| `:e1` | `020401` | code 0x01 HEQ5 |
| `:q1010000` | NAK `0` | no EQ8 features — arbiter for home_sensor/`:O`/`:W` |
| `:a1/2` | `00B289` | 9,024,000 steps/360° |
| `:b1/2` | `A7FD00` | 64,935 steps/worm |
| `:g1/2` | `10` | ratio 16 |
| `:s1/2` | `1C0501` | units vendor-unclear (raw payload only — do NOT revu24-decode without evidence) |
| `:f1/2` | `100` | idle word |
| `:j1/2` | `000080` | 0x800000 both axes — **counters are volatile across power cycles**; resume state must expect fresh-counter starts |
| `:D1/2` | `6C0200` | 620 µs = 1× sidereal default. `:D` = current step period in µs (revu24) — CONFIRMED |
| `:d1/2` | `000080` | HEQ5 `:d` ACKs with a `:j`-identical value — do NOT treat a `:d` ACK as home-encoder presence; trust only the `:q` probe |

Golden numbers: `revu24_parse("00B289")=9024000` (steps_360),
`("A7FD00")=64935` (steps_worm, = 0x00FDA7 nibble-pair swap — NOT 53415).
1× sidereal on Sirius = period 620 µs, low-speed.
2^24 steps = 669.46 AXIS deg (counter deltas wrap mod 2^24; wrap to ±2^23
before dividing).

## Board behavior (measured, both boards)

**`:F` arming**: `:J` NAKs `!4` ("Not Initialized") per axis until `:F` has been
sent once since board power-on. `:F` does NOT disturb the step counter.
`Board::start_motion` arms lazily (query `:f`, send `:F` if init bit clear,
cache per axis) and re-arms once on a `!4` at `:J`. `:E` does NOT arm.

**`:f` decode** (official + live-confirmed): 3 hex digits; digit1 = mode bits
(0x1 slew/GOTO, 0x2 backward, 0x4 high-speed), digit2 bit0 = RUNNING (bit1 =
blocked), digit3 bit0 = **initialized** (set by `:F`, persists across `:K`,
cleared only by board power cycle). Idle-fresh = `100`, armed-idle = `101`,
running = digit2 bit0 set.

**Running-axis wire rules** (measured; the simulator enforces all):

- `:G` NAKs `!2` while the axis runs.
- `:I` is ACKed and *immediately effective* while running in LOW speed;
  `:I` NAKs `!2` while running HIGH.
- `:J` while running ACKs idempotently.

`Board::slew_rate` obeys: identical program running → no wire traffic;
low-speed same-dir change → in-place `:I`; otherwise `:K` →
`wait_running_clear` (1000 ms low / 3000 ms high) → full reprogram → `:J`.

**Board has NO autonomous tracking**: no post-slew auto-track, no tracking-rate
command, nothing to "disable"; tracking *is* our `:I`+`:J` ownership. No
watchdog kills motion if `:J` isn't re-sent. Init bits + counters live in RAM:
power cycle ⇒ counters back to 0x800000 and axes disarmed.

**Acceleration / phase-loss**: the board accelerates into a commanded period
over ~15–25 s in the LOW 60–100× band (short-run delivered ratios read low:
0.90 at 80×, 0.70 at 100×, converging to 1.0 on ≥25 s runs); HIGH band ramps
in ~1 s. Any open-loop distance timing must add accel allowance; the
closed-loop tracker is immune (position feedback), but its rate estimate needs
≥200 ms sampling — sub-50 ms deltas are quantization garbage.

**Rate modes**: 104–128× sidereal band is promoted to HIGH speed (vendor
integer truncation would emit <6 µs LOW periods the stepper timer cannot
honor). `:I` accepts periods down to 5 µs without NAK; we keep a 6 µs LOW
floor. There is **no board speed ceiling** in our envelope (5000× free-runs);
600× is a chosen safe cap, not a limit.

**Deceleration tails (`:K`)**: speed-scaled — LOW ~0.03 s; HIGH ~1 s and
~18–19k steps (~0.74° at 600×). The tracker's stop-latency and coast
budgets model this; the simulator has `set_coast(axis, s)`.

**`:j` counts ISSUED pulses, not delivered motion.** Consequences:

- A skip/stall latch is *issuance starvation*, never "mechanical jam" (a
  slipping motor still counts).
- Backlash is fundamentally invisible to the wire (counter ticks on the slack
  flank). Any backlash tool needs an EXTERNAL oracle of physical motion.
- A hand-rotate moves the tube/cord with ZERO counter change — re-datum after
  any shaft work.

**Sign conventions**: `ra_sign`/`dec_sign` are PROFILE data — verify against
the sky, not the counters. (Sirius: `ra_sign = -1` — the tilt axis was
physically mirrored vs the model; see LESSONS.)

## WRITE GATE / READ-ONLY BENCH MODE

Enforced in CODE at the `Board` layer, not by convention:

- `protocol.h:is_query_command(c)` — whitelist of pure inquiries
  (`e a b g s j f D d q`). Everything else, **including `L`/`K` stops and
  `F`/`E` counter writes**, is a WRITE.
- `Board::block_writes(true)` — dispatch sends ONLY queries; a write's exact
  wire frame (minus CR) is pushed to `held_frames()` and a synthetic empty ACK
  returned. Queue capped (4096, drop-oldest).
- `sateqd --read-only` (config key `read_only`) turns the gate on at startup.
  It also sets `TrackerConfig::writes_gated`, which suppresses skip/stall
  latching (with writes gated, "commanded motion not delivered" is expected).
- `eqtool query` runs the FULL read-only battery and prints payload + revu24
  decode per line; `eqtool --hold-writes` dry-runs motion sequences.

## Control laws (current shipped law set)

Incoming rotctld telemetry is treated as **point-in-time samples of where we
should be** — between samples the mount runs a nearly constant slew rate
("run it like a telescope mount"), never dive-and-stop cycles.

- **Two modes, hysteresis**: DIVE (far: P + velocity feedforward on the
  extrapolated setpoint) / GLIDE (within `glide_engage_deg`=5°, leave at 8°):
  a rate-follower reference trajectory integrates
  `pattern_vel*taper + clamp(conv*(pat−ref_pos), ±conv_max)`; at steady state
  the commanded rate ≈ pattern rate. Engagement anchors the reference AT THE
  MOUNT (nearest-360°-turn alignment). Sample age past 2.5 s tapers prediction
  to zero; hard stop at 5 s — a silent client leaves the reference gliding to
  rest.
- **Pattern estimator = α-β filter on raw pushes** (α=1.0, β=0.5) + innovation
  gate (`pattern_gate_deg`=0.5 + rate·dt) + slow accel EMA, Jacobian-mapped to
  axis space. **Minimum estimator cadence** (`pattern_min_dt_s` = 1 s): pushes
  arriving sooner keep freshness bookkeeping but hold the filter; the next
  real update measures against the last filter time with the LATEST sample at
  full gain. (Client double-pushes 0.1–0.9 s apart previously kicked velocity
  sign flips → `:K` storms.)
- **Glide nudge = rate-level PI**: `rt = ref_vel + P(ref−cur) + I(des−cur)`.
  I MUST act on (des−cur), not (ref−cur) — the reference can sit in a
  self-consistent lag P cannot see. Tuned: `nudge_p_per_s`=2.0,
  `pattern_conv_k_per_s`=0.75, `nudge_i_per_s2`.
- **Glide I unwind bleed** (`nudge_i_unwind_gain` = 5.0): when the integral
  OPPOSES the present error AND the pattern is near-stationary, integrate the
  unwind 5× faster (drains dive-overshoot windup that otherwise strands settle
  at ~0.27° crawling 0.0075°/s, tau ~50 s → ~7 s). Pattern-speed gate
  mandatory (ungated it eats live tracking authority on moving patterns).
- **Chatter deadbands split by board band**: LOW speed (≤104×) takes in-place
  `:I` free (floor 0.05×, deadband 0 — any percentage band swallows the glide
  loop's small corrections and freezes transients into permanent offsets).
  HIGH keeps a 25 % reprogram deadband (`high_reprogram_db`; every change
  costs `:K`+coast).
- **HIGH permission gate** (`high_gate_moving_x` = 90.0): only patterns LOW
  (≤104×) physically cannot follow justify HIGH; the |err| >
  `high_gate_err_deg` (code default 2.0; bench profile 2.5 for throttled
  clients) exemption covers real catch-up. The old 3× moving-pattern
  exemption caused the LOW↔HIGH band-edge relay (~20 `:K`/min on normal
  passes, live-measured).
- **HIGH rescale direction-persistence** (`high_recommit_s` = 1.0 s): a
  same-direction rescale that touches HIGH and flips magnitude direction
  against the per-axis committed-direction ledger is held at the sent program
  until it persists (wire-free, pointing-neutral); a
  `high_recommit_hold_deg` (0.1 axis deg) budget shortens the window at high
  mismatch rates. Escapes: floor guard, |e| ≥ `high_recommit_escape_deg`
  (2.0), dive ticks (`!glide`), `high_recommit_s = 0` (rollback lever).
  ACCELERATION-ONLY flips outside a regime — holding a DECELERATION flip past
  the setpoint overshoots (direction-blind hold cost 2–7° sky at the
  1/sin(az) lever in fixtures).
- **Wobble-regime detector**: leaky rescale-COMMIT counter per axis (decay τ =
  `high_recommit_regime_s` 4 s) ≥ `high_recommit_regime_n` (3.0) AND a latched
  client-staleness gate (target age past `high_recommit_regime_age_s` 1.5 s).
  The pump is a THROTTLED-CLIENT phenomenon — fast-cadence rescale ladders are
  work, not disturbance. Inside a regime, acceleration-only FREEZE
  (`high_recommit_regime_db` default 1e9) gated by an error-GROWTH ledger
  (hold only once |e| stopped improving vs the last commit — pump commits land
  at error troughs; genuine convergent ladders walk error DOWN rung after rung).
- **Wobble-regime demand-LPF escalation**: while an axis's regime ledger
  `rc` ≥ `demand_lpf_regime_min_rc` (12.0, storm grade — the plain latch level
  3 arms on mid-grade dither where lag buys nothing) AND |axis err| ≤
  `demand_lpf_regime_release_err_deg` (1.0 — outrun patterns keep full
  unsmoothed authority), that axis's glide demand LPF runs at
  `demand_lpf_regime_tau_s` (1.0 s; base tau 0.3). Attacks the source: the
  demand dithers ±150× between 2 s/0.1°-quantized pushes levered 4–6× by the
  flip-branch Jacobian, and each crossing of the 25 % deadband commits ~1/s.
  A GLOBAL tau raise was measured and REJECTED (quiet-set regressions — LPF
  phase lag taxes every dive/reversal transient). TICK rows show `reg=`
  per-axis ledger + `lrpf` flag.
- **Reversal persistence**: a pattern sign flip must persist
  `reversal_confirm_s` (6.0 s) before the glide reference is retargeted, HOLD
  AT THE SENT PROGRAM meanwhile (wire-free, pointing-neutral), capped by
  `reversal_hold_deg` (0.5) budget = window/|sent rate| — near-zero dithers
  get the full window, HIGH flips auto-shorten. Floor-guard bypasses intact.
- **Coast-aware wire commits**: (1) landing gate — a brake commit (flip or
  stop) on a running HIGH axis is held while the counter-space error is
  farther ahead than the coast can travel (`high_coast_s` ~0.3,
  `coast_margin_deg` 0.3); (2) flip commits out of HIGH LAND IN LOW (≤104×):
  the flip absorbs the coast once; landing HIGH would re-coast — the pump
  cycle. Genuine >104× patterns re-escalate same-direction from LOW.
- **Floor guard = velocity projection** (`floor_el` = min_elev + 1.0 margin,
  `floor_coast_s` 1.2 s): demanded velocity projected onto the legal half-plane
  `el_dot >= (floor_el − cur_el)/floor_coast_s`; sinking component removed,
  tangential preserved → acquisition SLIDES ALONG the floor. Setpoint clamping
  was tried and REJECTED (feedforward still flung the mount down); joint
  rate-scaling cannot redirect. **Rate cap must be joint/direction-preserving**
  (per-axis clipping rotates the floor-slide back downward).
- **Waypoint routing** for far+low targets: climb → sweep at
  `route_transit_el` (12°) → descend, planned every tick in canonical hs
  space; `route_az_min_deg`=60, `route_reach_el/az_deg` 4/8 decide waypoint
  vs direct. Waypoint routing suppresses GLIDE engage/disengage (`route_active`
  — competing attractors caused stop/restart churn). Routing SKIPS when the
  flip branch is selected (canonical-geometry math; the direct flip line is
  in-box by construction). Geometry: at el 0 near due east az motion rides
  DEC, but the az-0 due-north low-el corridor winds RA in ~180° jumps (el-0
  singularity) — an az-hold slide there can wind RA hundreds of turns.
- **Skip detector** (deficit of commanded−measured issuance > threshold AND
  > 20 % of commanded) runs in ISSUED-counter space with grace profiles:
  band-aware cold grace (`skip_ramp_grace_low_s` 25 s for the LOW 55–104×
  slow-ramp band; HIGH cold = blind `skip_cold_blind_high_s` 2.0 s then ramp
  to 2.5 — real board issuance is dead-then-snaps, a linear model false-latched
  live dives); stop/restart windows use `skip_restart_blind_high_s` (3.0 s
  zero-expected) then BLIND PROFILE to grace end `skip_restart_grace_high_s`
  (4.0 s — strictly above the blind; consecutive storm restarts faster than
  the blind re-starve issuance and latched at ramp_age < blind); band ENTRY
  restamps the grace (in-place LOW `:I` climbs into the band missed it);
  5× hysteresis below the band edge. Sustained-supply floor inside the slow-
  ramp band (`skip_supply_slow_band` 0.75) — the band's measured plateau is
  0.78–0.80 of commanded, and a flat "expect full supply" model false-latched
  an apex dwell. Stop/restart events reset accumulators (decel gap sits in the
  window).
- **Demand slew shaping**: `demand_lpf_tau_s` 0.3 base first-order LPF on the
  GLIDE demand only (DIVE bypasses; mode-transition/floor-guard reseeds).
  Magnitude deadbands are FORBIDDEN (they freeze transients into permanent
  offsets). Velocity-feedback damping was measured and REJECTED (monotonic
  :K regression; ideal plant has no delay to ring).

## Mechanic model: travel envelope, branches, cord ledger

- **Every sky pointing is TWO axis points.** The AltAzLevel inverse has the
  over-the-top branch `(a+180°, 180°−d)` pointing IDENTICALLY. ALL travel /
  mechanics / guard reasoning must enumerate BOTH.
- **Datum-relative travel envelope**: `max_ra_travel_deg` /
  `max_dec_travel_deg` (profile keys `ra_travel_deg`/`dec_travel_deg`,
  fold180 offsets in (−180, 180]) — per-axis travel from the calibrated
  datum in EITHER direction on EITHER branch. Sirius bench: RA ±115 (stand/
  cable; tripod clearance verified), DEC ±180 (= unlimited — DEC is the free
  shaft axis, mechanically ~±182; a guard beyond 180 is unrepresentable in the
  box, over-travel stays the cumulative guard's job). The 540° RA / 270° DEC
  cumulative guards stay as cabling-budget backstops (latching); DEC is folded
  in the cumulative guard (the flip fusion makes raw |DEC| fiction — every
  axis-offset consumer must fold).
- `SafetyMonitor::check_travel` is dumb box math (non-latching Blocked); branch
  awareness lives in the tracker: `select_branch(can, cur)` — both-outside =
  refuse (non-latching → `returning_home_` datum flight, resume on any push
  that passes `check_travel` with the `travel_resume_margin_deg` 5 hysteresis),
  one-in-box = chase it, both-in = nearest to cur. **Membership hysteresis**
  `branch_handover_margin_deg` (5): the RIDDEN branch keeps its box test with
  the margin added (box membership is NOT invariant under the flip map; near-
  wall scrapes otherwise teleport the setpoint ~170° across the sky — live-
  found at LOS). Far-branch handover (> max(3·engage, 15°)) = re-routing EVENT
  (estimator+glide+integral reset, fresh dive authority) — DEFERRED while the
  pattern is descending (rides the wall instead; never dive 170° for a dying
  satellite) and flown on a clearly climbing pattern.
- **Wall-strain watchdog** (`wall_strain_deg` 10 / `wall_strain_s` 5): if the
  clamped deferred setpoint sits > band from the mount for that long, the wall
  cannot hold this target (wall-rate guard could veto demand to x=0 forever at
  err 128°) — trip releases the deferral to the forced branch (fresh dive) or,
  if the pattern is setting, takes the envelope-refusal home flight. Genuine
  along-wall slides never trip (their setpoint rides the wall BESIDE the mount).
- **DEC cord-wrap law** (treat the antenna cable as existing on DEC): the
  UNFOLDED DEC offset from the datum is the ledger. Clause 2: branch exclusion
  only when a branch's landing exceeds `wrap_window_deg` (270) + grace AND digs
  DEEPER than the current |ledger| (the ridden branch lands at the ledger by
  construction, so "deeper than cur" is load-bearing). Clause 3
  (`wrap_home_unwind`): any datum-return flight re-images onto the CANONICAL
  cord-free solution (DEC offset 0) — pay a revolution to land clean. Clause 4:
  at branch SELECTION prefer the cord-lighter branch when it saves >
  `wrap_tiebreak_margin_deg` (90) of wrap for ≤ `wrap_tiebreak_extra_deg` (30)
  extra axis travel.
- **Azimuth-first cord unwind** (`cord_unwind_route`): cord-unwind datum
  flights order the legs — leg 1 carries RA home while DEC holds the cord; leg
  2 unwinds DEC along the datum RA meridian (the meridian IS the horizon circle
  el ≡ 0 for every ledger — floor-safe by construction). Leg gate: |RA − datum
  RA| > 1° (inside the HIGH coast, cannot ping-pong); an RA error past 1° at
  ANY point re-pins the hold. The straight axis chord pierces the elevation
  floor (live: −2.4 with the guard arresting near −2) and overshoots datum
  ~50° az in the floor slide.
- **Clients must never be trusted to name a reachable pose** — refuse-and-
  hold, never "best effort". Marginal-wall poses (canonical |RA| 1–2° past the
  wall) STAY canonical by design: the membership margin keeps them, clamp pins
  the setpoint at the wall, and the mount settles ~1.3° off with no wind.

## Re-home after pass

`rehome_idle_s` (bench: 60) + `rehome_arrive_deg` 0.5 + el arm-gate
`rehome_min_el_deg` 2.0: target gone (last-client-drop/stale) for idle_s ⇒
datum synthesized as a SELF-PUSHED target through the normal tracking path
(routing/envelope/floor/skip stay armed). Only a real pass (past the el gate)
arms the clock; el-0 pushes (pre-aims, post-LOS ghosts) cancel but neither arm
nor disarm. Arrival = err < arrive AND both axes LOW band; done-flag until the
next real push; arrival log line `re-home: arrived at datum (DEC cord wrap
now N deg)`. Any real push cancels; `on_stop`/`on_park` suppress; `R` re-arms.
DEPLOY CAUTION: with the key enabled, a client-less idle daemon flies home on
its own — enable only where consented.
Authority gates: EVERY per-tick consumer of `target_` sees the self-pushed
datum as a normal client target — gate routing, envelope-resume, and glide on
`rehome_active_`, not just the obvious routing block (two shipped-law hijacks
were this class: canonical hs router re-wrapping/de-routing the flight, and
the resume path clearing the latch one tick after trigger).

## Sessions, config, and state

- **XDG platform paths**: config home `$SATEQ_CONFIG_DIR` >
  `%APPDATA%\sateq` (Windows) > `$XDG_CONFIG_HOME/sateq` > `~/.config/sateq`;
  state home `$SATEQ_STATE_DIR` > `%LOCALAPPDATA%\sateq` > `$XDG_STATE_HOME/sateq`
  > `~/.local/state/sateq`.
- Layering: station conf + `<mounts_dir>/<name>.conf` via `load_session`
  (precedence CLI/--config > profile > station). Live mount code != profile
  `mount_code` ⇒ REFUSE (exit before any write). `--mount auto` probes every
  profile's own port with the read-only `:e1` and accepts only profiles whose
  mount_code answers there; zero/several matches die with a full inventory.
- `<state_dir>/<name>.state` (ref counters + axes + orientation) saved at clean
  exit; resumed iff orientation matches AND live counters within slop of the
  saved reference (directed delta mod 2^24). Moved or orientation changed ⇒
  fresh budget at park pose. Read-only sessions NEVER write state.
- **State poisoning rule**: the counter stream is only a pose while you keep
  watching it — a session that idles is BLIND, and any persistence of an
  unwatched stream is a lie. `kCounterJumpDeg` = 30 check in the ONE raw-counter
  funnel `counters_to_axes` (600× = 2.5°/s at ≥10 Hz reads ⇒ 12× margin over
  the worst serial-stall case). On exceed: stop + safety-trip with the operator
  procedure BEFORE any setpoint computes from the garbage anchor; STICKY across
  `R`. Exit with the latch tripped ⇒ DELETE the state file + loud log. Known
  accepted edge: a reset within 30° of the old pose is undetected.
- Recovery ritual (validated repeatedly): hand-swing to datum + mount power
  cycle (counters → 0x800000) + delete state + start `--no-resume` ⇒ fresh
  calibration with fresh budget. The daemon deliberately does NOT self-recover
  across a power cycle (auto re-datum = auto TRUSTING an unobserved pose).
- `<state_dir>/<name>.cal` (BacklashCal): machine-measured backlash per axis
  (worst of fwd/rev medians), method + raw windows. `load_session` auto-loads;
  explicit profile `backlash_ra_steps`/`backlash_dec_steps` veto the file; a
  cal whose `mount_code` != profile's is IGNORED LOUDLY. `eqtool backlash` is
  the motion-gated wizard (run-in → reverse → geometric burst search with an
  EXTERNAL motion oracle `--eyes ask|cmd`; net arcmin-level travel ⇒ remind to
  re-home after). Values are currently advisory (the measurement-side play
  clamp in feedback was LIVE-REVERTED: it limit-cycled on the real worm;
  demand-side take-up is the redesign candidate).

## rotctld contract + tracker-client behavior

- Verified against hamlib: `\dump_state` handshake, `P`→`RPRT 0`, `p`→`az\nel\n`,
  unknown→`RPRT -1`, `q` closes, `+` prefix stripped, negative az normalized
  (`deg_norm360`).
- Caps command `1` (GridTracker2 contract): reply name line +
  `Max/Min Azimuth:` + `Max Elevation: 90.00` / `Min Elevation: -90.00` +
  `RPRT 0`. Rules: exactly one line may start `Max Elevation:`; NO
  bare-numeric-only lines (GT2 pairs consecutive numeric lines as position);
  never `RPRT -` (GT2 latches rotor Error). Answering `1` is required for a
  clean GT2 session.
- **Multi-client** server (thread per connection; stop() shuts every live fd).
  A tracker can hold the session while monitors attach concurrently.
- **Client disconnect semantics**: last-client-disconnect drops the target
  (socket close = operator gone); `target_stale_s` default 90 s; the watchdog
  is a HOLD not a latch — any rotctld command resumes; a `p` poll alone keeps
  the client alive (GT2 polls `p` even when throttled). Motion-gate smoke
  scripts must model client DISCONNECT, not just command grammar (a one-packet
  `P...q` batch closes before the first tick → stale-halt → looked like a bug).
- GT2 behavior notes (client side, current build): connect → caps `1` → poll
  `p` every 2 s; follow = re-push every 2 s while last-READ rotator position
  is >1° from the current satellite target ("a rotator that stops when it
  isn't told for a while gets there all the same"), else 1°/2 s throttle;
  el clamped 0–90 (never negative → −5° arm band never engages; GT2 pre-aims
  `P aosAz 0` when AOS ≤ 5 min — pre-aim at el 0 is a real target on our side,
  benign); post-LOS el-0 ghost tails measured 0–12 s on the current build
  (older builds ghosted ~7 min); double-push pairs rare now (0.5–0.9 s gaps)
  but the estimator min-cadence law remains the insurance; follow AUTO-SWITCHES
  to the next queued sat at LOS.

## TICK tracing (`sateqd --debug`)

One `[TICK] k=v` row per 10 Hz control tick to the main log, OFF by default
(~170 MB/h worst case). Layers stay log-free: `Board` records a drainable
`WireExchange` trace (tx frame, raw rx verbatim, err tag `timeout|nak|
malformed|held`, latency, retry attempts) at the single `Board::dispatch`
choke point — SerialTransport AND simulator, so replays get the same rows as
live metal. `step()` = lock + `step_impl()` + emit, so EVERY exit path prints
exactly one row (a halt row is the row the autopsy needs); tick-derived fields
are NaN-poisoned at entry so early-return rows never show stale setpoints. Row
carries counters, axes, hs pointing, target/pattern, setpoint/err/flip, demanded
vs sent rates + band code, skip accumulators, `:f` words, `reg=` per-axis wobble
ledger + flags incl. `lrpf`, `srk=` stop/restart census + per-row `srkd` deltas
(SUM PER ROW in awk — it is a delta), then every wire exchange of the tick.
Observer effect by design: tracing adds one `:f` poll per axis per tick.

## Tools

- `tools/eqtool` — swiss-army probe: `info | query | poll | stop | home | rate |
  backlash` (motion commands need `--force-motion`; `--hold-writes` dry-runs
  and dumps exact wire frames).
- `tools/replay_gt2` — replays recorded push CSVs through the live
  Tracker+Board+Simulator stack at virtual clock. Modes: dense (all pushes) /
  `--sparse` (old-GT2 follow-gate emulation). Sweep knobs: `--lpf --lrpf
  --lrcbar --lrrel --rcs --hold --coast --hcs --rehome --nowrap --unwind
  --ra-limit --dec-limit --gerr --accel --slowramp --coldblind --coldgrace
  --cblind --mgate --promote --debug`. Judge by per-30-min steady error
  buckets, NOT global rms (acquisition dives legitimately hit >80°).
  "Metal-plant" replay knobs reproducing live board behavior:
  `--coast 0.3 --hcs 0.3 --accel 1.0 --slowramp 25`.
- `tools/analysis/drive_dryrun` — fixture-style rehearsal for consented-motion
  experiments (push → settle → drop → auto-home with ledger/RA/az/el traces +
  narratives). ADOPTED: rehearse every consented-motion experiment here before
  metal. `tools/analysis/wind_probe` — enumerates in-box flip poses with RA
  travel + ledger band grading.
- `tools/probes/` — raw wire probes. NEVER run against a live sateqd port.
  Raw-probe discipline: exactly one process on the port, send `:F` before
  `:J`, and use byte-SWAPPED revu24 `:I` periods (unswapped periods decoded
  to ~0.0002× sidereal and produced a whole session of "mount is dead"
  misdiagnoses).
- `tools/live/` — rotctld clients for live sessions (position monitors,
  settle checks, dive latency monitors, camera-oracle helpers, `watch_events.sh`
  — background a blocking watcher that exits at the first new non-TICK log line
  so a human gets pinged during passes instead of sleep-polling).

## Tests and replay discipline

- `cmake -B build && cmake --build build && ctest --test-dir build` → 10 suites
  (config, session, board_sim, byteorder, framing, protocol, astro, safety,
  tracker, rotctld) + smoke scripts (`smoke_rotctld.py`, `smoke_readonly.sh`,
  `smoke_session.sh`, `smoke_autodetect.sh` — needs socat; skips without).
- The simulator models: handshake goldens (heq5/eq8), `:F` lazy arm + NAK-4
  re-arm, running-axis rules, `:K`/`:L` flavors, coast (`set_coast`), issuance
  ramps (`set_accel`, `set_slow_ramp`), cold/restart blind, truncated-reply
  retry, write-gate held frames, backlash play model, wall-clock advance toggle.
- **Every law change ships with a simulator test + the replay guard matrix**
  (8 committed datasets × dense/sparse, new-vs-legacy SAME binary; error rms
  must stay flat-to-better). Commit replay deltas.
- Measurement traps (all paid for):
  1. A regime/ledger detector riding on leaky COMMIT counters is
     SELF-SUPPRESSING when the law it gates reduces commits — **measure
     candidate ledgers with the candidate law OFF** (a "ledger max 5.7, class
     excluded" claim was an artifact measured under the escalation it gated).
  2. Intermediate thresholds on live ledgers are CHAOTIC (arm threshold flips
     mid-burst as the ledger oscillates) — validate the tuned triple as one
     law, don't re-fit per fixture.
  3. Replay-vs-live divergence: the simulator lacks `:K` coast-blocking and
     full board accel ramp → replay DIVE-phase `:K` counts are an upper
     bound; pass-phase counts track live well.
  4. Synthetic fixtures must match the live phenomenon's cadence + geometry —
     an early corridor fixture read legacy ≈ fixed and taught the wrong lesson
     until the replay (the real discriminator) exposed it.

## Data and evidence policy

- GT2 push records + monitor CSVs + raw probe batteries: COMMITTED under
  `data/` (they are the replay corpora).
- Daemon TICK logs: huge; gzipped to `data/evidence/` on disk but kept OUT of
  git (the .gz files are machine-local archives; the CSVs carry the analysis).
- Both boxes run /tmp on tmpfs — never park bulk data or long-lived archives
  there. Pull model for evidence: `scp bench:FILE data/evidence/`, verify size
  + `gzip -t`, only then delete on the bench. NEVER chain tar+scp+rm in one
  command.

## Lessons (standing rules)

- After any new home pose, mount, or head orientation: at least one
  sky-direction probe (eyes or camera oracle) must confirm axis SIGN against
  the physical world before trusting a pass. Counter-space green ≠ sky-space
  green (a mirrored tilt axis was invisible at el 0 and survived every test
  and three sweeps; the model was internally consistent, just reflected).
- Every guard consuming axis offsets must use FOLDED semantics and enumerate
  the (a+180, 180−d) image — audit any new axis-space consumer near the flip
  fusion.
- `:j` counts ISSUED pulses: skip latches are starvation, backlash is
  invisible, hand-rotates are ledger-invisible. After any shaft work, re-datum.
- Sustained big-rate reversals every ~2 s at small error = tube not following
  (operator watch signature); err columns are NOT a follow indicator once
  slipping.
- A rotctld `p` stall of ~1.2–1.5 s (not a clean 1 s timeout) means the serial
  thread held the tracker lock across board I/O; degraded-idle (dead wire)
  stretches tick period to ~9 s and holds the lock — daemon looks hung from
  outside, isn't. Full dead-wire detection: HALTED within one tick with the
  board I/O failure line (verified).
- Interface breakouts fed from the mount 12 V rail can brown out under
  dual-HIGH + `:K` regenerative braking (open hardware suspicion; meter before
  dismissing).
- Software steering (floor guard) is powerless against a dead wire; the board
  keeps running its last `:J` program with a dead host (no autonomous watchdog).

## Open items

- HDX-110: `ra_sign` UNKNOWN (profile not trusted for sky pointing until its
  own eyes-on sign probes); its travel envelope is unmeasured (profile stays
  105/105 until operator guidance).
- First live passes: verify the demand-LPF escalation on a 22:00-descent-class
  window (expect well under 60 `:K`/min DEC, `lrpf` flag + `reg=` ledger in
  TICK rows); handover-arming-storm law defect reproduces in sim from deep
  canonical poses (waypoint + far flip target).
- Backlog law candidates: flip-branch-safe routes for far+low flip dives;
  cord-cost pricing at pre-aim branch picks near wall-only geometry;
  elevation-eligibility pricing for handover fly decisions; elevation-floor
  routing for envelope-refusal returns.
- Hardware: meter the mount 12 V rail at the RJ45 during a deliberate
  dual-HIGH + `:K` storm stress; independent interface supply / bulk cap /
  1 kΩ series R if sag confirmed. Daemon-side: fast `:L` retry loop while
  serial-down.
- GPS NMEA ingestion (4800 baud puck) for lat/long/UTC auto-sync (not started).
- `sateqd --version` git hash is captured at cmake configure time (cosmetic).
- `on_reset` (rotctld `R`) does not zero the cumulative rotation accumulators
  (re-arms rehome but leaves the budget wound) — needs a fix + test.
- `p`-latency 1 s `:j`-timeout spikes under high churn remain unexplained at
  the link level (board DEC `:j` freeze suspicion; no kernel USB events seen).

## Commit hygiene (standing instruction)

- No drive-by changes folded silently into feature commits: dead-code removals,
  warning fixes, reformatting etc. get their OWN commit (with their own reason),
  or don't do them. "Harmless to include" is NOT a standard.
- Don't leave dead/unused code around for later sessions to wonder about.

## Design decisions (agreed)

- Standalone daemon owning the serial port, exposing rotctld (over patching a
  Hamlib backend or INDI-first). Language: **C++** (matches hamlib/indi-eqmod
  ecosystem).
- Board is step-dumb; all astronomy in the daemon (hand-rolled, no libnova dep).
- rotctld subset must cover what GPredict/GridTracker2 send for rotor control
  and status, and stay friendly to `rotctl` CLI for manual testing.
