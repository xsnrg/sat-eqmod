# GT2 rotor issue: follow throttle starves slow targets (rotctld)

**Title:** rotor (Hamlib rotctld): the ≥1° follow-throttle starves slow near-horizon targets — pre-aim can't complete, and mounts with a stale-target guard can never catch up

**Body:**

Running the GT2 test release with Features → Rotor Control, protocol "Hamlib rotctld", against a dedicated satellite-tracking rotctld server (SkyWatcher motor board, station on the bench). Live RS-44 pass, 2026-11-16 ~20:25 UTC.

**Behavior**

`rotorPoint(az, el, follow = true)` (rotor.js ~638) sends only if the target moved **≥ 1.0°** since the last push, on top of the `ROTOR_FOLLOW_MS` rate limit:

```js
if (now - R.sent.at < ROTOR_FOLLOW_MS) return false;
if (Math.abs(az - R.sent.az) < 1 && Math.abs((el === null ? 0 : el) - (R.sent.el === null ? 0 : el)) < 1) return false;
```

Near the horizon, a LEO target moves much slower than 1°/2 s — RS-44 at AOS was az ≈ 0.05°/s, el ≈ 1.1°/min — so GT2 re-pushed only every ~19 s. Any rotator that still has distance to cover, or that implements a "stop when the tracker goes quiet" safety (which rotctld clients are expected to feed continuously — GPredict, for example, pushes every update unconditionally), makes almost no progress under this rule.

**Observed on the server side (rotctld `P` commands, daemon log)**

Pre-aim window (AOS −5 min): exactly **one** push, because `pass.aosAz` drifts < 1° in five minutes and every re-issue inside the window is eaten by the throttle:

```
20:19:44  P 232.2 0
```

The first minutes after AOS — the target advances *exactly 1.0° az* per push every ~19 s, which is the throttle talking, not the satellite:

```
20:25:02  P 233.2 0.2
20:25:21  P 234.2 1.0
20:25:40  P 235.2 1.8
20:25:57  P 236.2 2.5
20:26:14  P 237.2 3.2
20:26:30  P 238.2 3.9
```

Our mount started the pass 130° away (pre-aim never completed — 12° of slew, then the server's stale-target stop) and never caught up at one 5-second run per 19 s. The pass was lost.

**Why the 1° condition specifically hurts**

- The pre-aim (`rotorPoint(pass.aosAz, 0, true)`, rotor.js:673) *is* re-called every 2 s poll inside the 5-minute window — the throttle discards all but the first call. A rotator that's far from the AOS azimuth, or whose connection blipped mid-approach, can never recover.
- While following near the horizon, rotators with a stale/squat safety stall–resume forever on slow targets and fall behind the satellite permanently.
- The `ROTOR_FOLLOW_MS` rate limit alone already bounds traffic; for rotctld a position push is a 12-byte line.

**Suggested change**

While following a satellite, drop the ≥1° condition and push every poll cycle (keep `ROTOR_FOLLOW_MS`). If very slow serial backends motivated the threshold, a far smaller epsilon (e.g. ≥ 0.1°) would keep its purpose without breaking real tracking — rotators round to 0.1° anyway (`toFixed(1)`).

**Related, same service (minor):** `rotctld.parse` latches the rotor status to *Error* on any `RPRT -` line (rotor.js:114–115), including the reply to the caps `1` command. Hamlib servers answering `RPRT -1` for an unimplemented command is spec-conformant behavior; GT2 should treat an errored caps reply as "no caps" (leave `hasEl` unknown) instead of showing a persistent error. (Our server initially didn't implement `1`, so GT2 showed Error on an otherwise perfectly working rotator — we fixed our side, but GT2 currently punishes every spec-following server.)

**Environment:** GT2 test release as of 2026-11-16. 1 Hz rotctld position log and full server-side command log from the pass available on request.
