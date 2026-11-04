# B2 V17 low-speed candidate, diagnostics and firmware identity

Only files under app_program/subsystem2 and this Buddy2 document changed.
Buddy3, Buddy5, shared controller and the existing70mm/s trial speed setting
are unchanged. The known post-junction permission/resume bug is not fixed.

## Low-speed line-turn candidate

V16 sometimes requested about10mm/s on the inside wheel. Across the five
operator trials, four curve passes and one failure were reported. A zero
inside-wheel speed sample appeared in both a failed and a passed run, so it
is a control concern rather than proof of the sole failure cause.

V17 keeps the small-command straight filter and smooth paired target ramp.
For explicit trial LINE turns only, after V16's targeting it clamps a nonzero
inside target to a candidate20mm/s minimum, never above the caller's requested
speed or hard wheel cap. Ramp-up can still pass through lower speeds. Zero
commands remain zero. Outside targets are not increased by this change.
At the tightest turn nominal targets become120/20 rather than120/10mm/s.
That makes the tightest path wider; the floor is NOT a measured calibration
value or guaranteed fix. Compare low-speed tracking and actual curve tracking
before accepting it. PID gains, feedforward and calibrated encoder scales
are unchanged. Existing no-progress timeout, STOP, watchdog and caps remain.
Finite goals and Buddy5 requests retain their original target calculations.

## New runtime messages

- `[B2 CONFIG] fw=v17-low-speed-diag base=70mm/s ... min_line_wheel=20mm/s`
  identifies the actual candidate, speed and turn settings. This is the
  authoritative configuration; the shared CAR banner retains older50 text.
- `[B2 TRACK]` prints targets, measured speeds, validity, effort, state,
  authority generation, rejection count and floor_mask at most once per second.
  Floor-mask bit0=left, bit1=right; it marks the requested target clamp, not
  proof that the physical wheel achieved that speed.
- `[B2 REJECT]` names the rejection category and executor return status,
  source/ID, requested/current/paused generation and executor authorization.
  The existing post-junction failure should show
  `LINE_PAUSED_NEEDS_NEW_GENERATION`. It remains rejected, not bypassed.
- `[B2 STOP DETAIL]` retains the latest active-executor stop category, result,
  generation, command, affected-wheel mask, and target/measured speeds before
  the normal zero-output finish. Categories distinguish capture/measurement,
  authority deadline, continuous TTL, goal timeout/admission, driver inhibit,
  overshoot, invalid PID input, no progress, watchdog arm and motor output IO.
  A category does not diagnose physical wiring or instrument STOP latency.
- `[B2 INGRESS]` reports the last early-ingress failure category (not started,
  null request or mutex busy). It is not a lossless counter.

Logging uses one additional trial-only low-priority task on core0, priority18,
with4KiB static stack. It copies snapshots under the existing motion mutex,
then releases it before printing. The20ms motor task and ISR paths never print.
Repeated events between1s snapshots may coalesce; the latest fault sequence
and cumulative rejection count remain visible. Task creation failure aborts
initialization; diagnostic start failure keeps the public ingress stopped and
inhibits motor outputs. No new shared kernel configuration was introduced.

## Manual check and validation status

Compilation succeeded. Automated test suites were skipped at the user's
explicit request; no claim is made that the new fault paths, resource failure
paths or20mm/s floor have been runtime-tested. No flash, vehicle movement or
remote push was performed by the agent. Production-qualified remains false.

Flash the new UF2 manually. With wheels raised check START/STOP first, and
confirm `[B2 CONFIG]` shows v17 and70mm/s. Then repeat the same short straight
and curve with GP21 accessible. Retain all B2 CONFIG/TRACK/REJECT/STOP DETAIL
lines through the final stop. Check whether floor_mask marks the inside wheel,
its measured speed stays near20, and whether the wider tight-turn target still
follows the curve. The15s trial limit, no-black timeout, failed barcode
calibration and known junction resume rejection remain separate limitations.
