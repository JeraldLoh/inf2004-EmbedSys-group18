# Buddy 2 integrated trial status - 2 November 2026

## Testing publication update

The operator subsequently requested publication to remote testing. Active
firmware has been restored to the V5 baseline (d6bc6ad/a0190ea firmware sources),
with50mm/s cruise, no outside-wheel boost and the original HIGH=black setting.
V6 polarity and V7-V9 speed/boost experiments are not active. The source history
retains those experiments and their reverts. Shared adapters and trial-only
Buddy3 steering changes are included as dependencies of this integrated image.
Remote main is outside this publication. No new automated suites were run.

The later physical left/right/both-on-black check raised a possible polarity
mismatch; the attempted LOW=black change was reverted at the operator's request
after worse reported behavior. Physical mapping/polarity and line geometry are
therefore still open, not resolved by restoring the baseline.

## Earlier Buddy-2 publication scope (history)

Publish the local integrated trial work through d6bc6ad to Buddy-2, alongside
the previously published standalone v6 motor work. This includes explicitly
enabled trial startup, shared controller handoff changes, and trial-only
Buddy 3 steering candidates. Cross-subsystem changes remain review items for
their owners. Remote testing and main are not part of this publication.
The generated UF2 files remain local artifacts; source and recipes are tracked.

## Current hardware evidence

The operator reports following the line in three out of four attempts, with
one departure on a turn. This is an operator observation, not a measured
reliability rate or final integrated acceptance.

Recent v5 logs show both encoder speeds, independent wheel targets responding
to steering, drv_err=0, and normal physical STOP handling. Examples during
right turns include target80/19mm/s with measured81/19 and75/20mm/s. There is
no persistent wheel stall established by those samples. An isolated command
rejection around physical STOP is distinct from the sustained issue below.

The junction-pause fix allows entering probe settle without the earlier
150ms line-lease expiry during a250ms deliberate wait. Subsequent logs show
probe/approach progression without the earlier motion fault. The physical
latency of STOP and watchdog cutoff remains unmeasured.

## Open integration issues

- Both-white line readings can mean centred or off the tape. After the200ms
  fading correction, steering returns to straight. The existing3s no-black
  timeout can stop both genuinely lost and well-centred straight runs.
  Buddy 3/shared-controller line recovery needs further work.
- Curve readings can trigger both-black junction detection; whether those
  observations are real intersections is not established from logs alone.
- After a completed junction, one log shows VERIFY_LINE, zero outputs and
  repeated rejections up to256. car_motion_halt blocks the old line authority;
  the junction completion path resumes line tracking without a fresh authority
  generation. The controller/Buddy 2 resumption handoff still needs correction.
- Buddy 5 detects obstacles and interrupts tracking (reported23cm,15cm,7cm).
  Scanning motion occurred; successful full avoidance/rejoin is not established.
- Barcode calibration repeatedly fails low contrast. No barcode-route pass.

## Qualification and validation

Buddy 2's motor/encoder execution is functional under the reported tests, but
this integrated firmware is not final-qualified. Production-qualified remains
false. Explicit bounded CAR_B2_TRIAL is required; normal qualification gating,
manual START, physical STOP, watchdog, command lifetimes and15s run limit remain.
No global qualification or network/terrain integration claim is made.

V4 software history:157 native tests and19 shared suites passed. At the user's
explicit request, tests were not rerun for v5 or this publication. V5 compiled
successfully. This status document changes no firmware behavior. No firmware
was flashed and no vehicle was operated by the agent during publication.
