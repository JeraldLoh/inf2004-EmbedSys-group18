# Demo integration validation — 24 November 2026

Base `origin/main`: `b0dee3ee3fce810bc39c496031016342b8ba91da`.
Branch: `demo-integration-20261124`. All 12 original demo-fix commits were
cherry-picked onto that base, followed by review corrections. Main and the
original fixes worktree remain available. No push, flash or motor command was
performed as part of this integration.

## Software evidence

- 5 portable test executables passed: existing controller service, recovery
  controller, wheel-path/pickup regressions, original terrain configuration,
  and terrain with bounded jolt pause. Strict compiler warnings are errors.
- 25 selected Buddy 2 cases passed: driver foundation, STOP interleavings,
  car steering/obstacle/junction/startup, distance/reverse/pivot approach,
  blocked wheels, speed caps and the new real-executor S-bend retrace case.
- Pickup tests cover assisted/unassisted confirmation, junction suppression,
  stale words, a new word after a stale gap, timer wrap, continued latching
  through sensor loss, and one second of fresh floor evidence before re-arm.
- Retry regression covers a reverse lasting longer than the old six-second
  holdoff, reverse-decoded route rejection, a failed slow pass, no immediate
  second retry, and explicit new-run re-arm.
- Encoder backtrack regression verifies a measured 55 mm reverse request and
  early cancellation when a sensor sees the line, with a fresh generation.
- Both full RP2040 Wi-Fi/lwIP/MQTT firmware profiles compile and link.
- Optional optical pickup code compiles with a synthetic ADC threshold used
  only for compilation. No UF2 using that threshold is supplied.
- Analogue Buddy 3 acquisition and analogue IR-driver paths compile with
  APP_S3_LINE_PID=1. Neither packaged image enables analogue line steering.
- OFF_GROUND=1 without an ADC threshold is rejected with a readable error.

The 25-case selection initially exposed an old fixture that never declared
the line present and startup mocks missing current B4/controller interfaces.
These fixtures were repaired to express their intended startup/junction
scenarios. The five startup cases also check controller-mediated B5 bindings
and supported startup without an IMU. These are host simulations, not RTOS
timing, electrical wiring, PID qualification or physical sensor calibration.

The proposed 150 mm/2000 ms IMU setting did not meet the original test's
below-threshold height expectation: a physical-model 120 mm lift reached a
reported 150 mm. That candidate was not selected. This does not establish a
floor/hump false stop; it does show near-threshold height error in simulation.
The original 200 mm/1200 ms configuration passes, as does the independently
switchable bounded jolt-pause change with the original thresholds.

## Supplied images

Both use 80 mm/s cruise, damping 40 for 40 ms, digital line steering,
GP26 analogue barcode acquisition, MQTT, local private Wi-Fi settings and
broker 172.20.10.2:1883. Ground logging is enabled at 10 Hz for characterization.

| Image | Line-loss timer/backtrack | Barcode retry | IMU jolt handling | Optical pickup |
| --- | --- | --- | --- | --- |
| group18_demo_baseline_80_mqtt_2026-11-24.uf2 | Off/off | Off, no path ring recording | Existing reset-on-jolt | Off, measurements needed |
| group18_demo_recovery_80_mqtt_2026-11-24.uf2 | 1500 ms/150 mm | On, one per junction approach | Pause bad samples within original window | Off, measurements needed |

Use baseline first. Recovery is a dedicated test candidate: long centred
both-white stretches can falsely trigger the timer, and reverse rolling is
not physically qualified by these tests. Test each optional behavior in
isolation before a combined track run. Protective motor/STOP/fault mechanisms
remain enabled; WATCHDOG_TEST is zero in both images.

Package manifests record commit, full build flags, SDK/toolchain, hashes,
RP2040 UF2 block validation and the absence of hardware validation. Build logs
and compile checks are beside the UF2 files under
`output/firmware/demo-integration-2026-11-24` in the parent workspace. Credentials
are excluded from source control but are embedded in the local UF2 files.

## Remaining physical evidence

1. M1 floor/line/crossbar/hump/lift ADC and digital polarity readings, then an
   optical threshold that separates all relevant conditions. If readings
   overlap, do not enable the optical rule as written.
2. Fast, slow, low and tilted pickups, held in the air; no false stops through
   humps or the one-second junction pause; both START paths locked until floor
   confirmation; GP21 STOP always effective.
3. Stable straights and bends in both directions with MQTT running; measured
   encoder reverse and recovery attempts without stalls or false searches.
4. Known barcode characters and mapped turns, actual hump/obstacle dimensions,
   repeated complete bypass/rejoin and fresh real dashboard telemetry.

The code can support the ten rubric abilities, but no ten-point score or
arbitrary slow-pickup success is claimed. See DEMO_BUILD.md for configuration,
measurement steps and the three-run/latest-run scoring rule.
