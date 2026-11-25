# Buddy 2 — Motion control

Buddy 2 drives the Group 18 rover's motors, reads both wheel encoders, controls wheel speed, and reports odometry and motion results. The vehicle controller authorizes movement. Buddy 3 supplies line and junction observations, Buddy 5 requests obstacle maneuvers, and Buddy 4 uses wheel odometry for hump estimation. Buddy 2 owns normal motor output.

## Build and run

Use the [combined demo build command](../../docs/DEMO_BUILD.md#combined-demo-profile-through-u-turn-5), which selects `CAR_RUN=1 CAR_B2_TRIAL=1`. This profile loads the v6 wheel-control settings and waits for an operator START. Physical STOP, the watchdog, and motion interlocks remain active. The generic `CAR_RUN` profile with `CAR_B2_TRIAL=0` uses the unqualified candidate configuration with zero gains. The standalone movement test and bench profiles have their own build and acceptance scope.

## Motion-control library

Use the typed requests and results defined in [the motion header](subsystem2_motion.h) and [the shared contracts](../contracts/system_types.h).

| Interface | Use |
|---|---|
| `subsystem2_motion_continuous(request)` | Refresh a wheel-motion request for line following. Requests expire and carry controller authority. |
| `subsystem2_motion_goal(goal)` | Request a finite forward, reverse, pivot, or enabled retrace goal. Distance goals use millimetres and pivot goals use millidegrees. |
| `subsystem2_motion_goal_result_for(source, id, result)` | Read progress or the terminal result. Junction and obstacle requests each have their own goal ID namespace. |
| `subsystem2_motion_telemetry(telemetry)` | Read wheel-speed targets, measured speed magnitudes, validity, distance travelled, and motion diagnostics. |
| `subsystem2_motion_halt()` | Cancel the current maneuver and set motor output to zero. |
| `subsystem2_urgent_stop()` | Apply the driver inhibit for an urgent stop. Continuing requires fresh controller authority after the stop is cleared. |

The runtime adapter in [`car_motion.c`](car_motion.c) handles steering conversion and goal routing. The [measured executor](measured_motion_executor.c) runs wheel control and checks timeouts and encoder quality. The [motor task](subsystem2_motion.c) runs on core 0. Driver access uses the bound HAL and the shared GPIO dispatcher. Encoder snapshots capture both wheel counts and timing together. Direction in odometry comes from the command history because the car uses one encoder channel per wheel.

## Demo configuration and PID settings

| Setting | Value |
|---|---|
| Normal line-follow cruise | 80 mm/s |
| Nominal motor-control period and hardware cutoff | 20 ms loop, 100 ms watchdog |
| Encoder inputs and conversion | GP27 left and GP5 right, channel B on both edges. Left 1,271 and right 1,264 counts per revolution. |
| Wheel geometry | 65 mm wheel diameter, 112 mm ruler track width |
| Wheel PID gains as stored, multiplied by 1,000 | Both wheels Kp 1500, Ki 1500, Kd 0 |
| Feedforward as stored, multiplied by 1,000 | Left 2163, right 2689 |
| Effort limits | 30% for normal wheel control, 40% for integrated pivots |
| Latest pivot change | U-turn requests 5% more wheel travel than the preceding combined profile. Left and Right retain their prior travel. |

The v6 profile's initial feedforward estimates came from observed wheel speeds of about 92 mm/s left and 74 mm/s right at the same effort of 199 per mille. The [profile source](../diagnostics/b2_move_profile.c) records the gains, effort limits, and rolling-speed floors. A full PID tuning report should add target-versus-measured speed traces, the run conditions, and the rationale for the final gains.

## Motion accuracy and test evidence

| Evidence | Result and scope |
|---|---|
| Earlier standalone distance trials | Encoder feedback reported 296 mm for a 300 mm forward command, 996 mm for a 1,000 mm forward command, and 296 mm of reverse travel for a 300 mm reverse command. These are encoder estimates. Separate ruler readings are needed for physical error. |
| Earlier v6 turn trials | The operator reported approximately 90° Left and Right pivots. Exact measured angles and repeat counts were unavailable. |
| Integrated floor runs | The operator observed straight and bend tracking. Controller logs recorded accepted barcode routes and completed junction turns. One completed U-turn resumed following the line and then stopped on encoder `MEASUREMENT_QUALITY` fault `0x8`. The exact rejected encoder check is still unknown. |
| U-turn +5% candidate | The Pico build and controller host tests passed. The reviewed hardware logs predate this profile. Its physical turn angle and post-turn repeatability still need a floor check. |
| STOP and watchdog | Earlier standalone hardware tests recorded physical STOP and a watchdog cutoff. The logged observation time does not establish physical motor-pin cutoff latency. |

The [Buddy 2 test log](../../testing/subsystem2/TEST_LOG.md) is the index for test cases and results. The [standalone hardware evidence](../../testing/subsystem2/B2_HARDWARE_EVIDENCE_2026-11-02.md) identifies the earlier profiles and measurements. The [publication notes](../../testing/subsystem2/PUBLISH_VERIFICATION.md) and [host test instructions](../../testing/subsystem2/b2_01/README.md) cover reproducible software checks.

## Deliverables

- **Motion-control library:** the code and interfaces in this directory.
- **PID tuning report:** the settings and tuning evidence above are a starting point. The proposed standalone report is `testing/subsystem2/PID_TUNING_REPORT.md`.
- **Motion accuracy evaluation:** record commanded and ruler-measured distance, measured turn angle, absolute error, repeat count, test conditions, and tested firmware profile. The proposed standalone evaluation is `testing/subsystem2/MOTION_ACCURACY_EVALUATION.md`.

Link the finished reports and their evidence from the test log. Keep historical bench measurements attached to the profiles that produced them.
