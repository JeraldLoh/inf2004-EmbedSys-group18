# Buddy 5 - Obstacle Scanning and Avoidance

Primary responsibility: sonar trigger timing, servo scanning, obstacle
profiles, avoidance proposals, and bounded line recovery.

Servo scanning is the current primary assumption; calibrated whole-car
scanning is a conditional fallback. Buddy 5 also leads integration review but
this folder does not contain or own the separate shared Vehicle Controller.

The course folder is `subsystem5`; future `drv_sonar.c/.h` and optional
`drv_servo.c/.h` go directly here. Current lifecycle hooks return `E_NOSPT` and
target tests are SKIP placeholders. No trigger, servo or manoeuvre is started.

Hardware-independent logic ported from Buddy 5's standalone prototype (no GPIO,
PWM, interrupts or kernel calls; uses no libm, since the image is not linked
with it):

| File | Purpose |
|---|---|
| `range.c/.h` | Reading status (ok / no-echo / too-close / fault) and median-of-N filter |
| `obstacle.c/.h` | Scan geometry: direction+distance to x/y, obstacle width, side choice, bypass plan |
| `avoidance.c/.h` | Non-blocking detect, scan, decide, bypass, reacquire state machine (section 6.5) |

Scanning: the servo checks a narrow arc first (coarse sweep, then fine steps).
Edges outside the servo's reach are found by turning the whole car on the spot
in steps (`TURN_SCAN_RIGHT/LEFT`); once a turn shows open space, the servo looks
back to pin the edge down. The bypass is a mirrored three-leg detour (turn out,
drive, straighten, drive alongside, turn back the same angle, drive, straighten)
so the car ends near the line facing the line direction, then searches for it.
Motion is requested through `motion_turn` / `motion_forward` / `motion_reverse`.

`avoidance.h` defines `av_platform_t`, the callback table that the future
sonar/servo drivers and the controller/motion adapters fill in. Avoidance only
proposes motion through those callbacks; it never drives PWM itself. Host tests:
`../../testing/host/subsystem5/` (target `obstacle_avoidance_host_tests`).

Agreed pins (`../project_config/pinmap.h`): sonar TRIG GP16, ECHO GP17 (through
a voltage divider), scan servo GP12. The physical scan range and car-turn
accuracy remain unverified. Avoidance
proposes actions to the controller; it must not compete with Buddy 2 for PWM.
Keep evidence in `../../testing/subsystem5/`.

Component bench tests (sensor and servo on their own, Pico SDK, wheels never
move): `../../testing/subsystem5/bench_firmware/README.md`.
