# Buddy 5 - Obstacle Scanning and Avoidance

Primary responsibility: sonar trigger timing, servo scanning, obstacle
profiles, avoidance proposals, and bounded line recovery.

Servo scanning is the current primary assumption; calibrated whole-car
scanning is a conditional fallback. Buddy 5 also leads integration review but
this folder does not contain or own the separate shared Vehicle Controller.

The course folder is `subsystem5`; future `drv_sonar.c/.h` and optional
`drv_servo.c/.h` go directly here. Current lifecycle hooks return `E_NOSPT` and
target tests are SKIP placeholders. No trigger, servo or manoeuvre is started.

Sonar TRIG/ECHO are GP16/GP17 (Grove 4), with the 5 V echo through a voltage
divider; the optional scan servo is GP12. GP2/GP3 belong to the Buddy 4 IMU.
Verify the harness and divider before powering the sonar. The physical
scan mechanism/range and fallback calibration remain unverified. Avoidance
proposes actions to the controller; it must not compete with Buddy 2 for PWM.
Keep evidence in `../../testing/subsystem5/`.
