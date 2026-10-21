# Buddy 5 - Obstacle Scanning and Avoidance

Primary responsibility: sonar trigger timing, servo scanning, obstacle
profiles, avoidance proposals, and bounded line recovery.

Servo scanning is the current primary assumption; calibrated whole-car
scanning is a conditional fallback. Buddy 5 also leads integration review but
this folder does not contain or own the separate shared Vehicle Controller.

The course folder is `subsystem5`; future `drv_sonar.c/.h` and optional
`drv_servo.c/.h` go directly here. Current lifecycle hooks return `E_NOSPT` and
target tests are SKIP placeholders. No trigger, servo or manoeuvre is started.

GP6/GP7 and optional GP12 remain the provisional Week 6 section 2 baseline;
verify the harness because another report section used GP2/GP3. The physical
scan mechanism/range and fallback calibration remain unverified. Avoidance
proposes actions to the controller; it must not compete with Buddy 2 for PWM.
Keep evidence in `../../testing/subsystem5/`.
