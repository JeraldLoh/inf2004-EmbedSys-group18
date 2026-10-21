# Buddy 2 - Motion

Primary responsibility: motor output, encoder snapshots, wheel control,
odometry, continuous requests, finite goals, and motion faults.

This subsystem must remain the sole normal runtime writer of motor-driver
commands. Other buddies may contribute, but Buddy 2 remains accountable for
its contracts, calibration, and evidence.

The course folder is `subsystem2`. `motion_math.c/.h` is the first ported part
of the earlier standalone work: hardware-independent count/distance and pivot
geometry using caller-supplied calibration. Its host tests do not move hardware.
Put future `drv_motor.c/.h`, `drv_encoder.c/.h`, wheel-control and odometry files
directly here, not under root `device/` or deeper driver folders. Lifecycle hooks
remain inactive `E_NOSPT` placeholders; they do not configure GP8-GP11 or
GP26/GP5. No encoder PPR, geometry or PID constants are treated as measured.

Use shared motion contracts and explicit completion/cancellation/timeout/fault
results. Do not copy old blocking movement routines or assume local interrupt
disabling protects data across both cores. Evidence belongs in
`../../testing/subsystem2/`. The old standalone bench UF2 is not the integrated
kernel application.

See `../../docs/migration/standalone-port-inventory.md` before moving any more
files from the sibling Buddy 2/3/4 bench projects.
