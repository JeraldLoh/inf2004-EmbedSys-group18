# Buddy 2 - Motion

Primary responsibility: motor output, encoder snapshots, wheel control,
odometry, continuous requests, finite goals, and motion faults.

This subsystem must remain the sole normal runtime writer of motor-driver
commands. Other buddies may contribute, but Buddy 2 remains accountable for
its contracts, calibration, and evidence.
