# Vehicle configuration

This area will hold authoritative, unit-labelled configuration. Defaults must
be distinguishable from calibrated values and linked to test evidence. Safety
limits must not be silently disabled by subsystem tuning.

This is vehicle-level configuration. Root `../../config/` is the course kernel
and service configuration; do not put wheel tuning there. `pinmap.h` is a
preserved provisional declaration only and performs no GPIO setup. Record
pin/peripheral claims in `../../TEAM.md` before hardware use.
