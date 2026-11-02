# Local V18: restore V16 line-turn targets, retain diagnostics

The operator reports V17 no longer completes the curve and runs wide. Its
log confirms floor_mask0x2, line targets120/20mm/s and measured112/29 or
111/33: the actual wheel-speed differential is smaller still. Outside target
is already120mm/s and effort approaches300per-mille. No limit is increased.

V18 disables B2_LINE_MIN_ROLLING_MM_S (20 ->0), restoring V16's line target
geometry and retaining V17's named diagnostics, firmware identity and logger.
At full right correction targets return to approximately120/10.3mm/s, mirrored
for left. Straight remains70/70; small steering holds remain suppressed.
The low target may intermittently stall, as observed in earlier trials; this
is a geometry rollback, not proof of calibrated low-speed control.

The same multi-run attachment contains an independent obstacle bypass pivot
failure: target50/-50mm/s, NO_WHEEL_PROGRESS, result7, wheel_mask0x2, followed
by controller motion fault0x8. This is a finite Buddy5 goal and is unaffected
by the line-only floor. The goal request's50mm/s cap wins over B2's candidate
60mm/s pivot floor. That manoeuvre still needs its own load/supply/goal-speed
investigation; do not bypass the fault or exceed the caller's cap.

The new logs also confirm LINE_PAUSED_NEEDS_NEW_GENERATION after junction
completion (request_gen=executor_gen=paused_line_gen=2). That known shared-
controller handoff remains unresolved. No Buddy3/controller/Buddy5 source
was changed. Compilation succeeded; automated tests skipped per user request.
No flash, vehicle operation or remote push performed. Production qualification
remains false. Verify raised-wheel START/STOP, then retry the same clear curve
without an obstacle to separate line tracking from avoidance. Retain the full
fresh B2 CONFIG/TRACK/STOP DETAIL log. Firmware tag=v18-tight-turn-diag.
