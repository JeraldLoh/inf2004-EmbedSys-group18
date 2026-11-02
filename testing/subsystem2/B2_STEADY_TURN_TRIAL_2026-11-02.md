# B2 V15 steady-average turn candidate

Only subsystem2/measured_motion_executor.c changed in this revision. Buddy3,
Buddy5, controller,70mm/s cruise and V14 small-command shaping remain unchanged.
This is a local integrated-trial candidate, not a production qualification.

The operator reports V14 straights are smoother and requests smoother turns
at steady speed. The attachment repeats the earlier pre-V14 log (including
steer20 with81/49 targets), so it is not evidence of V14 wheel performance.

For trial continuous LINE requests, the progressive inner-wheel speed cut is
now transferred to the outside wheel before cap scaling. Desired left+right
therefore stays at twice the requested cruise while the wheel cap permits.
At70mm/s and7/m, nominal targets are approximately100.4/39.6mm/s: mean70.
At full14/m, the outside120mm/s cap requires proportional slowing to about
120/12.1mm/s: mean66.1. Preserving exact70 in that tight turn would require
more outside speed or a wider radius. The cap is not increased.

Both nonzero wheel targets now advance together by the same fraction of their
remaining change, bounded by the existing acceleration limit. When starting
and ending at the same mean speed, turning does not inject an intermediate
mean-speed pulse (apart from integer rounding). Start/stop and cap-limited
transitions still change the mean. Zero targets, reduced hard-cap enforcement,
urgent STOP, expiry and faults bypass ordinary smoothing as before. Finite
goals and obstacle commands retain their old targeting and ramp behavior.

Actual floor speed cannot be guaranteed from targets: motor load, speed PID,
slip and sensor geometry still matter. Check measured speeds and whether the
same curve remains tracked. This does not resolve known line-loss or post-
junction resume rejections. The15s whole-run limit remains.

Compilation succeeded; automated suites skipped per user request. No physical
V15 trial, flash or push performed. First check raised-wheel START/STOP, then
repeat the straight and curve with GP21 ready and retain a fresh serial log.
The shared startup banner retains older50mm/s text; actual cruise is70.
