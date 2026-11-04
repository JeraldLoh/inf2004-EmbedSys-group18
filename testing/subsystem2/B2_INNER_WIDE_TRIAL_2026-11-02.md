# B2 V16 inner-wheel reduction for running wide

The operator confirmed the V15 departure was wide to the outside of the bend.
The log shows some inside-wheel overspeed (target47, measured64mm/s) and
outside-wheel underspeed (target92, measured81), but periodic snapshots cannot
establish the precise floor trajectory or guarantee a fix by changing targets.
No motion fault or command rejection occurred in that run. It ended with
physical STOP; a later run ended with the3s no-black timeout.

V16 is a Buddy2-only local trial: after V15's paired target calculation and
wheel-cap scaling, multiply the inside target by0.85 on nonzero-curvature LINE
requests. This is an additional15% relative to V15, not a new PWM effort value.
The outside target is not raised further. Moderate-turn targets of about
100.4/39.6 become100.4/33.6mm/s; full turn120/12.1 becomes120/10.3, mirrored
for left. Straight70/70 and the small-command deadband remain unchanged.

This intentionally gives up exact mean70mm/s during turns in favour of a
sharper, slightly slower path. Paired target interpolation is retained to
smooth entry/exit. Finite goals, obstacle commands, Buddy3/Buddy5/controller,
wheel PID, effort/speed caps, STOP and watchdog paths are unchanged.
The low inner target may not be tracked under load; inspect measured speeds
and floor behaviour, and do not claim that the new target guarantees tracking.

Compilation succeeded; automated tests skipped per user request. No physical
V16 test, flash or push performed. First check raised-wheel START/STOP, then
repeat the same straight/curve with GP21 accessible and preserve a fresh log.
The15s limit, line-loss logic and known post-junction resume issue remain.
The shared startup banner retains older50mm/s text; actual base remains70.
