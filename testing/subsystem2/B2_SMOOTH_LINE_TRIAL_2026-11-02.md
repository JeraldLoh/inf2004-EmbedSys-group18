# Local B2 straight/turn smoothing candidate V14

Scope: only app_program/subsystem2 code changed for this revision. Buddy3,
Buddy5, shared controller and existing70mm/s trial speed configuration remain
unchanged. No remote push or hardware operation performed.

The restored Buddy3 hold +/-20 maps to +/-2.8 reciprocal metres in the
controller's legacy micro-curvature request. In explicit CAR_B2_TRIAL line
requests only, B2 maps that small range to zero curvature. Above it, B2
progressively remaps the remaining range to the original full14/m turn.
For example, steer30 maps to about2/m instead of4/m; steer60 maps7/m;
steer100 retains14/m. Left/right symmetry and the ordinary input lease/
source/authority checks are retained. This intentionally filters weak line
corrections too; gentle-curve response needs physical checking.

The additional inner-wheel cut now grows from0 to15% with turn strength,
rather than applying15% to every nonzero turn. Full-turn capability and the
120mm/s wheel cap are retained. Ordinary nonzero LINE target decreases are
ramped with the existing200mm/s^2 limit, like increases. Zero targets,
reduced hard caps, STOP, command expiry and faults retain immediate stop/
cap enforcement. Finite goals and obstacle commands keep their old behavior.

Straight desired targets are70/70mm/s even if controller status still reports
steer+/-20; targets converge during the bounded wheel-speed transition.
Measured speed may differ under load and PID fluctuations are not eliminated
by changing setpoints alone. No guarantee of straight floor position follows
from equal encoder speed. The3s line-loss stop,15s whole-run limit and known
post-junction resume rejection remain unresolved.

Compilation succeeded. Automated tests skipped at the user's request. No
physical V14 result yet. First check raised-wheel START/STOP, then a short
straight followed by the same curve, retaining the complete serial log.
The existing shared startup banner still contains older50mm/s text; the
actual trial configuration remains70mm/s and is recorded in the manifest.
