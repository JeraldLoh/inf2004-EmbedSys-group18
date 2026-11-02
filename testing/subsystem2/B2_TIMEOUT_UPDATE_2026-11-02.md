# Buddy 2 integrated trial timeout update

The operator requested removing the 15-second whole-run limit and the
3-second no-black stop, and publishing the changes to testing.

Testing already contains commit 5766946, which removes both the integrated
trial supervisor run limit and the executor trial authority deadline. Preserve
that implementation rather than replacing it with the alternative local code.
This update disables the no-black timer for APP_CAR_B2_TRIAL only: both sensors
can report white while the line is between them. A nonzero CAR_LINE_LOST_MS
build override still enables the timer; normal CAR_RUN retains its default.

This removes a possible stop after a junction, not the junction probe/settle
pause. Route selection, Buddy3, Buddy5, controller generation logic and motor
tuning are unchanged. The known LINE_PAUSED_NEEDS_NEW_GENERATION rejection can
still prevent resuming after a junction. No automatic off-track recovery is
added. With both overall timers disabled, do not expect an automatic stop
simply because the line is missing.

Manual START, GP21 STOP, command TTL, finite-goal deadlines, watchdog, progress
faults, wheel-speed/effort caps and production qualification gating remain.
This supersedes the 15-second-limit description in the V18 publication note.

Automated tests skipped at the user's request. Firmware compilation is checked
for publication; no flash, movement or MQTT command is performed. The private
Buddy1 network experiment and WiFi secrets are excluded from this publication.
