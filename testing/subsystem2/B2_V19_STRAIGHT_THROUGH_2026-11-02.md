# V19 trial: stronger line turns and no-barcode straight-through

The operator requested publishing both changes to testing.

For nonzero LINE turns, the extra inside-wheel multiplier changes from 85%
to 76.5% (0.85 * 0.90): another 10% reduction of the previous inside target.
It applies symmetrically to left and right turns after existing target/cap
calculation. Outside and zero-curvature targets are unchanged, as are PID
gains, finite distance/pivot goals and obstacle commands. Firmware reports
v19-inner-minus10 and extra_inner=76.5%. Lower inner targets still need a
physical check under load; no sharper-turn performance is claimed yet.

In APP_CAR_B2_TRIAL only, junction evidence without a live pending barcode
route keeps continuous line following active. The controller does not pause,
probe or submit a finite approach goal. T-junctions also default straight;
this policy does not classify junction shape. Valid barcode routes retain
their existing approach/turn/settle sequence and renewed resume generation.
The no-barcode case supersedes the probing descriptions in prior notes.

This changes the shared controller integration policy as explicitly selected
by the operator. Buddy3 and Buddy5 sources remain unchanged. Swaying is not
separately retuned. Both overall timers remain disabled. Manual START,
physical STOP, watchdog, command leases, finite-goal/progress faults and
speed/effort caps remain. Production qualification remains false.

The private WiFi firmware compiled with these changes. The offline testing
profile is also compiled for publication. Automated tests are skipped at the
operator's request. No flash or movement is performed by the agent. Hardware
confirmation is pending. Private WiFi headers and firmware are not published.
