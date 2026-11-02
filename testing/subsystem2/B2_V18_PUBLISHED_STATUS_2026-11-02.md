# Buddy 2 V18 testing publication - 2 November 2026

The user reports that V18 works and requests publishing the Buddy2 changes to
testing. This is evidence of the reported manual trial, not a final integrated
qualification or a measured reliability figure. V16's earlier batch was reported
as4 curve passes out of5; do not assign that count to V18.

## Current settings and scope

- Actual explicit trial line cruise70mm/s; max wheel target120mm/s.
- Curvature scaling140000 in the existing trial configuration.
- B2 suppresses the small +/-20 steering hold and progressively maps larger
  line requests to turns, without editing Buddy3 perception or steering code.
- Progressive inside reduction, balanced target calculation and paired wheel
  ramps improve straight/turn target transitions. V16's extra inside15% cut is
  retained; V17's20mm/s minimum is DISABLED in V18 to restore sharper turns.
- V17 named rejection/stop diagnostics, pre-stop target/speed snapshots and
  a separate low-priority trial logger remain. Tag=v18-tight-turn-diag.
- PID gains, encoder calibration,30% effort cap,100ms watchdog, manual START,
  physical STOP,15s whole-run limit and production qualification gating remain.

Only Buddy2 runtime sources plus the existing shared trial configuration and
Buddy2 documentation differ from the fetched testing baseline e04921b. Buddy3,
Buddy5 and shared controller source match that baseline. The single speed/
conversion configuration header is necessary to select the requested B2 trial
settings; it introduces no Buddy3 LINE_* definitions. Other buddies' work is
preserved. Main and Buddy-2 branches are not targets of this publication.

## Evidence and remaining work

V18 firmware source b1eae65 compiled successfully. Its local UF2 is
group18_b2_local_tightturn_v18_2026-11-02.uf2, SHA256
c449e172d50d58b089984f76ea2c5f486e626f1f81228199533f0b530fb15225.
The publication status commit changes documentation only. Generated UF2/ELF/
maps remain local ignored artifacts; versioned source/build recipes are pushed.
Automated suites were not run for these latest changes, as explicitly requested.
No firmware flash or vehicle movement was performed by the agent.

The post-junction LINE_PAUSED_NEEDS_NEW_GENERATION rejection remains unresolved;
it needs coordinated controller authority renewal rather than weakening B2's
old-command rejection. The logs also record a distinct Buddy5 bypass pivot
NO_WHEEL_PROGRESS fault at50/-50mm/s on the right wheel. That finite-goal issue
is not fixed by changing line-turn targets. Barcode calibration, physical
sensor interpretation and line-loss/recovery concerns remain separate items.

The shared CAR startup text still includes older50mm/s wording; use the new
B2 CONFIG message for actual70mm/s settings and firmware identity. None of the
operator observations qualify physical STOP latency or complete obstacle/
junction navigation. Production-qualified remains false.
