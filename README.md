# Group 18 vehicle architecture on the course template

The canonical team checkout is this `Project/All` directory. It now follows
[the official project template](https://github.com/sirfonzie/INF2004_ProjectTemplate)
at commit `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`.

Status: template migration plus hardware-independent controller and Buddy 2
motion-math foundations. Motor/sensor drivers, RTOS adapters
and the dedicated Vehicle Controller task are NOT implemented or started.
No target build, UF2 generation, flashing or physical test was performed in this
migration. Do not flash the course examples or this scaffold onto a connected car.

## Where each buddy works

| Buddy | Role | Application code | Evidence |
|---|---|---|---|
| 1 | Communications, telemetry, remote validation | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motors, encoders, speed control and odometry | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line, barcode and junction perception | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU, terrain, hump detection and height validity | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Sonar, scanning, obstacle planning, integration lead | `app_program/subsystem5/` | `testing/subsystem5/` |

Subsystem numbers match buddy numbers. Keep the template names and read each
folder's Buddy-labelled README rather than creating a second set of buddy folders.

```text
All/
  app_program/
    app_main.c                # usermain(), deliberately inactive scaffold
    app.h                     # shared application lifecycle declarations
    subsystem1/ ... subsystem5/
    vehicle_controller/       # shared portable controller and stop logic
    contracts/                # shared messages, units and IDs
    project_config/           # vehicle pins and future calibration settings
    platform/                 # future application-level kernel/IRQ adapters
    diagnostics/              # future bounded application diagnostics
  build_make/                 # course GNU Make target build
  config/                     # kernel/service configuration, NOT vehicle tuning
  device/ include/ kernel/ lib/ etc/ tools/  # course support, preserved
  testing/
    host/                     # existing portable controller checks
    subsystem1/ ... subsystem5/
    integration/ system/ target/
  videos/                     # evidence links, not video binaries
  TEAM.md                     # owners, pins, resources and interfaces
  docs/
    platform/course-template-review.md
    migration/course-template-migration.md
    template-reference/       # unmodified upstream examples and documentation
```

## Shared controller and collaboration

The Vehicle Controller stays separate from every buddy's subsystem. It will have
its own task on the same Pico W once runtime integration is authorised. All five
buddies may contribute; Buddy 5 leads integration review. Buddy 2 remains the sole
normal runtime motor-output owner. Sensor modules publish evidence/proposals.

Use reviewed branches/PRs. Cross-buddy changes need the primary owner's review;
shared contracts, pins, startup, library hooks and safety policy need affected
owners plus the integration lead. The migration did not change your Git remote,
branch, index or personal `.vscode` files and did not push anything.

## Course-specific rules that affect our design

- `usermain()` is the startup hook, not a second `main()` or `stdio_init_all()`.
  Future startup creates ALL application objects before starting ANY application
  tasks. Current `usermain()` only reports that the scaffold is inactive and sleeps.
- The inspected Make rule finds `.c` directly in `app_program/` and ONE folder
  level below it. Keep driver `.c/.h` pairs directly in the owning subsystem.
  Do not assume deeply nested driver folders are automatically built.
- Put vehicle pins/tuning in `app_program/project_config/`. Root `config/` belongs
  to kernel/service settings. Do not add application drivers under `device/`.
- The built-in service owns CYW43/lwIP/MQTT. Buddy 1 provides application messages
  and validation, not a second task that directly calls the raw network stack.
- Default target profile is dual-core. Local interrupt disabling and `volatile`
  alone are not cross-core synchronization. Runtime locking/affinity remains open.
- Our right line sensor uses GP0 (UART0 TX). A later target build must use USB
  console with `CRASH_REPORT=0`; UART console or crash mirroring conflicts with it.
- Kernel service priorities and the example priority bands are not a measured
  real-time schedule for our car. Agree and measure the application schedule.

## What changed and what remains open

Read `docs/migration/course-template-migration.md` for the old-to-new path map and
backup location, and `docs/migration/standalone-port-inventory.md` for the
Buddy 2-4 source-by-source port decisions. `firmware/` is now just a redirect;
do not put new code there.
Other sibling workspaces (`Project/buddy2`, `buddy2_bench`, `buddy3`, `buddy4`,
`group18-main`) and the leftover nested checkout were not migrated or deleted.

The source-template portion of G0 is inspected; G0 is NOT closed. Build-environment
qualification, runtime integration, ISR legality, timing and memory still need
verification. G1 electrical checks remain open. Servo/body scanning and all
calibration values remain qualified decisions, not measurements.

See `TEAM.md`, `docs/architecture/architecture.md` and `docs/verification/gates.md`.
