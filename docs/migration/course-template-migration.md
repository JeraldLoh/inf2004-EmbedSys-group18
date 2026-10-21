# Course template migration

## Source and scope

Inspected on 19 October 2026 from
`https://github.com/sirfonzie/INF2004_ProjectTemplate`, commit
`c3b09544cb1fd6dac6ad4b8f523be0af59b44482`.

The goal was to redo the existing architecture layout around the required course
template, not implement or operate the vehicle. The previous conversation
`Plan Vehicle Controller architecture` was read to preserve the shared controller,
buddy boundaries, contracts and Git working location.

Imported unchanged: `build_make/`, `config/`, `device/`, `etc/`, `include/`,
`kernel/`, `lib/`, `tools/`. Copyright/license notices in these files are retained.
The original README, TEAM, application examples, testing guides and video guide
are retained unchanged in `docs/template-reference/`. No template Git metadata
was copied into the team checkout and no remote was changed.

The active demo application was deliberately NOT copied. Its GP19 LED, producer,
consumer and monitor do not represent the five vehicle buddies. New lifecycle
files are explicitly inactive placeholders (`E_NOSPT`), test hooks say SKIP,
and the initial application task sleeps without starting any vehicle tasks.
This does not certify a flashed board as safe: kernel services run before the
application hook and electrical/reset behavior is still unverified.

## Backup

A recovery copy of the previous README, docs and firmware exists outside the
canonical checkout at:

`C:/School-SIT/Y2T1/INF2004-Embedded/Project/template_migration_backups/before-course-template-1387a1c48c6b4936ba34abcaa8e909a1/`

The previous framework is also in the existing Git history. No user workspaces
were deleted. The backup is not a second active development root.

## Path mapping

| Previous path | Canonical path |
|---|---|
| `firmware/app/` | `app_program/vehicle_controller/` |
| `firmware/contracts/` | `app_program/contracts/` |
| `firmware/config/` | `app_program/project_config/` |
| `firmware/buddy1_comms/` | `app_program/subsystem1/` |
| `firmware/buddy2_motion/` | `app_program/subsystem2/` |
| `firmware/buddy3_perception/` | `app_program/subsystem3/` |
| `firmware/buddy4_terrain/` | `app_program/subsystem4/` |
| `firmware/buddy5_obstacle/` | `app_program/subsystem5/` |
| `firmware/platform/` | `app_program/platform/` |
| `firmware/diagnostics/` | `app_program/diagnostics/` |
| `firmware/tests/host/` | `testing/host/` |
| `firmware/tests/target/` | `testing/target/` |
| `firmware/tests/README.md` | `testing/FRAMEWORK_TESTS.md` |

Controller, safety, contract and pinmap implementation content was preserved.
The moved host test and its CMake file only needed relative-path updates; test
assertions are also explicitly retained in Release builds. The controller is
still partial and has not become a complete runtime implementation by moving it.

New code should not go under `firmware/`; its README is only a redirect.
Other `Project/buddy*`, `Project/group18-main`, the old bench pack and the stale
nested `All/group18-main` are untouched. Personal `.vscode` and the nested leftover
are ignored, not deleted. No branch switch, commit, index staging or push was done.

## Design adaptations

1. Course names `subsystem1`-`subsystem5` map one-to-one to buddy numbers. Descriptive
   Buddy-labelled README files preserve discoverability without changing build paths.
2. The Vehicle Controller remains separate/shared with Buddy 5 integration lead.
   A dedicated task remains future work and is not silently placed in subsystem5.
3. Vehicle configuration is distinct from root kernel configuration.
4. The Makefile discovers only direct C files and one subfolder level, so shared
   `.c` code and subsystem drivers stay at that depth.
5. Buddy 1 owns application telemetry/validation, while raw radio/network calls
   belong to the built-in service. The Week 6 two-student-network-task description
   needs a recorded refinement before integration.
6. Default SMP requires cross-core-safe communication and explicit ownership;
   the single-core standalone diagnostic's local interrupt lock is not adopted.
7. USB console with `CRASH_REPORT=0` avoids the line-sensor GP0/GP1 conflict.
8. Source inspection advances G0 but does not replace toolchain, runtime, timing,
   ISR-context, stack or hardware verification. G1 remains open.

## Verification boundary

Static checks and the existing portable host tests are appropriate for this
migration. No `make`, `make -n`, firmware compiler/linker invocation, UF2 build,
flash, network service run or motor/sensor test is part of this task. Even a Make
dry run may evaluate the template's parse-time shell actions, so it is not used.
Actual validation results are recorded in `testing/host/MIGRATION_CHECK.md`.
