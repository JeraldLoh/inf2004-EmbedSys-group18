# Course template migration verification

Date: 19 October 2026. Scope: local, uncommitted structural migration from the
previous framework on baseline `ffb9352`. Course source snapshot:
`c3b09544cb1fd6dac6ad4b8f523be0af59b44482`.

These results apply to the migrated files checked in this task, not to the
historical baseline alone. Commit the reviewed migration and rerun before using
it as versioned tribunal/demo evidence. No target or physical results are implied.

## Checks actually executed

| Check | Actual result |
|---|---|
| Static layout script | PASS: 13 active target C files fit the exact one-level Make wildcard |
| Buddy paths and relative includes | PASS; no old-root C duplicate or active LED/producer demo |
| Upstream support-file hashes | PASS: all 602 imported support files match the source snapshot |
| Portable implementation hashes | PASS: controller/safety C/H, contracts H/H and pinmap H (seven files) are byte-identical to the recovery copy |
| Direct native GCC strict C11 compile/run | PASS: existing portable controller checks |
| Host CMake Release build with assertions retained | PASS |
| Host CTest | PASS: 1 executable, containing five existing behavioral checks; 0 failures |
| Git diff whitespace check | PASS for tracked changes |

The five existing checks cover normal tracking without a route, confirmed
junction without a route, single route reservation, ignoring a pre-stop goal
result, and terrain-cap/stop composition. They do not validate the whole controller.

## Host environment and captured output

Native host GCC 15.2.0, CMake 4.3.4, Ninja; Release configuration. Compilation uses
`-Wall -Wextra -Werror -Wpedantic` and `-UNDEBUG` so C assertions remain active.
The first sandboxed CMake compiler probe could not find its temporary Ninja file;
the approved retry outside the sandbox configured, built and passed. This was
a host environment issue, not a target-build success or firmware qualification.

Actual captured summary:

```text
PASS: 13 target C files fit the one-level Make discovery rule.
PASS: subsystem paths, relative includes and old-root redirect verified.
Static check only: no Make, firmware build, flash or hardware operation performed.
vehicle_controller host tests passed
1/1 Test #1: vehicle_controller_host_tests .... Passed
100% tests passed, 0 tests failed out of 1
```

The successful CMake host build and its complete CTest log were generated under:

`C:/Users/rbola/AppData/Local/Temp/group18-host-cmake-7ca19fd1f4a94a5fbe34aef59cd757dd/`

Temporary artifacts are not tracked or treated as permanent evidence links.

## Not performed

- No course target Make invocation, including dry run.
- No ARM firmware compile/link, UF2 generation, flashing or Pico operation.
- No runtime task creation, ISR/SMP qualification or kernel timing measurement.
- No motor, sensor, voltage, calibration, obstacle or autonomous-mission tests.
- No staging, commit, push, branch change, remote change or teammate messaging.

G0 runtime qualification, G1 and hardware readiness remain open.

## 21 October 2026 porting addendum

After the structural migration, the hardware-independent parts of the Buddy 2
motion geometry, Buddy 3 Code 39 decoder, and Buddy 4 LSM303 raw conversion were
ported into their course subsystem folders. The earlier results above are kept
as historical evidence and are not rewritten.

Checks executed against the current uncommitted tree:

| Check | Actual result |
|---|---|
| Static layout and API guard | PASS: 16 target C files fit the one-level wildcard; no standalone `main`, Pico SDK startup/delay, or raw network call entered `app_program` |
| Strict host build | PASS with GCC 15.2.0, `-Wall -Wextra -Werror -Wpedantic -UNDEBUG` |
| Host CTest | PASS: 3 executables, 0 failures |
| Vehicle Controller | PASS: existing 5 behavioral checks |
| Buddy 2 motion math | PASS: invalid calibration, conservative round trip and millidegree pivot conversion |
| Buddy 3/4 pure processing | PASS: synthetic complete `*A*` maps to LEFT; signed LSM303 +1000/-1000 mg decoding |
| Git whitespace check | PASS |

These are computer-only tests of deterministic C code. They do not validate the
barcode printout, sensor sampling, I2C transactions, motor/encoder drivers,
calibration, RTOS timing, target linking, flashing, or vehicle behavior.
