# Buddy 2 publication verification — 28 October 2026

This change packages the Buddy 2 driver foundation and bounded runtime bench on
top of main at 4af4b10, preserving the existing Buddy 3 work. It is prepared in a
clean checkout; unrelated local codec, diagnostic, sensor and configuration work
from the development checkout is not included. Firmware remains software-tested
only: no board was flashed, motors/servos operated or wiring changed.

## Contents and dependencies

- Production motor/encoder drivers, safe lifecycle, persistent STOP inhibit,
  coherent raw count/time/epoch snapshots and candidate calibration provenance.
- Core-0 IRQ/clock/SMP lock binding, shared GPIO dispatcher, alarm0 cutoff,
  controller STOP reconciliation and manually triggered 15%/250 ms trials.
- Standalone fake-HAL/MMIO/kernel/BSP tests and the operator guide. The regular
  application remains inactive; the bench requires explicit B2_BENCH=1.
- Minimal prerequisites: the controller's existing proposed hardware-fault
  storage/helpers allow bench admission to preserve hardware inhibits; host
  targets using the controller link that helper. No fault-bit meanings or new
  protocol commands are assigned. The PWM peripheral is released even when
  physical timers are disabled; device include and Windows host-tool filename
  fixes make the course image reproducible on the development host.
- The entry hook adds only the opt-in B2 bench. It preserves main's existing
  normal/TEST behaviour and does not import the older component diagnostic.

## Results for this main-based source

| Check | Result |
|---|---|
| Dedicated B2 CTest suite | 38/38 PASS, GCC 15.2.0, C11, warnings-as-errors, assertions on |
| Shared testing/host suite | 4/4 PASS, including Buddy 3 sensor tests |
| Course B2 bench firmware | Compile/link PASS, ARM GCC 15.2.1, SDK 2.3.1, SMP1/USB CDC/UART0-free/WIFI=none/TEST0/B2_BENCH1 |
| Normal firmware | Compile/link PASS; normal startup remains inactive |
| Hardware qualification | NOT RUN; <=50 ms physical STOP, direction, edge loss, floor/load/PID/distance/pivot/ramp acceptance pending |

The separate app_program CMake test aggregation has a **pre-existing missing
Buddy 4 test file**: subsystem4/CMakeLists.txt references
testing/host/test_lsm303_decode.c, which is absent from the parent main commit.
This publication does not create that failure or alter Buddy 4. Use the two
verified test entry points below; no claim is made that the aggregate ran.

From the repository root, with GCC/CMake/Ninja available:

```sh
cmake -S testing/subsystem2/b2_01 -B ../build-buddy2 -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build ../build-buddy2
ctest --test-dir ../build-buddy2 --output-on-failure

cmake -S testing/host -B ../build-host -G Ninja -DCMAKE_BUILD_TYPE=Debug
cmake --build ../build-host
ctest --test-dir ../build-host --output-on-failure
```

Build the course image in a disposable checkout; the existing makefile clears
generated objects when a profile changes:

```sh
make -C build_make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=1 PICO_SDK_PATH=/path/to/pico-sdk
```

Normal firmware uses B2_BENCH=0. Generated ELF/UF2/map/native binaries are ignored
and are not committed. Earlier local UF2s were built from the development
snapshot, not this newer main-based tree; rebuild this source when associating
a hardware log with this commit. Record the resulting SHA-256/configuration.

See [the staged test guide](B2_RUNTIME_TEST_GUIDE.md) for the later authorized
hardware session. Do passive disabled-state checks before any raised-wheel
trial; time-ended is not distance completion, and software STOP-call timestamps
are not physical output measurements.
