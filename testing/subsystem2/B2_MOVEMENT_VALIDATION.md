# Buddy 2 movement-test v2 software validation — 29 October 2026

This revision merges the current testing branch (a112ed4, including Buddy 1
telemetry/remote commands) with Buddy 2 measured CAR_RUN integration and the
circuit/movement diagnostic profiles. Conflict resolution preserves Buddy 1
fields and uses measured-executor state/measurement freshness for telemetry.
Main is not a publication target.

## Movement-test changes

- Individual and both-wheel raised checks:30%, up to5 seconds each.
- Long raw trials exist only in B2_MOVE_TEST, with refreshed100ms cutoff.
  Premature watchdog cutoff is a failure; default bench/circuit caps remain.
- Straight PID:100mm/s for5 seconds or500mm wheel-rotation cap.
- Forward/reverse short goals:300mm (30cm). Other optional goals:1m/90degrees.
- Retained65mm wheel diameter,1271/1264 B-channel both-edge counts/revolution,
  ruler track112mm. Untuned PID candidates; production-qualified staysfalse.

## Software evidence

111/111 native B2/circuit/movement CTests pass, including extended raw duration,
short-watchdog refresh, premature-cutoff rejection, actual5-second straight
termination, STOP/stall, startup gates and watchdog observation.19/19 shared
CTest suites pass, including incoming Buddy 1 and remote START cases.

Full RP2040 movement-test and offline CAR_RUN firmware builds pass. Strict
real-header Cortex-M0+ compilation with-Wall-Wextra-Werror passes for merged
motor task, supervisor with CAR_MQTT enabled, B3 telemetry, trial and measured
executor. Normal-profile symbol checks confirm calibration authorization and
extended movement-trial initialization are excluded. Live MQTT/network hardware
and physical car behavior were not tested.

Build movement firmware from build_make:

```sh
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=0 CIRCUIT_TEST=0 B2_MOVE_TEST=1
```

Delivered UF2:group18_b2_movement_test_v2_2026-10-29.uf2.
SHA-256:7161b49afebed3392c172e408baa77dbe868c5b3d5b417a39e62b3d37c3246d9.
The packaged UF2/ELF/map, source snapshot and detailed logs are in the local
output/firmware/b2-movement-test-v2-2026-10-29 delivery directory. Generated
firmware files remain outside Git, following the repository's build-output
ignore rules. Source and build instructions are versioned.

No flashing or motor operation was performed. Physical direction, load/speed,
floor-distance accuracy, effective pivot track and STOP latency are pending.
