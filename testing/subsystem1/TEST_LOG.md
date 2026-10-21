# Buddy 1 Communications test record

This is a proposed plan, not test evidence. No hardware test was run during
folder migration. Define exact preconditions, thresholds, procedures and evidence
before execution. Electrical work requires G1; active motion needs explicit
bounded-test approval and a physical power cut.

## Proposed cases

| ID | Purpose | Method and acceptance | Status |
|---|---|---|---|
| S1-U10 | Validate remote command syntax without network I/O | Procedure and quantitative acceptance to be agreed | Not run |
| S1-U11 | Exercise bounded telemetry ingress and overload policy | Procedure and quantitative acceptance to be agreed | Not run |

The code's subsystem1_test() is only a SKIP placeholder and is not invoked by
the inactive app_main. Do not report it as a subsystem PASS.

## Results

Append actual runs only. Keep failures and link the exact tested commit, profile,
configuration and raw log/video. Upstream example rows are archived separately
under docs/template-reference and are not results for this team.

| Date | Test ID | Commit/profile | Tester | Result | Evidence | Notes |
|---|---|---|---|---|---|---|

## Text port validation 22 October 2026

These are source/build checks, not hardware results.

- PASS: 7 CTest executables: existing vehicle controller, motion math,
  perception math; Buddy 1 text protocol, retry policy, default startup and
  standalone lifecycle/failure handling.
- PASS: Python dashboard model, 5 tests.
- PASS: website protocol/model, 6 tests.
- PASS: Pico W standalone text demo target generated ELF/UF2 with Ubuntu Arm
  GCC 14.2.1 and SDK 2.3.0. B1_DEMO_RUN=1, NET=lwip, MQTT=1, TEST=1,
  CONSOLE=usb_cdc, CRASH_REPORT=0, default SMP=1. Placeholder settings only.
- PASS: source comparison confirms no modifications to team contracts,
  vehicle controller, pinmap, subsystem2-5, kernel or tracked config files.
- NOT RUN: Pico flashing, physical WiFi/MQTT, on-target parser tests,
  WiFi/broker recovery, integrated control deadlines and task stack usage.

Host lifecycle tests use kernel doubles; they verify startup branching,
creation/start error propagation and dormant-task deletion, not RTOS timing.
The three B1-U checks in subsystem1_test.c are compiled but not hardware PASS.

- PASS: default inactive application profile also generated ELF/UF2 with
  B1_DEMO_RUN=0 and otherwise the same network/test configuration.
  Static sections: text 330236 B, data 96 B, bss 82772 B.
- Standalone demo sections: text 333840 B, data 96 B, bss 82784 B.
  These section sizes are build observations, not peak runtime RAM measurements.
