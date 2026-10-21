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

## Heartbeat gap investigation 22 October 2026

User hardware observations: Pico obtained a WiFi IP, Mosquitto received
telemetry/heartbeats, dashboard values matched subscriber values, and demo
speed changed to 42 then returned to 0 after STOP. These are communication/demo
observations, not motor or sensor integration tests. Duplicate broker service
and Windows console selection pauses were observed during setup.

Received heartbeat uptime values 1593180, 1603200, 1613220, 1618230 imply gaps
of 10020, 10020 and 5010 ms. This does not identify which pipeline stage lost
messages or establish on-time broker arrival.

Reproduced source defect: heartbeat scheduling advanced hb_at before put()
could obtain its nonblocking mutex. Forced 20 ms and 100 ms producer contention
made the original bridge miss one scheduled heartbeat in host tests. Normal
no-contention test passed. Fix: only advance hb_at and heartbeat sequence after
slot insertion succeeds; retry on the next task iteration otherwise.

After fix: all 10 CTest executables/cases pass, including three real-bridge
fake-clock tests (normal, busy once, busy 100 ms). Actual RTOS scheduling and
network behaviour are not simulated by these tests. TEST=1 firmware adds [HB]
lines showing generated_ms, publish_attempt_ms and result. result=0 means
lwIP accepted the publish, not broker delivery. Hardware retest is PENDING.

## Hardware text demo retest and dashboard alignment 22 October 2026

The following observations were demonstrated by the user on the Pico W; they
are not results from an integrated car or independently measured safety timing.

| Check | Observation | Scope / evidence |
|---|---|---|
| WiFi + MQTT | Pico obtained 172.20.10.2; MQTT ready; broker received heartbeat/telemetry. | Standalone Pico W; no real sensors/motors. |
| Command/telemetry round trip | Requested 42%, simulated speed became 42; STOP returned simulated speed to 0. | logs/simulated-speed-42.png; logs/simulated-stop-zero.png. |
| Heartbeat after fix | Received uptime 30060, 35070, 40080, 45090, 50100: four 5010 ms gaps. Subsequent shown publish attempts result=0. | logs/heartbeat-after-fix.png shows continuing generation/publish samples. Local publish acceptance is not delivery acknowledgement. No hard real-time guarantee. |
| Heartbeat loss | User unplugged Pico; dashboard reported heartbeat lost, commands locked; screenshot age 26.7 s. | logs/heartbeat-loss-dashboard.png. Exact transition latency was not independently measured. |
| Recovery after reboot | Replugging Pico restored heartbeat/readings. | logs/recovery-after-power-cycle.png. |
| Hotspot interruption | User reports restoration without reboot after switching hotspot off/on; dashboard fresh again. | logs/recovery-after-hotspot-test.png. Recovery duration and unchanged power not proven by screenshot alone. |

Setup findings: a duplicate Mosquitto Windows service listened on loopback while
manual broker served the Pico. Service was stopped during the test. Console
text selection paused broker processing; Esc resumed it. These are distinct
from the reproduced firmware producer-lock bug.

Dashboard alignment: retain last readings across heartbeat expiry and broker
transport loss; explicitly mark stale values and reception age; show last
telemetry reception time in SGT. Heartbeat timeout says Communication
Unavailable. A fresh heartbeat never refreshes the age of old telemetry.
Commands remain gated on a fresh heartbeat. An explicit operator disconnect or
new connection-settings session resets data to avoid mixing sources.

Verification after display change: 8 website tests pass (including retained
stale data, broker recovery and telemetry age); isolated browser check on port
19001 confirmed 42% / 93 cm retained with stale labels and controls disabled
after heartbeat expiry. This UI test sent no Pico commands. Actual hardware
retest of the revised stale-value presentation remains available to the user.

Still open: real sensors/motors, combined-task timing, long-duration reliability,
queue flooding, protocol robustness beyond existing cases, fault integration,
and binary/200 ms requirement change. These results do not close A1 for a full
robot. User screenshots are reference evidence, not automated assertions.
