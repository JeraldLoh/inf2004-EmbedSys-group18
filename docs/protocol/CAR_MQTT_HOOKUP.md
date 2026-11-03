# Buddy 1 CAR_RUN integration checkpoint

## Status

Buddy 1 is connected to the existing Buddy 2/3/5 CAR_RUN application. The user reported WiFi/MQTT/dashboard connectivity and working GP20 hold-and-release START. However, line following behaved incorrectly compared with the original testing firmware. This regression is unresolved: WiFi/task timing is a hypothesis, not a confirmed cause. No sensor-polarity, line-following or WiFi-priority fix has been made. This is a testing checkpoint, not a qualified robot release.

## Build

From build_make/, with the local Pico SDK path configured:

```
make -j6 CAR_RUN=1 CAR_MQTT=1 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

One image: `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2`. Credentials stay in ignored `config/wifi_secrets.h`; UF2/ELF/build outputs must remain local. Offline CAR_RUN remains available with CAR_MQTT=0 WIFI=none NET=none MQTT=0.

## Changes

- Buddy 1 bridge: merge independently produced sections under its mutex, preserve producer timestamps, reject invalid values, select real telemetry in the live profile and use an ARM-safe busy counter.
- Buddy 2 subsystem2_motion.c/.h: telemetry timestamp and first-sample validity only; no motor/encoder algorithm changes. Target speed, integrated distance and result acknowledgement remain unavailable.
- Buddy 3 subsystem3.c: owner-task telemetry copies of line, steering, ADC/calibration, retained route/counters and actual completed bar duration. No event queues are consumed and no sensing/decoding algorithm changes. Pattern remains unavailable. Probe/default routes do not display a fabricated barcode duration.
- Buddy 5 subsystem5.c: owner-task range completion timestamps, separate front/latest direction, avoidance state and metrics. Invalid statuses never become valid distances. Servo/heading are commanded, not feedback. Profile/plan/failure-code detail and event streaming are not exported.
- Shared car_run_target.c: Buddy 1 lifecycle, wheel/controller telemetry collection and bounded remote command handling in the existing supervisor.
- Shared controller_task.c/.h and controller_service.c/.h: conditional remote START tagged with STOP generation; newer STOP invalidates older restart requests. Remote START clears remote STOP only, not physical/fault latches. Existing physical STOP, line-loss stop and run limit remain.
- Build profile, dashboard bar-duration validity display and corresponding regression tests/documentation.
- Buddy 4, pinmap, motor drivers, line-following algorithm and avoidance algorithm are unchanged.

## Command and reading limits

START requests automatic driving at the existing configured speed; no dashboard speed override. STOP wins within an ingress batch and uses the controller sticky STOP lane. Duplicate START does not reset the run timer. Queued START waits up to approximately 1 s for AUTO or requests STOP. MQTT command_result=queued confirms queue insertion, not execution. QoS 0 is not guaranteed delivery; no new network-loss emergency-stop policy is added.

Telemetry uses the current binary schema approximately every 200 ms. Source timestamps expose stale data; invalid submissions keep the last valid snapshot. Real sonar measurements start when existing avoidance is armed in AUTO, so No reading yet while parked is expected. Calibration false cannot distinguish running from failed with the current fields.

## Checks

- WSL: `python3 testing/host/verify_buddy1_car.py` (GCC required).
- Windows Node: `node testing/host/buddy1_car_mqtt.mjs` (isolated loopback Mosquitto on 19001; no robot commands).
- Existing web suites: 33 tests passed; Python dashboard suites: 9 passed.
- Buddy 1 portable C suites, stale START/new STOP regression and existing controller_service tests passed.
- Actual merged bridge packet crossed isolated MQTT to the dashboard decoder/model; combined ARM firmware compiled.
- Full team CMake tests still reference missing Buddy 4 test files and are not claimed passing.
- These automated checks do not establish correct combined hardware behavior. Keep wheels raised for initial motor checks; physical line following still needs diagnosis.

## Supplementary live status (binary v1)

`<prefix>/live_status` adds owner-produced metadata without changing telemetry v2/v3.
The 64-byte, little-endian frame is published every 200 ms when at least one owner
has submitted data. QoS 0, not retained, one latest-value slot; it is not an event log.
No new motor commands or sensing algorithms are introduced.

| Offset | Bytes | Meaning |
| --- | --- | --- |
| 0 | 4 | `B1 01 04`, then demo flag 0/1 |
| 4 | 4 | Kernel uptime ms |
| 8 | 1 | Present bits: motion/controller=1, line=2, avoidance=4 |
| 9 | 1 | Motion flags: targets valid=1, qualified=2, searching=4 |
| 10 | 1 | Applied line/barcode hump pause, 0/1 |
| 11 | 1 | Avoidance flags: fast S-curve=1, bypass active=2, clearance reverse active=4 |
| 12 | 4 | Motion sample time ms |
| 16, 20 | 4 each | Signed left/right commanded target mm/s |
| 24, 28 | 4 each | Controller line searches / failures |
| 32, 36 | 4 each | Applied line state / avoidance sample times ms |
| 40 | 1 | Zero-based segment index bits 0-2; segment count bits 3-5 |
| 41 | 1 | Chosen side: none=0, left=1, right=2 |
| 42 | 2 | Reserved, zero |
| 44, 48 | 4 each | Clearance back-off counter / active encounter elapsed ms |
| 52, 56, 60 | 4 each | Motion accepted / rejected / driver errors |

All counters/times are unsigned 32-bit. Unlisted flag bits must be zero.
A segment index equal to the count means finishing; an inactive bypass has zero
index/count. Segment count can vary; the display does not invent segment names.
Owner snapshots are merged under Buddy 1's nonblocking mutex. Each group's own
sample timestamp and this topic's reception age determine freshness (<1500 ms),
including kernel wrap handling. Missing/stale metadata is shown as not reported.
This feed never refreshes the heartbeat or proves calibration passed. Qualification
is separate from calibration; B2's measured wheel speeds are encoder magnitudes,
while these commanded targets are signed. Bandwidth: 64 bytes per update, in
addition to the existing telemetry and MQTT headers. Diagnostics' telemetry size
still describes the original telemetry packet only.

PC checks: `node --test dashboard/web/*.test.mjs dashboard/web/test.mjs` and
`python3 testing/host/verify_buddy1_car.py`. The optional isolated MQTT roundtrip
also exercises this topic. These checks do not establish on-car behavior.

Current testing integration at `b1a63a9`: the team reverted Buddy 4/hump pausing
and the S-curve bypass. Buddy 3 therefore does not submit the optional line-pause
group; the UI reports it as not reported. Buddy 5 reports the seven-leg bypass
with fast=false. Reserved codec support remains compatible for future producers.
The team website launcher now uses port 8080.

### Local adaptation to cfc1905

The active base is cfc1905, before Buddy 4 integration. No reverted control,
radio-priority or motion-qualification changes are restored by this reporting hook.
Live-status v2 is 76 bytes: the first 64 bytes retain v1 layout except byte 1=2.
Offsets 64, 68 and 72 add unsigned little-endian 32-bit active left/right both-edge
encoder counts/revolution and wheel diameter mm. The browser still decodes v1;
absent settings stay Not reported. These active settings are read by the existing
Buddy 2 telemetry accessor under its lock. Configured is not a live calibration
pass/fail result. The current website launcher uses port 8080.

At cfc1905, default CAR_B2_TRIAL=0 uses the unqualified, zero-gain candidate
profile. The WiFi build preserves this; it does not reinstate later driving sign-off.

Current base 61f8836 supersedes the cfc1905 motion limitation above. The team
restored the qualified v6 CAR_RUN default, combined WiFi support and radio
priority 13. Local reporting changes preserve those settings.
