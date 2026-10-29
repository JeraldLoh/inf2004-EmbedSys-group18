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
