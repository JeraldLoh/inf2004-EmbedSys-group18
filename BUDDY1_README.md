# Buddy 1 text MQTT baseline on team main

This port preserves the text protocol before the tribunal binary change.
It adds only Buddy 1 application code, dashboard/tests, a named demo profile and
reviewable service-side hooks. Main's other buddies, contracts and controller
remain intact. Shared network-library changes require team integration review.

## Build for Pico W

In Ubuntu, set PICO_SDK_PATH to your SDK 2.3.0 directory. Copy
config/wifi_secrets.example.h to config/wifi_secrets.h and fill settings locally.
Never commit that local header or publish a credential-bearing UF2.

From build_make:

    make NET=lwip MQTT=1 TEST=1 B1_DEMO_RUN=1 CONSOLE=usb_cdc CRASH_REPORT=0 -j8

Output: mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test_b1demo.uf2.
Only this explicit profile starts Buddy 1's priority-18, 4 KiB task. Default
B1_DEMO_RUN=0 preserves inactive team application startup. USB/radio kernel
services may still run. Do not run the scaffold as an integrated robot image.

## What is implemented

group18/robot1/{heartbeat,telemetry,command,command_result}; QoS 0, non-retained.
Commands are exact ASCII START, STOP, SET_SPEED 0..100; telemetry/results are JSON.
Telemetry interval is the original 1 second; heartbeat 5 seconds; dashboard
heartbeat expiry 15 seconds. Data and command execution are explicitly simulated
(B1_DEMO=1). No physical execution acknowledgement or emergency STOP guarantee.
Source code checks are not proof of hardware WiFi, MQTT or motor timing.

The website is dashboard/web; run dashboard/Start-Website.cmd for port 8080.
Mosquitto uses dashboard/mosquitto-local.conf (TCP 1883, loopback WebSocket 9001).
Pico and laptop use the same reachable broker; the Pico uses laptop LAN IP,
not localhost. Dashboard preview is simulated and never sends robot commands.
Clear retained command messages before testing; the firmware callback cannot
inspect MQTT retained flags. Never retain robot commands.

The radio task alone polls MQTT, including link-down transitions. Retries:
WiFi down/failure 5 s; stalled association/IP 30 s; MQTT 3 s with 15 s attempt
deadline. Bridge buffers are bounded (~8 incoming and vehicle commands), with
one latest outgoing slot per type. Commands expire after 1 s and connection
epochs invalidate queued commands across observed reconnects. Overflow can drop
commands/results; this is not a guaranteed-delivery command protocol.

## Real controller integration remains open

b1_submit_snapshot supplies a complete speed_percent (0..100), distance_cm
(0..100000) snapshot. b1_receive_vehicle retrieves a fresh command. Task context
only; never call from an ISR. Main's Vehicle Controller has no running adapter;
there is no automatic percentage-to-mm/s mapping or real motor command execution.
B1_DEMO=0 requires a reviewed producer/controller adapter and a separate runtime
startup plan. Do not treat standalone connection epoch as vehicle authority.

## Checks and later work

Python: python -B -m unittest discover -s dashboard -p test_model.py
Website: node --test dashboard/web/test.mjs
C: host_tests/protocol_test.c and retry_test.c; lifecycle/default startup tests
under testing/host/test_buddy1_lifecycle.c using buddy1_stubs for host builds.
Existing testing/host CMake suites continue to cover team portable foundations.

Binary codecs, 200 ms combined telemetry, common fault mask, out-of-range event
logging and integrated timing are deferred to the next requirement-change phase.
Dashboard stale-reading presentation also needs checking against FR7/FR8.

### Host test runner

    cmake -S testing/host -B testing/host/build-b1-check
    cmake --build testing/host/build-b1-check
    ctest --test-dir testing/host/build-b1-check --output-on-failure

See testing/subsystem1/TEST_LOG.md for the dated source/build evidence.
The generated files currently use placeholder network settings. Fill the ignored
config/wifi_secrets.h and rebuild the B1_DEMO_RUN=1 profile before hardware use.
Do not flash the unsuffixed inactive image expecting Buddy 1 telemetry.
