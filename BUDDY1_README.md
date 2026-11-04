# Buddy 1 — demo WiFi/MQTT and live dashboard

This guide covers normal startup, private configuration, building and flashing.
The local demo integration reports the current Buddy 2/3/5 code. It does not change
motor control, PWM caps, line following/search, watchdog, pins or task priorities.
The base is 61f8836. Only START/STOP are implemented as MQTT commands; a
character-to-route configuration command is not implemented by this interface.

## Normal startup (already flashed)

1. Connect the laptop to the same 2.4 GHz WiFi/hotspot configured in the Pico.
2. Run `ipconfig`. The WiFi IPv4 address must match the broker address compiled
   into the Pico. Changing that setting requires rebuilding and flashing.
3. Run one Mosquitto broker. From the repository root in Command Prompt:

   ```cmd
   "C:\Program Files\mosquitto\mosquitto.exe" -c dashboard/mosquitto-local.conf -v
   ```

   PowerShell needs the call operator instead:

   ```powershell
   & "C:\Program Files\mosquitto\mosquitto.exe" -c dashboard/mosquitto-local.conf -v
   ```

4. Power the already-flashed robot normally, without BOOTSEL. Keep wheels raised
   for the first command check. Physical STOP and power removal stay accessible.
5. Double-click `dashboard/Start-Website.cmd` in File Explorer. Open
   `http://127.0.0.1:8080/` and keep both terminal windows open.
6. Exit Preview, connect to `ws://localhost:9001` using topic prefix
   `group18/robot1`, and wait for a fresh rover heartbeat and readings.
7. Start rover requests autonomous operation at the onboard speed. Stop requests
   stopping. A queued reply does not confirm physical movement or execution.

Website port 8080 is different from MQTT TCP 1883 and browser WebSocket 9001.
Ctrl+F5 refreshes dashboard changes; browser changes alone need no Pico flash.
Preview explicitly uses generated samples and is not a hardware test.

## Private settings and prerequisites

Use ignored `config/wifi_secrets.h`, copied from `.example.h` only if missing.
Fill in WIFI_SSID, WIFI_PASSWORD, MQTT_BROKER_IP (laptop WiFi IPv4, not localhost),
MQTT_BROKER_PORT=1883 and a unique MQTT_CLIENT_ID. Do not commit this header or
share binaries containing private credentials. The Pico does not follow the
laptop to a different WiFi automatically.

Build prerequisites: WSL Ubuntu, Make, Arm GCC and Pico SDK with required
submodules (this setup uses SDK 2.3.0). Python starts the website; Node is needed
only for the optional dashboard tests. Windows Mosquitto supplies the broker.

## Build one combined robot/WiFi firmware

In Ubuntu (adjust paths on another laptop):

```bash
export PICO_SDK_PATH=/mnt/c/Users/ftna/.pico-sdk/sdk/2.3.0
cd /mnt/c/Users/ftna/Desktop/emb/inf2004-EmbedSys-group18/build_make
make -j6 CAR_RUN=1 CAR_MQTT=1 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

Wait for successful linking and elf2uf2 generation. Output:

```text
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2
```

The team's default v6 car profile is selected by CAR_RUN=1 (historical flag
CAR_B2_TRIAL defaults to 1). The existing motor cap remains 30% for normal wheel
control and 40% for pivots. The existing radio priority remains 13 and motor
watchdog 100 ms. No later 40% normal cap or watchdog experiment is included.
CAR_B2_TRIAL=0 selects the old unqualified zero-gain diagnostic profile.

## Flash when firmware/compiled settings change

Stop Serial Monitoring, unplug USB, hold BOOTSEL while reconnecting, then release
when RPI-RP2 appears. Copy the newly built combined UF2 to that drive. It disappears
as the Pico restarts. One UF2 contains the whole application; flashing each
buddy separately would overwrite the previous firmware. Normal power-on needs
neither BOOTSEL nor another build. A successful build is not an on-car test.

## Reported data and calibration

- Buddy 2: measured wheel-speed magnitudes, validity, signed commanded targets,
  controller mode, stop/fault masks, search counts, qualification and active
  encoder counts/revolution plus wheel diameter. Configured encoder settings
  are not a live calibration pass/fail result. Distance and goal acknowledgement
  remain unavailable unless their producer explicitly supplies them.
- Buddy 3: IR state/steering, ADC/thresholds where available, retained barcode
  results/counters. The existing ADC calibration routine now reports Not started,
  Calibrating, Passed or Failed (low contrast), without changing its decisions.
  Edge mode does not invent a calibration result.
- Buddy 5: separate scan-start-front/latest ranges and statuses, timestamps,
  commanded direction/servo/heading, avoidance state, bypass-leg progress and
  existing metrics. Servo position is commanded, not physical feedback. No
  sonar/servo calibration result is exposed: those remain Not reported.
- Buddy 4: not connected in this demo integration. Its card stays unavailable
  outside Preview. No missing field is filled with sample data on the live path.

At 61f8836 the original no-black line-search setting is still 2000 ms. Both-white
is also Buddy 3's centred state; that ambiguity is not resolved by reporting.
Known captures have included watchdog cutoff and recovery-failed stops. These
reporting changes do not claim to fix either behavior.

## MQTT contract and freshness

All topics use the selected prefix (default group18/robot1), QoS 0, non-retained.
Only the WiFi service calls raw CYW43/lwIP/MQTT APIs. Owner tasks copy bounded
snapshots to Buddy 1 through a nonblocking mutex; it keeps latest values, not
an event history. No dashboard code directly controls GPIO/PWM.

| Topic suffix | Payload | Nominal interval |
| --- | --- | --- |
| heartbeat | JSON | 5 s |
| telemetry | Existing binary v3, v2 web decoder compatible | 200 ms |
| live_status | Binary type 4 v3, 80 bytes; web supports v1/v2 too | 200 ms |
| diagnostics | Binary validation/rejection counters | 1 s |
| command | Binary v2 START/STOP, command ID; 8 bytes | Operator request |
| command_result | JSON queued/rejected/queue_full/simulated | Request result |

Each section has its own sample time. Supplementary groups expire at 1500 ms;
heartbeat alone cannot make old sensor readings current. Browser-reception age
is also included. Missing/stale status is not interpreted as a healthy state.
400 bytes/s of extra live-status payload is added, excluding MQTT headers.
Diagnostic telemetry-size comparison still counts the original telemetry packet.

Activity stream messages show observation time and rover uptime for state changes,
including stop masks, faults, scans/searches and calibration; duplicate states
are suppressed. Mission names follow the contract. It logs command IDs/results
and connection loss/recovery without treating queued as physical execution.
Updates missed between snapshots or while disconnected cannot be reconstructed.

## Troubleshooting and tests

Waiting for rover: check laptop WiFi, compiled broker IP, one broker and a fresh
heartbeat. Browser connected only means browser-to-broker. For stopped movement,
read controller stop flags and the existing USB serial log at 115200 before
restarting. Do not disable the watchdog to mask a fault. Never assume the fault
is hardware solely from a shared controller-stop flag.

No website: run the launcher and use its printed port. No Pico messages: check
for a loopback Mosquitto service competing with the LAN broker (`netstat -ano`).
Do not run retained movement commands or publish plain ASCII START to the binary
command topic. Keep this anonymous broker configuration on a trusted local LAN.

From the repository root, in Ubuntu:

```bash
cmake -S testing/host -B build/demo-host
cmake --build build/demo-host -j6
ctest --test-dir build/demo-host --output-on-failure
python3 testing/host/verify_buddy1_car.py
```

With Node installed:

```text
node --test dashboard/web/*.test.mjs dashboard/web/test.mjs
```

The optional `node testing/host/buddy1_car_mqtt.mjs` roundtrip uses its own broker
port 19001 and synthetic bridge fixtures; it never sends car commands. Passing
PC tests and firmware compilation do not establish on-car timing, sensor accuracy,
barcode calibration quality, line recovery or physical STOP latency. This local
integration needs a combined hardware retest. No push/flash is automatic.

## Exact robot-side files touched by this local integration

| File | Change |
| --- | --- |
| app_program/subsystem1/buddy1_bridge.c | Copy/merge supplementary owner snapshots; fifth latest-value topic |
| app_program/subsystem1/buddy1_bridge.h | Supplementary reporting API and topic count |
| app_program/subsystem1/buddy1_live_status.c (new) | Validate/encode supplementary status |
| app_program/subsystem1/buddy1_live_status.h (new) | Plain C reporting structs and size |
| app_program/subsystem1/CMakeLists.txt | Link new portable codec |
| app_program/subsystem1/README.md | Current communication/setup description |
| app_program/subsystem2/subsystem2_motion.c | Copy three active encoder settings inside existing telemetry getter |
| app_program/subsystem2/subsystem2_motion.h | Add those three telemetry fields |
| app_program/subsystem3/subsystem3.c | Observe existing calibration branches; publish result in existing reporting block |
| app_program/subsystem5/subsystem5.c | Copy current bypass progress/elapsed time in existing reporting block |
| app_program/vehicle_controller/car_run_target.c | Copy owner/controller values to supplementary telemetry |
| lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c | Publish fifth topic using existing network-service owner |

Dashboard files, protocol documentation, golden fixtures and host tests were also
updated. No motor executor/driver, motion profile, steering algorithm, avoidance
algorithm, pin map, timer limit or task-priority file was modified. This integration is intended for the demo branch; generated builds and private
WiFi credentials are not committed.
