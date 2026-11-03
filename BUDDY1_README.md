# Buddy 1 — WiFi, MQTT and dashboard

Buddy 1 connects the Pico W robot to a laptop dashboard. It sends telemetry,
receives START/STOP requests and forwards them to the shared controller. The
controller and other buddies own driving, speed, sensing and obstacle avoidance.

This README covers normal startup, configuration, building, flashing and checks.
It describes the current `testing` integration, including local Buddy 1 reporting
changes. It replaces the older text-only, simulated-speed prototype instructions.

## What runs on the robot

One combined UF2 contains the robot application and WiFi/MQTT. Do not flash a
separate image for each buddy: flashing another UF2 replaces the running image.

- Buddy 2: wheel feedback and commanded left/right speed targets.
- Buddy 3: line and barcode readings.
- Buddy 5: ultrasonic readings, avoidance state and bypass progress.
- Shared controller: operating mode, stop reasons, faults and line-search status.
- Buddy 1: protected data copies, binary encoding, command validation and dashboard.

This local integration is based on team revision `61f8836`. Buddy 4 integration
and the fast S-curve bypass are absent. Buddy 5 uses the seven-leg bypass. The optional hump-pause status is not
reported by this integration. Missing fields stay unavailable rather than being
invented. Preview is generated sample data, not a connection to the car.

## Normal startup — no rebuild needed

Use these steps when the correct firmware is already flashed and its network
settings have not changed.

1. Connect the laptop to the same 2.4 GHz WiFi/hotspot configured in the Pico.
2. Check the laptop WiFi IPv4 address with `ipconfig`. It must match the broker
   address compiled into the firmware. Do not use the WSL/VPN adapter address.
3. Run one Mosquitto broker. In **Command Prompt**, from the repository root:

   ```cmd
   "C:\Program Files\mosquitto\mosquitto.exe" -c dashboard/mosquitto-local.conf -v
   ```

   In **PowerShell**, the same executable needs the leading `&`:

   ```powershell
   & "C:\Program Files\mosquitto\mosquitto.exe" -c dashboard/mosquitto-local.conf -v
   ```

4. Power the robot normally, without BOOTSEL. For the first motion check, keep
   the wheels raised and the physical STOP accessible.
5. Double-click `dashboard/Start-Website.cmd` in File Explorer. Keep that window
   and the broker window open. The current launcher uses port **8080**.
6. Open `http://127.0.0.1:8080/`, exit preview and connect the dashboard:

   ```text
   WebSocket URL: ws://localhost:9001
   Topic prefix:  group18/robot1
   ```

7. Wait for a fresh heartbeat and live readings. Use **Start rover** to request
   autonomous operation and **Stop** to request stopping. Driving speed is set
   onboard; there is no dashboard speed slider.

A queued reply confirms that a request entered the command queue, not that the
controller accepted it or the wheels moved. Remote START does not clear physical
or hardware/safety stops. A network STOP is not a guaranteed emergency stop.

A previously started website server on 8082 keeps using 8082 until closed; the
launcher now starts a server on 8080. This website port is separate from MQTT
1883 and WebSocket 9001. Refresh with Ctrl+F5 after dashboard-only edits.

## Configure WiFi and MQTT

Private settings live in ignored `config/wifi_secrets.h`. If it does not exist,
copy `config/wifi_secrets.example.h` and fill in the local values.

| Setting | Value to supply |
| --- | --- |
| `WIFI_SSID` | Exact 2.4 GHz WiFi/hotspot name |
| `WIFI_PASSWORD` | Its password |
| `MQTT_BROKER_IP` | Laptop WiFi IPv4 address, not localhost or the Pico address |
| `MQTT_BROKER_PORT` | `1883` |
| `MQTT_CLIENT_ID` | Unique client ID for this Pico |

Changing compiled WiFi/broker settings requires rebuilding and flashing. The Pico
does not automatically switch to whichever WiFi the laptop chooses. Do not commit
the private header or share firmware containing real credentials.

## Build the combined robot firmware

The team update at `61f8836` makes the v6 profile the official CAR_RUN default.
The historically named `CAR_B2_TRIAL` flag now defaults to 1 for CAR_RUN, and
Buddy 2 marks that car profile qualified. WiFi builds use radio priority 13.
Explicit `CAR_B2_TRIAL=0` still selects the unqualified zero-gain diagnostic
profile. Encoder Configured remains separate from motion qualification.

Prerequisites: WSL Ubuntu, Make, Arm cross compiler and Pico SDK with its required
submodules. The current local setup uses Pico SDK 2.3.0. Run this in **Ubuntu**,
adjusting paths for another laptop:

```bash
export PICO_SDK_PATH=/mnt/c/Users/ftna/.pico-sdk/sdk/2.3.0
cd /mnt/c/Users/ftna/Desktop/emb/inf2004-EmbedSys-group18/build_make
make -j6 CAR_RUN=1 CAR_MQTT=1 WIFI=cyw43 NET=lwip MQTT=1 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 TEST=0
```

Wait for a successful link and UF2 generation. The combined WiFi image is:

```text
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2
```

The filename stays the same after rebuilding; check the build succeeded and the
file timestamp. The repository also supports a standalone Buddy 1 demo profile;
its simulated image is not the combined car firmware above. Follow the current
shared pin configuration; USB console avoids taking sensor pins for a UART console.

## Flash only when firmware or compiled settings change

1. Stop Serial Monitoring and unplug the Pico USB cable.
2. Hold BOOTSEL while plugging USB in; release when **RPI-RP2** appears.
3. Copy the combined `_carrun_b1live.uf2` file onto that drive.
4. The drive disappears as the Pico restarts and attempts WiFi, then MQTT.
5. Check the dashboard heartbeat and live readings before testing movement.

An ordinary power-on does not need BOOTSEL, a build or another flash. Dashboard
HTML/JavaScript changes only need a browser refresh; they are not stored in the UF2.

## What Buddy 1 sends and receives

Topics use the `group18/robot1` prefix:

| Suffix | Format / purpose | Nominal interval |
| --- | --- | --- |
| `/heartbeat` | JSON connection heartbeat | 5 seconds |
| `/telemetry` | Binary v3 combined readings; web decoder also accepts v2 | 200 ms when data is available |
| `/live_status` | 76-byte supplementary binary v2 (web also accepts v1): targets, searches, bypass progress | 200 ms when data is available |
| `/diagnostics` | Binary validation and rejection counters | 1 second |
| `/command` | Binary v2 START/STOP with command ID | On request |
| `/command_result` | JSON queued/rejected/queue-full or simulated response | On request |

The dashboard sends START/STOP only, not SET_SPEED. Each command is an explicit
8-byte binary packet; use the dashboard rather than a plain-text MQTT command.
All publishes use QoS 0 and are non-retained. Do not publish retained commands.

Only the kernel WiFi service calls raw CYW43/lwIP/MQTT APIs. Buddy tasks copy
readings to Buddy 1 through protected APIs; bounded buffers transfer messages to
and from the radio service. Outgoing slots keep latest values rather than a
history. Commands expire after one second and older connection epochs are ignored.

Sensor sections carry timestamps. Supplementary status expires after 1500 ms;
its arrival does not refresh the heartbeat. A live connection does not prove every
sensor is fresh or calibrated. The wheel panel shows Configured only from fresh firmware-reported encoder
counts/revolution and wheel diameter, not a live pass/fail test.
Qualification is separate from wheel calibration;
measured B2 wheel speeds are magnitudes, while commanded targets are signed.

Interfaces: `app_program/subsystem1/buddy1_bridge.h`.
Detailed telemetry and supplementary status: `docs/protocol/CAR_MQTT_HOOKUP.md`
and `docs/protocol/telemetry-fields.json`.

## Troubleshooting

- **Waiting for rover:** check the laptop joined the configured WiFi, the compiled
  broker IP matches its current WiFi IP, and the broker is running. Browser
  connected means the browser reached the broker; it does not prove the Pico did.
- **Website refused connection:** start the website server and use its displayed
  port. It is a different service from the MQTT broker.
- **No Pico messages despite a connected subscriber:** check for two brokers.
  A loopback-only Windows service and a manual LAN broker can split clients.
  Use `netstat -ano | findstr :1883` and identify the processes before stopping any.
- **START queued but no movement:** inspect controller mode, stop reasons, hardware
  faults and qualification under Motion sensor details. Also check physical STOP
  and motor power. Do not assume this needs reflashing.
- **Stale readings:** inspect the individual sample ages. A current heartbeat alone
  does not make old sensor data current.
- **Serial diagnostics:** open the Pico USB COM port at 115200 baud. Useful markers
  include `wifi: link up`, `buddy1: MQTT ready` and periodic `[HB]` logs. Reopening
  the serial monitor does not require another flash.

## Checks and evidence limits

From the repository root, run the C checks in Ubuntu:

```bash
cmake -S testing/host -B build/host-live-status
cmake --build build/host-live-status -j6
ctest --test-dir build/host-live-status --output-on-failure
python3 testing/host/verify_buddy1_car.py
```

For dashboard checks, with Node installed:

```text
node --test dashboard/web/*.test.mjs dashboard/web/test.mjs
```

The local combined build, PC suites and an isolated MQTT roundtrip have passed.
The user also observed real MQTT telemetry and dashboard START causing movement.
That is not verification of every buddy, route, calibration or STOP timing on the
car. Preview and PC tests do not establish hardware behaviour.

## Shut down

Stop the rover, stop Serial Monitoring, close the website server and manual broker
with Ctrl+C, then remove robot power as appropriate. Keep local credentials private.

## Diagnosing a sudden stop

The dashboard Activity stream records observed controller-mode, stop-mask,
hardware-fault, mission and avoidance changes with rover uptime. These are
snapshot observations: events between messages or during disconnection can be missed.
For exact executor detail, open USB Serial Monitor at 115200 before starting.
Keep the output after the stop and do not restart until captured. Useful markers:
`[CAR STATE]`, `[CAR REASON]`, `[CAR STOP]`, `[CAR START]`, and `[B2 STOP DETAIL]`.
A motor detail includes reason (for example NO_WHEEL_PROGRESS or COMMAND_EXPIRED),
wheel mask (1 left, 2 right, 3 both), command/generation and pre-stop targets/speeds.
Its at_ms is the logger observation time, not an exact ISR event timestamp.
Logs add diagnosis only; they do not relax timeouts, qualification or stop logic.

Known open issue: an on-car serial capture recorded cutoff=1 at uptime 62832 ms,
followed by DRIVER_INHIBIT_OR_STOP. The watchdog trigger is confirmed; the cause
of the missed refresh is not yet established. Timing diagnostics are investigative,
not a confirmed fix. The motor watchdog remains enabled.
