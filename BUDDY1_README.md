# Buddy 1 - WiFi/MQTT and live dashboard

Build and run guide for the combined team application, checked against main
commit `806faea` on 25 November 2026. Buddy 1 connects reporting from Buddy 2,
3, 4 and 5 to the dashboard and forwards supported commands. The onboard
controller decides whether movement is allowed; Buddy 2 owns motor outputs.

## Normal startup (firmware already flashed)

1. Connect the laptop to the 2.4 GHz WiFi/hotspot configured in the Pico.
2. Run `ipconfig`. The laptop's WiFi IPv4 must match the broker IP compiled into
   the Pico. A changed WiFi password or broker IP needs a rebuild and reflash.
3. Open Command Prompt and start one broker:

   ```cmd
   cd /d C:\Users\ftna\Desktop\emb\inf2004-EmbedSys-group18
   "C:\Program Files\mosquitto\mosquitto.exe" -c dashboard/mosquitto-local.conf -v
   ```

   In PowerShell, put `&` before the quoted executable path. Keep this window
   open. Avoid running a second Mosquitto service/broker at the same time.
4. Open another Command Prompt:

   ```cmd
   cd /d C:\Users\ftna\Desktop\emb\inf2004-EmbedSys-group18
   dashboard\Start-Website.cmd
   ```

5. Open `http://127.0.0.1:8080/`, or the port printed by the launcher. Keep the
   website terminal open. Exit Preview and connect to `ws://localhost:9001`
   with topic prefix `group18/robot1`.
6. Power the flashed robot normally, without BOOTSEL. Wait for fresh rover
   heartbeat and readings. Keep wheels raised for the initial command check
   and keep the physical STOP accessible.
7. Start requests autonomous operation; Stop requests stopping. A `queued`
   result confirms receipt/queuing, not physical execution. Check reported
   controller state and actual movement.

Website HTTP 8080, MQTT TCP 1883 and browser MQTT WebSocket 9001 serve different
purposes. Browser-to-broker connection alone does not prove the Pico is online.
Preview uses sample data. Ctrl+F5 reloads dashboard changes without reflashing.

## Private configuration

Use ignored `config/wifi_secrets.h`; copy the example header only if it is
missing. Configure `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP` (laptop LAN
IPv4, not localhost), `MQTT_BROKER_PORT=1883` and a unique `MQTT_CLIENT_ID`.
Do not commit credentials or share a UF2 containing private credentials.

## Build one combined WiFi UF2

This setup uses WSL Ubuntu, Make, Arm GCC and Pico SDK 2.3.0 with its required
submodules. Python runs the website; Node is needed for dashboard tests.

Open Ubuntu and run the following documented baseline profile. Adjust paths
for another laptop:

```bash
export PICO_SDK_PATH=/mnt/c/Users/ftna/.pico-sdk/sdk/2.3.0
cd /mnt/c/Users/ftna/Desktop/emb/inf2004-EmbedSys-group18/build_make
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=cyw43 NET=lwip MQTT=1 TEST=0 B1_DEMO_RUN=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=1 CAR_MQTT=1 CAR_B2_TRIAL=1 CIRCUIT_TEST=0 B2_MOVE_TEST=0 OBSTACLE_TABLE=0 LINE_PID=0 NO_BARCODE_CAL=0 CRUISE_MM_S=80 LINE_DAMP_STEER=40 LINE_DAMP_MS=40 LINE_LOST_MS=0 LINE_BACKTRACK_MM=0 BARCODE_RETRY=0 WATCHDOG_RETRY=1 WATCHDOG_TEST=0 OFF_GROUND=0 S3_GROUND_LOG=1 LIFT_MM=200 LIFT_WINDOW_MS=1200 LIFT_JOLT_PAUSE=0
```

Wait for successful linking and UF2 generation. Flash this output:

```text
C:\Users\ftna\Desktop\emb\inf2004-EmbedSys-group18\build_make\mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2
```

This is the team's documented baseline floor-test candidate, not a guarantee
of completed car validation. It uses 80 mm/s cruise, digital line steering and
damping 40. Automatic line backtrack/search, barcode retry, analogue LINE_PID
and optical off-ground detection are disabled in this profile. Existing IMU
lift/overturn handling remains. Both-white can mean the sensors straddle the
line, so that reading alone does not establish line loss.

[Team build profiles](docs/DEMO_BUILD.md) also describe the combined barcode-retry/
U-turn profile and an optional recovery test profile. These are different
configurations, not interchangeable test results. All produce the same UF2
filename: check the build command and serial `[CAR DEMO]` banner to identify
which settings were used. Do not copy another laptop's IP/path from that guide.
`CAR_B2_TRIAL=0` selects the old unqualified diagnostic profile.

The watchdog is retained. With `WATCHDOG_RETRY=1`, a fresh physical GP20 gesture
may retry a watchdog cutoff; MQTT START cannot clear that cutoff. Ordinary
motion faults remain latched. Optical pickup protection requires measured ADC
thresholds and polarity characterization before enabling `OFF_GROUND`.

## Flash and restart

Stop Serial Monitoring. Unplug USB, hold BOOTSEL while reconnecting, then release
when `RPI-RP2` appears. Copy the newly built `.uf2` onto that drive. The Pico
restarts automatically. Do not copy the `.elf` for BOOTSEL flashing.

One combined UF2 contains the team application and WiFi. Flashing another buddy's
standalone UF2 overwrites it. Normal power-on needs neither BOOTSEL nor rebuilding.
USB Serial Monitor at 115200 can show startup configuration and fault details.
A successful build does not prove physical line following, stopping or sensing.

## Dashboard readings and barcode mapping

- **Buddy 2:** wheel speeds, commanded targets, controller/motion state and
  stop/fault information. Encoder counts and wheel dimensions are configured
  settings, not evidence that a live calibration procedure passed.
- **Buddy 3:** line observations, steering, barcode results and available
  acquisition/calibration status. The default ADC barcode path uses fixed
  thresholds; do not describe configured thresholds as a measured calibration
  pass. Search/recovery labels follow the reported controller state and the
  selected build profile.
- **Buddy 4:** IMU/terrain, hump and run-end information supplied by its producer.
  Invalid height estimates are not valid zero-height results. The browser hump
  history is a received-session history, not a guaranteed complete device log.
- **Buddy 5:** front/latest range status, scan direction, avoidance state,
  available obstacle width/measurement status, chosen direction and bypass/
  rejoin progress. Missing echoes do not prove a clear path. Width is an estimate
  with validity limits, not a precision measurement. Servo angle is commanded
  position, not measured servo feedback. No sonar/servo calibration pass is
  invented; the card shows available operational outputs instead.

Use the barcode map controls to read the rover's current map, edit assignments
and send them. Check the rover's returned map/revision; an edited local draft
is not confirmation that the car applied it. Confirm the map again after reboot.
The mapping selects the action at a junction; decoding a barcode is not proof
that the physical junction maneuver executed.

The Activity stream records observed commands/results, connection changes,
controller transitions, faults and scan/search states, with duplicate suppression.
The Pico diagnostic log provides wireless log viewing/export where available.
Download the log for investigation, but neither browser history nor QoS 0
transport guarantees every event was received. Reporting must not be used as a
substitute for local STOP behavior.

## MQTT and freshness

Default prefix: `group18/robot1`. Publications use QoS 0 and are non-retained.
Only the kernel WiFi service calls raw network APIs. Reporting copies producer
snapshots; dashboard code does not directly drive PWM or GPIO.

| Topic suffix | Content | Nominal rate |
| --- | --- | --- |
| `heartbeat` | JSON heartbeat | Every 5 s |
| `telemetry` | Binary telemetry v3 | Every 200 ms |
| `live_status` | Binary type 4: v4 is 96 bytes; v5 is 100 bytes with map revision | Every 200 ms |
| `diagnostics` | Binary validation/rejection information | Every 1 s |
| `command` | Binary START/STOP, MAP_SET and MAP_SHOW | Operator request |
| `command_result` | JSON acceptance/rejection or map result | On result |
| `log` | Diagnostic log batches | When available |

START/STOP and MAP_SHOW commands are 8 bytes; MAP_SET is 22 bytes. Do not publish
plain ASCII START to the binary command topic. See the
[protocol guide](docs/protocol/CAR_MQTT_HOOKUP.md) for encoding details.

No heartbeat for 15 seconds marks communication unavailable and disables remote
commands. Supplementary sections have a 1500 ms expiry. Firmware also rejects
unchanging IMU sequences after 200 ms rather than repeatedly refreshing old B4
data. Packet reception age and actual measurement age are different: a fresh
heartbeat does not prove every sensor is fresh or valid.

Outgoing latest-value slots can overwrite unsent reports. A guaranteed essential-
event FIFO with replay protection is not established by these dashboard features.
Binary telemetry does not mean every internal firmware field uses its minimum
possible bit width; heartbeat and command-result messages still use JSON.

## Diagnostics and verification

Telemetry payload bytes describe a packet, not RAM consumption. The recorded
static-RAM comparison on the website is historical, not a live measurement of
the current firmware. To report current static RAM, measure `.data + .bss` from
the newly linked ELF and record its commit/profile. Runtime stack/heap usage is
separate. Do not compare unmatched build profiles as proof of memory savings.

For waiting/stopped behavior, check WiFi, the compiled broker IP, a single broker,
heartbeat freshness and controller stop reasons. Use serial logs before changing
motor settings or disabling protective checks. GP21 is the local physical STOP.

Optional PC checks from the repository root:

```bash
cmake -S testing/host -B build/demo-host
cmake --build build/demo-host -j6
ctest --test-dir build/demo-host --output-on-failure
python3 testing/host/verify_buddy1_car.py
```

```text
node --test dashboard/web/*.test.mjs dashboard/web/test.mjs
```

These commands are checks to run, not a claim that this README update reran them.
See [recorded validation and remaining checks](docs/DEMO_VALIDATION.md). Compilation
and PC tests do not establish sensor accuracy, physical pickup detection, STOP
latency, barcode turns or complete obstacle bypass/rejoin. Test the selected
combined profile on the car and keep baseline/recovery results separate.

This guide update changes documentation only. It does not rebuild, flash or push.
