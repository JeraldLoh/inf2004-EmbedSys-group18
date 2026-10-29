# Buddy 1 Text MQTT Prototype

Buddy 1 provides Pico W WiFi/MQTT communication and a local website dashboard
using micro T-Kernel. The current firmware sends **simulated readings** over a
real WiFi/MQTT connection. START, STOP and SET_SPEED change simulated state;
they do not operate motors. The binary-format requirement is not implemented yet.

## Quick start on a previously configured laptop

1. Connect the laptop to the same trusted 2.4 GHz network as the Pico.
2. Check the laptop's current WiFi IPv4 address. If it changed, update the
   private configuration, rebuild and reflash before proceeding.
3. Run **one** Mosquitto broker using `dashboard/mosquitto-local.conf`.
4. Plug the already-flashed Pico in normally, without BOOTSEL.
5. Run `dashboard/Start-Website.cmd` from Windows File Explorer.
6. Open `http://127.0.0.1:8080/`, exit preview and connect to
   `ws://localhost:9001`, topic prefix `group18/robot1`.

Ubuntu/building/flashing are only needed when code or compiled settings change.
Keep the broker and website-server windows open while using the dashboard.

## First-time prerequisites

- Raspberry Pi **Pico W** (RP2040) and a USB data cable. This build is not for
  Pico 2 or Pico 2 W. Disable/disconnect motors for standalone testing.
- Windows with WSL Ubuntu; Arm cross compiler, Make, host C/C++ compiler and
  Pico SDK with TinyUSB, CYW43 and lwIP submodules present. SDK 2.3.0 and
  Ubuntu Arm GCC 14.2.1 were used for the recorded builds.
- Windows Python and Mosquitto installed. A browser runs the website's
  JavaScript; Java and npm are not required to start it.
- VS Code Serial Monitor or another USB serial terminal, 115200 baud.

On Windows, Ubuntu can be installed from an administrator PowerShell:

```powershell
wsl --install -d Ubuntu
```

Restart Windows if requested, then finish Ubuntu's username/password setup.
In Ubuntu, install the build tools if missing:

```bash
sudo apt update
sudo apt install -y build-essential gcc-arm-none-eabi libnewlib-arm-none-eabi git cmake
arm-none-eabi-gcc --version
```

The Pico SDK is a separate prerequisite; use the course setup instructions in
`docs/template-reference/README.md`. Adjust SDK and project paths below to your
own installation. Do not repeat installation on every startup.

## 1 Configure WiFi and the laptop broker address

From the project root in Ubuntu, create the local configuration **only if it
does not already exist**:

```bash
test -f config/wifi_secrets.h || cp config/wifi_secrets.example.h config/wifi_secrets.h
```

Edit `config/wifi_secrets.h` locally:

| Setting | Meaning |
|---|---|
| `WIFI_SSID` | Exact 2.4 GHz network name, including case and spaces. |
| `WIFI_PASSWORD` | Password for that network. |
| `MQTT_BROKER_IP` | Laptop's current WiFi IPv4 address, not the Pico's IP and not localhost. |
| `MQTT_BROKER_PORT` | `1883`. |
| `MQTT_CLIENT_ID` | A unique ID for this Pico, to avoid session-takeover conflicts. |

Find the laptop address in a new Windows Command Prompt:

```cmd
ipconfig
```

Use **Wireless LAN adapter Wi-Fi -> IPv4 Address**, not a WSL/VPN adapter.
The address can change after changing networks. The Pico connects to the SSID
compiled into its firmware; it does not follow the laptop's chosen network.

`wifi_secrets.h` is ignored by Git. Share the `.example.h`, never the private
header or firmware binaries built with real credentials. Changing any compiled
setting requires rebuilding and reflashing.

## 2 Build the standalone demo

Open Ubuntu. Example paths for the current Windows checkout:

```bash
export PICO_SDK_PATH=/mnt/c/Users/ftna/.pico-sdk/sdk/2.3.0
cd /mnt/c/Users/ftna/Desktop/emb/inf2004-EmbedSys-group18/build_make
make NET=lwip MQTT=1 TEST=1 B1_DEMO_RUN=1 CONSOLE=usb_cdc CRASH_REPORT=0 -j8
```

Wait for a successful build and UF2 generation. Output:

```text
build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_test_b1demo.uf2
```

`B1_DEMO_RUN=1` explicitly starts only Buddy 1's standalone application task.
The default `B1_DEMO_RUN=0` retains the team's inactive application startup;
do not flash its unsuffixed image expecting demo telemetry. Kernel USB/radio
services may still run. Other buddies' tasks and a Vehicle Controller task are
not started. Priority 18 and the 4 KiB stack are provisional for integration.

Use USB console with `CRASH_REPORT=0`: UART/crash mirroring conflicts with the
team's GP0/GP1 line-sensor pins.

## 3 Flash the Pico W

1. Stop Serial Monitoring and unplug USB.
2. Hold BOOTSEL while plugging USB in; release when **RPI-RP2** appears.
3. Copy the newly built `_b1demo.uf2` onto that drive.
4. The drive disappears as the Pico reboots. No second flashing command is needed.

For an ordinary reboot, unplug/replug **without BOOTSEL**. If RPI-RP2 remains,
the Pico is still in bootloader mode rather than running the application.

## 4 Start one Mosquitto broker

If a background **Mosquitto Broker** Windows service is already running, stop
it temporarily in `services.msc` before this manual test. Do not run two brokers
on port 1883. In our setup, a loopback-only service and a manual LAN broker
accepted different clients, so a subscriber connected but received no Pico data.

In a Windows Command Prompt, adjusting paths if needed:

```cmd
"C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Users\ftna\Desktop\emb\inf2004-EmbedSys-group18\dashboard\mosquitto-local.conf" -v
```

Expect ports **1883** (Pico TCP MQTT) and **9001** (browser WebSocket). Leave
this window open. If its title starts with **Select**, press **Esc**: console
text selection can pause processing. Click its title bar to focus it without
selecting console text.

The supplied broker allows anonymous access on a trusted LAN. Allow the exact
Mosquitto executable through Windows Firewall on the matching **Private**
profile when using your own trusted hotspot/home network. Keep the firewall
enabled; do not mark untrusted school/public WiFi as Private. The WebSocket
listener is loopback-only. This is a local test setup, not an internet service.

Clear any old retained command before control testing:

```cmd
"C:\Program Files\mosquitto\mosquitto_pub.exe" -h 127.0.0.1 -t group18/robot1/command -r -n
```

Never publish retained movement commands. The firmware receive callback cannot
inspect the retained flag. The dashboard ignores retained incoming status data.

## 5 Check the Pico serial log

In VS Code Serial Monitor, refresh the port list, select the Pico's USB COM
port and click Start Monitoring at **115200 baud**. COM numbers can change.

Expected messages:

```text
wifi: link up, ip=...
buddy1: MQTT ready
[HB] generated_ms=... publish_attempt_ms=... result=0
```

WiFi link up proves association/IP acquisition; MQTT ready confirms the command
subscription. `[HB] result=0` means local lwIP publish acceptance, not confirmed
broker delivery. Compare received heartbeat timestamps and broker logs too.

WiFi-only testing does not need Mosquitto. MQTT testing does. Check WiFi name,
2.4 GHz availability and range for SSID-not-found errors. An empty monitor after
reboot can mean the early log was missed; reconnect the port and wait for the
periodic heartbeat diagnostics. Do not repeatedly reflash to reopen the monitor.

## 6 Open and connect the website

In Windows File Explorer, double-click `dashboard/Start-Website.cmd`. Clicking
the file in VS Code only opens its source. Alternatively, from the project root
in a Windows terminal:

```cmd
python -B -m http.server 8080 --bind 127.0.0.1 --directory dashboard/web
```

Use only one website server on port 8080. Leave its window open and out of Select
mode. Open **http://127.0.0.1:8080/** in Chrome/Edge; no trailing asterisks.

Exit preview, click Connect broker and enter:

```text
WebSocket URL: ws://localhost:9001
Topic prefix:  group18/robot1
```

The browser on the broker laptop uses localhost; the Pico uses the laptop's LAN
IP. Wait for a fresh heartbeat. **FIRMWARE / SIMULATED** means real Pico transport
with sample firmware readings. **PREVIEW / SIMULATED** is browser-only generated
data with no broker or robot commands. Neither is a real sensor/motor test.

Apply a speed percentage, then Start rover; reported simulated speed should
match. Stop returns it to zero. A command result is not physical execution.

After 15 seconds without heartbeat, the dashboard shows **Communication
Unavailable** and locks controls. It retains last readings, labelled **STALE**
with age and last reception time. Telemetry itself becomes stale after three
seconds without a fresh sample. Heartbeat age is seconds since last heartbeat,
not a sending rate. Broker reconnection waits for a fresh heartbeat. Explicit
operator disconnect or a new settings session resets readings to avoid mixing
data sources. Use Ctrl+F5 after website changes; no Pico reflash is needed.

## 7 Optional independent message checks

Use another Command Prompt, not the running broker/server window. Five messages
are printed before the subscriber exits; the Pico and broker keep running.

```cmd
"C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -p 1883 -t "group18/robot1/telemetry" -v -C 5 -W 15
```

```cmd
"C:\Program Files\mosquitto\mosquitto_sub.exe" -h 127.0.0.1 -p 1883 -t "group18/robot1/heartbeat" -v -C 5 -W 40
```

Compare `speed_percent` / `distance_cm` with the dashboard, allowing for updates.
Heartbeat `uptime_ms` gaps should be roughly 5000 ms. Add `-d` for connection
diagnostics. `-W` is the subscriber's wait limit, not the dashboard timeout.

For a connected subscriber with no messages, check duplicate listeners:

```cmd
netstat -ano | findstr LISTENING | findstr :1883
```

Identify a listed PID using `tasklist /FI "PID eq <PID>"` before stopping anything.
Repeated session-takeover logs can indicate duplicate MQTT client IDs. A local
website timeout needs a website-server/port check, not a Pico reflash.

## Protocol and team interface

| Topic suffix under `group18/robot1` | Payload / interval |
|---|---|
| `/heartbeat` | JSON schema, seq, uptime_ms, demo; nominally every 5 s. |
| `/telemetry` | JSON schema, seq, uptime_ms, demo, speed_percent, distance_cm; every 1 s in this baseline. |
| `/command` | Exact ASCII START, STOP or SET_SPEED followed by one space and 0-100. No JSON/newline. |
| `/command_result` | JSON schema, demo, status: simulated / queued / rejected / queue_full. |

All use QoS 0 and non-retained publishes; no guaranteed delivery or emergency
STOP response. The radio service alone owns raw CYW43/lwIP/MQTT. Bounded bridge
buffers hold about eight incoming and eight controller commands; outgoing data
uses one latest slot per message type, not an unbounded outage backlog. Commands
expire after one second and observed connection changes invalidate older epochs.
Overflow can drop commands/results. WiFi retries ordinary down/failure after
5 s and stalled joining/IP after 30 s; MQTT retry is 3 s with a 15 s attempt
deadline. These policies are not measured worst-case recovery guarantees.

Application interface: `app_program/subsystem1/buddy1_bridge.h`.

- `b1_submit_snapshot(&snapshot)` copies a complete snapshot; success 1, busy or
  invalid 0. Current fields: speed_percent 0-100; distance_cm 0-100000.
- `b1_receive_vehicle(&command)` returns 1 for a fresh current-epoch request,
  otherwise 0. The Vehicle Controller must decide execution and safety.
- Both are task-context APIs, not ISR APIs. Use no raw networking calls from
  other buddies' tasks.

Real integration is still open: `B1_DEMO=1` is the current simulation default.
Changing it to zero alone does not connect the controller. A reviewed producer,
controller adapter, percentage-to-physical-speed policy and runtime startup plan
are needed. The connection epoch is not a vehicle-authority generation.

## Verification and evidence

Recorded observations and screenshots: [testing/subsystem1/TEST_LOG.md](testing/subsystem1/TEST_LOG.md).
Standalone hardware testing demonstrated WiFi/MQTT telemetry, matching dashboard
values, simulated speed 42/STOP zero, heartbeat samples about 5010 ms apart after
the producer-lock fix, loss indication, reboot recovery, and user-reported
hotspot-interruption recovery. Exact outage transition/recovery deadlines and
long-duration reliability are not established by screenshots.

Local checks after the fixes: 10 CTest cases, 8 website tests and 5 Python model
tests passed. Pico demo and inactive application profiles compiled; heartbeat
fix firmware subsequently compiled and was retested on the Pico. Mock kernel
tests do not establish actual RTOS scheduling or integrated control timing.

Run tests from the project root:

```bash
cmake -S testing/host -B testing/host/build-b1-check
cmake --build testing/host/build-b1-check
ctest --test-dir testing/host/build-b1-check --output-on-failure
python -B -m unittest discover -s dashboard -p test_model.py
node --test dashboard/web/test.mjs
```

Node is only required for optional website tests. Real sensors/motors and A1
combined-task timing remain unverified. Binary payloads, 200 ms combined
telemetry, the common fault mask and out-of-range diagnostic events are later
work, not features of this text baseline.

## Shut down

Stop Serial Monitoring, press Ctrl+C in the website-server and manual broker
windows, then unplug the Pico. Rebuilding, BOOTSEL and Ubuntu setup are not
required for every normal startup.
