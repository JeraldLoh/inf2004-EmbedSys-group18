# ROVER / 18 website dashboard

## Open the website

Double-click dashboard/Start-Website.cmd, keep its window open, and open
http://localhost:8080 in Chrome or Edge. Or from the project root run:

    python -B -m http.server 8080 --bind 127.0.0.1 --directory dashboard/web

Click Explore preview to see generated readings and test the controls locally.
Preview mode disconnects MQTT and never publishes any command. It is clearly
labelled SIMULATED. Exit preview before connecting to your Pico.

## Connect Mosquitto

The broker config now has TCP 1883 for the Pico and WebSocket 9001 for the browser.
Your existing running broker must be restarted to read the added listener:

1. In the old Mosquitto Command Prompt press Ctrl+C.
2. Run this command, and leave that window open:

    "C:\Program Files\mosquitto\mosquitto.exe" -c "C:\Users\ftna\Desktop\emb\inf2004-EmbedSys-group18\dashboard\mosquitto-local.conf" -v

Expect messages opening ports 1883 and 9001. If 1883 is already in use, stop
only your old broker before restarting; do not start two copies on the same port.
The website's Connect broker button opens settings: ws://localhost:9001,
topic group18/robot1. No internet connection or npm install is needed.
The broker allows anonymous clients on a trusted local LAN. Do not expose it
publicly. The WebSocket listener is bound to this laptop's loopback interface.

## Pico setup stays the same

uTKernel firmware and TCP MQTT port 1883 are unchanged. The Pico's broker IP
is the laptop's WiFi IPv4 address (you reported 192.168.1.113; recheck ipconfig
if your WiFi network changes). That address must be entered in the private
config/wifi_secrets.h before rebuilding/flashing. Real credentials have not
been added by this website work. A website alone does not configure the Pico.

A browser connected to the broker still waits for the Pico heartbeat. Commands
unlock only after a non-retained heartbeat, and lock when it reaches 15 sec old.
A firmware demo is labelled FIRMWARE / SIMULATED, distinct from browser preview.
Speed is percentage, not cm/s; distance is the agreed telemetry field, not a map
position. The rover drawing is an illustration. No fake mission completion,
battery, latency, RSSI or other unmeasured sensor values are shown.

MQTT 3.1.1 clean sessions, QoS 0, non-retained commands. No offline command queue.
Results say simulated/queued/rejected/queue_full, never physical execution.
A retained /command on the broker must still be cleared as described in
BUDDY1_README.md because the firmware receive API cannot inspect its retained bit.

## Verification

Node is needed only to run the optional automated tests:

    node --test dashboard/web/test.mjs

The website consists of local HTML/CSS/JavaScript; it uses native WebSocket and
no external fonts, CDNs or packages. The old Python dashboard is retained as an
optional fallback. Do not run both dashboards when issuing robot commands.

Verified 2026-10-15: 6 automated tests passed; real Mosquitto WebSocket
CONNECT/SUBACK/heartbeat/telemetry roundtrip passed on isolated port 19001.
Desktop and 390px mobile layouts checked in browser; preview stop,
mode exit/control locking, and invalid settings validation checked.
Pico end-to-end operation remains untested. User broker was not restarted.
