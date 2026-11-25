# Group 18 autonomous rover

The Group 18 rover is designed to navigate a marked course autonomously. It follows a line, uses Code 39 barcodes to choose turns, and goes around obstacles before returning to the line. An operator can start or stop a run, assign barcode routes, and see the rover's status in the browser dashboard. The combined firmware runs on a Raspberry Pi Pico W with µT-Kernel.

## How the Vehicle Controller ties it together

The Vehicle Controller manages the run and decides which maneuver has permission to move the rover. Buddy 2 executes authorized wheel commands using encoder feedback and reports wheel speed and motion results. Operator STOP and pickup evidence feed the controller's stop handling.

1. **Follow the line.** Buddy 3 reads the line sensors and supplies steering and junction observations. The controller converts steering into wheel targets for Buddy 2. The demo profile sets cruising speed to **80 mm/s**.
2. **Take a barcode-directed turn.** Buddy 3 decodes the letter and resolves its assigned direction. The controller applies that instruction once at the next eligible junction, then hands control back to line following.
3. **Bypass an obstacle.** Buddy 5 uses a servo-mounted ultrasonic sensor to scan for clearance, proposes a bypass, and requests movement through the controller. Buddy 2 carries out the authorized moves. Buddy 5 uses line-sensor feedback to rejoin, with a search that can extend to 90° on each side in the current profile.
4. **Report terrain and stop on pickup.** Buddy 4 uses the IMU and wheel odometry to estimate hump height. A detected pickup signals the controller to stop. Buddy 1 sends rover status and diagnostic logs over MQTT and relays operator commands from the dashboard.

[Combined startup and subsystem bindings](app_program/vehicle_controller/car_run_target.c) · [Controller service](app_program/vehicle_controller/controller_service.c)

## Recorded results

Earlier on-rover logs recorded successful barcode decodes and turns, an obstacle bypass with a return to line tracking, and pickup-triggered stops. Those runs also produced live MQTT telemetry and downloaded logs with the rover operating wirelessly. [Recorded rover observations and firmware versions](testing/DEMO_EVIDENCE.md)

The **25 November 2026 compact-data build** uses **110,184 B of static RAM**, down from **113,400 B** in the matched next-junction build. Flash load increased from **466,132 B** to **467,920 B**. The binary MQTT heartbeat is **12 B** and a barcode-map reply is **26 B**. The dashboard's Recorded static RAM figure comes from the linked firmware's `.data + .bss` sections. [Build and payload measurements](docs/protocol/STATIC_RAM_COMPARISON.md) · [Field ranges and bit widths](docs/protocol/DATA_REPRESENTATION.md)

## Run the dashboard on Windows

1. From PowerShell in the repository root, start one Mosquitto instance and leave it open.

   ```powershell
   & 'C:\Program Files\mosquitto\mosquitto.exe' -c dashboard/mosquitto-local.conf -v
   ```

2. Run [Start-Website.cmd](dashboard/Start-Website.cmd) with Python installed. Open [the dashboard on port 8080](http://127.0.0.1:8080). Use **Explore preview** to view simulated readings without a Pico.
3. For live data, exit preview and select **Connect broker**. Use `ws://localhost:9001` and topic prefix `group18/robot1`. Wait for a fresh Pico heartbeat before issuing commands. The Pico connects to the laptop's Wi-Fi IPv4 address on MQTT port **1883**.
4. To save evidence, use **Download log** in the **Pico diagnostic log** panel. With the Pico on battery and connected to Wi-Fi, the log can continue with USB unplugged.

The broker configuration is for a trusted local network. If a broker is already serving these ports, use that instance. See the [Buddy 1 setup and troubleshooting guide](BUDDY1_README.md).

## Build and flash the Pico W

Use GNU Make, the Arm GNU toolchain and Pico SDK 2.3.1. Set `PICO_SDK_PATH` and create a local `config/wifi_secrets.h` defining `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP`, `MQTT_BROKER_PORT` and `MQTT_CLIENT_ID`. Use the laptop's Wi-Fi IPv4 address as the broker IP and `1883` as the port. Keep the header and UF2s containing credentials private.

Run the [recorded combined Wi-Fi build command](docs/protocol/COMPACT_DATA_PICO_TEST.md#build-the-wi-fi-image) from `build_make/`. Copy its UF2 to `RPI-RP2` while the Pico is in BOOTSEL. Check for firmware ID `group18-combined-compact-data-2026-11-25` in the dashboard log.

## Subsystems and ownership

| Buddy | Documentation |
|---|---|
| 1 | [Communications](app_program/subsystem1/README.md) |
| 2 | [Motion library, PID tuning and accuracy](app_program/subsystem2/README.md) |
| 3 | [Line and barcode](app_program/subsystem3/README.md) |
| 4 | [Terrain and pickup](app_program/subsystem4/README.md) |
| 5 | [Obstacle avoidance](app_program/subsystem5/README.md) |

[Team directory](TEAM.md) · [Firmware pin map](app_program/project_config/pinmap.h)

## Repository conventions

Keep drivers with the owning Buddy. The course Make rules discover C files in `app_program/` and one directory level below it. Put vehicle pins and tuning under `app_program/project_config/`. `usermain()` is the application entry point, and CYW43/lwIP access stays in the network service. Use reviewed branches and involve the affected owners when changing shared contracts, pins, startup or stop behavior.

The [course-template migration](docs/migration/course-template-migration.md) records the original port and path mapping.
