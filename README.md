# Group 18 autonomous rover

The combined firmware runs on a Raspberry Pi Pico W with µT-Kernel. It follows a line, reads Code 39 barcodes to select the next junction maneuver, scans and bypasses obstacles, and reports terrain and pickup events. The browser dashboard shows rover status, lets an operator start or stop a run and update barcode routes, and exports wireless diagnostic logs.

This branch contains the **25 November 2026 compact-data build**. Its demo profile uses an 80 mm/s cruise, a one-use barcode route at the next eligible junction, and an obstacle rejoin search that can extend to 90° on each side. The profile includes pickup stop and keeps ordinary line-loss reversing disabled. See the [build and Pico test guide](docs/protocol/COMPACT_DATA_PICO_TEST.md).

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

Run the [recorded combined Wi-Fi build command](docs/protocol/COMPACT_DATA_PICO_TEST.md#build-the-wi-fi-image) from `build_make/`. Copy its UF2 to `RPI-RP2` while the Pico is in BOOTSEL. Check for firmware ID `group18-combined-compact-data-2026-11-25` in the dashboard log. The guide lists hardware checks for tracking, junction routing, obstacle rejoin, pickup and STOP.

## Subsystems and ownership

| Buddy | Role | Source and setup | Test evidence |
|---|---|---|---|
| 1 | Wi-Fi, MQTT, telemetry and remote commands | [Communications](app_program/subsystem1/README.md) | [Buddy 1 tests](testing/subsystem1/TEST_LOG.md) |
| 2 | Motors, encoders, wheel-speed control and odometry | [Motion library, PID tuning and accuracy](app_program/subsystem2/README.md) | [Buddy 2 tests](testing/subsystem2/TEST_LOG.md) |
| 3 | Line tracking, Code 39 decoding and junction perception | [Line and barcode](app_program/subsystem3/README.md) | [Buddy 3 tests](testing/subsystem3/TEST_LOG.md) |
| 4 | IMU, terrain, hump estimation and pickup detection | [Terrain and pickup](app_program/subsystem4/README.md) | [Buddy 4 tests](testing/subsystem4/TEST_LOG.md) |
| 5 | Sonar scanning, obstacle profiling, bypass and line rejoin | [Obstacle avoidance](app_program/subsystem5/README.md) | [Buddy 5 tests](testing/subsystem5/TEST_LOG.md) |

The [Vehicle Controller](app_program/vehicle_controller/README.md) handles run state, motion authority and stops. Buddy 2 owns normal motor output. Sensors publish observations and Buddy 5 requests obstacle maneuvers through the controller. See the [team directory](TEAM.md) and [firmware pin map](app_program/project_config/pinmap.h).

## Measurements and demo evidence

| Recorded build measurement | Matched next-junction baseline | Compact-data build |
|---|---:|---:|
| Static RAM, `.data + .bss` | 113,400 B | **110,184 B** |
| Flash load | 466,132 B | 467,920 B |
| MQTT heartbeat payload | Up to 65 B | 12 B |
| MQTT result with barcode map | Up to 131 B | 26 B |

The compact-data source passed **27 firmware host tests, 70 browser tests and 10 Python dashboard tests**. The firmware linked successfully. The new UF2's physical regression, task timing under Wi-Fi load, and runtime stack and heap peaks remain unverified. The dashboard's Recorded static RAM card shows the linked build values. See the [matched measurements](docs/protocol/STATIC_RAM_COMPARISON.md) and [field ranges and bit widths](docs/protocol/DATA_REPRESENTATION.md).

- [Demo requirement and test-case matrix](testing/DEMO_TRACEABILITY.md)
- [Evidence ledger](testing/DEMO_EVIDENCE.md) and [whole-rover scorecard](testing/system/REHEARSAL_SCORECARD.md)
- [Demo video index](videos/README.md)

## Repository conventions

Keep drivers and tests with the owning Buddy. The course Make rules discover C files in `app_program/` and one directory level below it. Put vehicle pins and tuning under `app_program/project_config/`. `usermain()` is the application entry point, and CYW43/lwIP access stays in the network service. Use reviewed branches and involve the affected owners when changing shared contracts, pins, startup or stop behavior.

The [course-template migration](docs/migration/course-template-migration.md) records the original port and path mapping.
