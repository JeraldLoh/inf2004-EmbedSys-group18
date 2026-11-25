# Combined compact-data Pico check

## Build the Wi-Fi image

Use Pico SDK 2.3.1 and set `PICO_SDK_PATH`. Configure Wi-Fi and MQTT in the local `config/wifi_secrets.h` using the repository's example. The locally packaged demo UF2 uses lab-specific Wi-Fi settings. Build your own image for another network.

From `build_make/`, run the recorded profile:

```sh
make all -j6 SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=lwip MQTT=1 CAR_RUN=1 CAR_MQTT=1 CAR_B2_TRIAL=1 CRASH_REPORT=0 TEST=0 CRUISE_MM_S=80 LINE_DAMP_STEER=40 LINE_DAMP_MS=40 LINE_LOST_MS=0 LINE_BACKTRACK_MM=0 BARCODE_RETRY=1 BARCODE_RETRACE_MARGIN_MM=25 BARCODE_REREAD_MM_S=80 WATCHDOG_RETRY=1 S3_GROUND_LOG=1 WIFI_DIAG=0 JUNCTION_PIVOT_APPROACH_PERCENT=110 JUNCTION_PIVOT_TRAVEL_PERCENT=90 JUNCTION_UTURN_TRAVEL_PERCENT=105
```

The build outputs `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_carrun_b1live.uf2`. The test steps below use the packaged name for the same combined profile.

## Run the hardware comparison

Use the same track, lighting and obstacle placements for the currently flashed firmware and the new image. Keep the baseline wireless export so failures can be compared across images.

1. With the 180° firmware still on the Pico, open PowerShell in the integration repository. Start the dashboard with `python -B -m http.server 8080 --bind 127.0.0.1 --directory dashboard/web`. If the demo broker is stopped, run `& 'C:\Program Files\mosquitto\mosquitto.exe' -c dashboard/mosquitto-local.conf -v` in a second PowerShell window. A running demo broker already owns those ports.
2. Open the local dashboard on port 8080, refresh the page and connect to `ws://localhost:9001` under `group18/robot1`. Confirm a fresh heartbeat, the existing Rover overview and barcode panels, and the Recorded static RAM values of 113,400 B and 110,184 B. Those figures come from the linked comparison builds. Save a baseline run and export its wireless log.
3. Enter BOOTSEL and copy `group18_demo_combined_80_mqtt_compact_data_2026-11-25.uf2` onto `RPI-RP2`. Wait for a fresh heartbeat. Check for firmware ID `group18-combined-compact-data-2026-11-25` in the wireless log. Run on battery and unplug USB. Confirm the log still updates and can be exported.
4. With the rover stationary, send a known barcode assignment and read it back. Confirm the displayed assignment matches. Run the same straight and bends, then test a known barcode with a delayed junction. It should execute one turn at the next eligible junction. Repeat with a different assignment if you want to verify map replacement.
5. Test the same obstacle on each side, including the curved rejoin if available. Record scan direction, width or unavailable, chosen side, and whether line tracking resumes. An extended search can use up to 90° on either side of the centre heading. Record the printed sweep targets if that path occurs.
6. Run the usual hump and pickup checks. During pickup, the motors should stay stopped until an intentional restart after safe conditions return. Check dashboard STOP and physical STOP in separate runs. Try reconnecting Wi-Fi while stationary and check that the dashboard returns to live.
7. Export the wireless log. Record firmware ID, scenario, passes and failures, and note the timestamp for any unusual motion or stale panel. A wireless log can establish reported outcomes. Deadline and stack high-water measurements need a separately instrumented paired Pico test.
