# Buddy 1 — live WiFi/MQTT reporting

See [setup, build and runtime guide](../../BUDDY1_README.md).

Buddy 1 validates/copies owner-produced data, encodes binary telemetry and receives
START/STOP requests. The shared controller decides execution and safety. Motor
settings, line/avoidance algorithms, watchdog, pins and priorities are unchanged.
Only the kernel WiFi service calls raw CYW43/lwIP/MQTT. Nonblocking protected copies
and bounded buffers connect it with subsystem tasks. Current supplementary status
is type 4 version 3 (80 bytes); see [wire format](../../docs/protocol/CAR_MQTT_HOOKUP.md).
