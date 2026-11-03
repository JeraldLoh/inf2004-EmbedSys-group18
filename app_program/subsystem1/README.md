# Buddy 1 — WiFi/MQTT communication

Buddy 1 copies and validates owner-produced telemetry, encodes binary packets,
and receives START/STOP requests for the shared vehicle controller. It does not
own motor speed, line following or obstacle-avoidance decisions.

The kernel WiFi service is the only raw CYW43/lwIP/MQTT caller. Protected snapshots
and bounded message buffers connect Buddy 1 to that service. The combined robot
profile is `CAR_RUN=1 CAR_MQTT=1`; the standalone demo is a separate simulated mode.

See [Buddy 1 setup and operation](../../BUDDY1_README.md) for normal startup,
configuration, building, flashing, troubleshooting and evidence limits.
See [integration/protocol notes](../../docs/protocol/CAR_MQTT_HOOKUP.md) for the
robot-facing interfaces and supplementary live-status wire format.
