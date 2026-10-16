# Buddy 1 - Communications

Primary responsibility: validated commands, Wi-Fi/MQTT state, bounded
telemetry, heartbeat, and reconnection.

The network backend and callback context remain open until G0. Other buddies
may contribute, but Buddy 1 remains accountable for this subsystem's contracts
and evidence.
