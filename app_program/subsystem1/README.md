# Buddy 1 - Communications

Primary responsibility: validated commands, Wi-Fi/MQTT state, bounded
telemetry, heartbeat, and reconnection.

The course service task is now verified as the sole raw CYW43/lwIP/MQTT owner.
Do not implement another application networking task that calls those APIs.
Buddy 1's application task(s) prepare telemetry, validate commands and exchange
bounded data with a reviewed service-side bridge. That bridge is not implemented.
The service exposes status reads and a heartbeat MQTT example, not a complete
robot telemetry/command transport.

The active folder is `subsystem1`. Put future driver/application `.c` files
directly here, within the Makefile's one-level discovery rule. Lifecycle hooks
are inactive `E_NOSPT` placeholders; target tests are SKIP placeholders. Other
buddies may contribute through reviewed changes. Save evidence in
`../../testing/subsystem1/`. Shared network hooks require integration review and
instructor approval where specified by the course template.
