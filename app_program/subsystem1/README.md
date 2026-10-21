# Buddy 1 text communications

Ported onto team main; kernel, contracts, controller, pin map and subsystems 2-5
are retained. The radio service remains the sole raw CYW43/lwIP/MQTT owner.
subsystem1 owns protected readings, bounded message buffers and text validation.
See ../../BUDDY1_README.md for build and test instructions.

Default application startup is inactive. B1_DEMO_RUN=1 opts into the standalone
demo only. This is not an integrated vehicle runtime or a verified hardware image.
