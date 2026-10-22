# Inspected course platform constraints

Source: official course template commit
`c3b09544cb1fd6dac6ad4b8f523be0af59b44482`. These are source findings, not board
measurements or a closed G0 gate. Unmodified examples are under
`docs/template-reference/` from the repository root.

## Application startup

The kernel's initial task calls `usermain()` in `app_program/app_main.c`; returning
shuts the system down. The example creates every subsystem's objects first, then
starts tasks. In SMP another core can run a started task immediately, so creation
of ALL cross-subsystem objects must precede ANY application task start.

Our separate controller must join that lifecycle. It is not a sixth MCU or a
sixth buddy; its task, ingress objects, startup failure policy and adapter have
not been implemented. The migrated hook is inactive and does not call the buddy
placeholders. The upstream CHECK macro logs errors and continues; it must not be
treated as a safe all-or-nothing vehicle startup policy without further design.

## Build and file discovery

The target build is GNU Make under `build_make/`, not our old standalone Pico
CMake project. The upstream Windows instructions use WSL Ubuntu. SDK 2.2.0 is
specified and 2.3.0 is reported tested by upstream; no local SDK/toolchain/WSL
combination is qualified by this migration.

`build_make/mtkernel_3/app_program/subdir.mk` uses exactly:

```make
$(wildcard ../app_program/*.c) $(wildcard ../app_program/*/*.c)
```

This is narrower than the README's broad subfolder wording. Drivers stay directly
in their subsystem; deep `.c` directories need an explicit shared build-rule change.
Host tests stay outside `app_program` to avoid a second `main()` in target sources.

Default options: `SMP=1 CONSOLE=usb_cdc WIFI=cyw43 NET=none MQTT=0 TEST=0
CRASH_REPORT=0`. Wi-Fi on does NOT imply an enabled TCP/IP/MQTT application.
`NET=lwip MQTT=1` needs the SDK lwIP submodule and local `config/wifi_secrets.h`.
Do not create, print or commit real credentials during structural work.

Make evaluates shell actions for object directories/profile changes. Do not run
even `make -n` as a supposedly non-mutating check in this migration.

## Networking ownership

`cyw43_utk_kernel.c` creates the built-in service at priority 4, core 0 affinity
when SMP is enabled, with a 6 KiB stack or 8 KiB for lwIP. MQTT runs in that same
service context. `mqtt_utk.h` exposes a start hook intended for the owner context,
not a general robot-command publish/subscribe API. The example heartbeat is not
proof of the robot's required telemetry, remote commands or reconnection policy.

The template permits status access through `cyw43_utk_get_status()`. Application
messaging still requires a designed bounded bridge and a reviewed service-side
hook; the course asks for team/instructor agreement for shared network-library
changes. Buddy 1 must not create an independent raw lwIP/MQTT owner on either core.

## Pins, console and peripheral ownership

Radio pins GP23/24/25/29 are reserved. Our right line sensor uses GP0 (UART0
TX). The actual build options make UART console force crash reporting; either
UART console or USB crash mirroring uses GP0/GP1. Plan USB-only,
`CRASH_REPORT=0`, while the line sensors are attached. Some early Makefile comments still describe older mirroring
behavior; use the option logic and actual console code, then verify on hardware.

The vehicle pin declarations are in `app_program/project_config/pinmap.h`.
Root `config/` controls the kernel, devices, services and object limits.
Application drivers are `drv_name.c/.h` in a subsystem, not new files under
the kernel's `device/` tree. Record each peripheral's owning task in TEAM.md.

## SMP, interrupts and scheduling

Default SMP uses both RP2040 cores. A `volatile` counter is not a coherent
multi-field snapshot, and disabling interrupts on one core does not exclude the
other core. The portable controller is not thread-safe for concurrent mutation;
it needs a single task owner and suitable event transport.

Kernel services run at higher priority than the example application bands (USB 3,
Wi-Fi 4; lower numeric value means higher priority). Do not reuse S1-S5 example
bands as if they met motion/ADC/sonar deadlines. Budget enabled kernel services,
all application tasks, stacks and communication before allocating object IDs.

Encoder and sonar IRQ routing, interrupt-context legality of each selected kernel
API, timer reservations, time units, SMP critical sections, acquisition pacing,
priority inversion and callback locking are remaining source/integration checks.
Do not transplant standalone SDK GPIO callbacks into this kernel without review.

## Follow-up before runtime work

- Confirm a supported WSL/native toolchain and exact SDK/submodule versions.
- Agree the task/priority/core/stack map and memory/object budget.
- Select and inspect the actual ISR-safe signalling and locking primitives.
- Design controller plus all-init-before-any-start lifecycle with failure rollback.
- Agree the service-side network bridge and approval path.
- Keep G1 and all hardware calibration/evidence separate from G0 source inspection.
