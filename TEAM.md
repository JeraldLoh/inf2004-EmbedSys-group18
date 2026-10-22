# Group 18 team and subsystem ownership

Status: architecture scaffold; electrical verification, numeric application
priorities, object allocation and runtime integration are incomplete. The course
example table is preserved in `docs/template-reference/TEAM.md`.

## 1 Members and collaboration

| Buddy | Primary responsibility | Code | Test evidence |
|---|---|---|---|
| 1 | Communications, validated requests, telemetry | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motor outputs, encoders, wheel control, motion outcomes | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line, Code 39 and junction perception | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU, terrain, hump height and validity | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Sonar, scan/profile, avoidance proposals, integration lead | `app_program/subsystem5/` | `testing/subsystem5/` |
| Shared | Separate Vehicle Controller and stop policy | `app_program/vehicle_controller/` | `testing/host/`, `testing/integration/` |

All five may contribute to shared controller code. Primary ownership means
accountability and review, not isolated applications. Use reviewed branches.
Cross-subsystem changes require the affected owner's review; shared startup,
pins, contracts, safety and library hooks need affected owners and Buddy 5.
Follow the instructor-approval requirement for network library hooks. Add
names, student IDs and GitHub usernames after the team confirms them.

## 2 Pin claims

These follow the team-agreed `app_program/project_config/pinmap.h`. They are
not yet a physical inspection (G1) result. Grove numbers are from the Robo
Pico datasheet. Buddy 1 (communications) has no application GPIO.

| Pins | Owner | Purpose / qualification |
|---|---|---|
| GP23, GP24, GP25, GP29 | Course/Pico W | Radio, reserved |
| GP7 | Buddy 3 | Left digital line IR, Grove 7 (shared connector with GP28) |
| GP0 | Buddy 3 | Right digital line IR, Grove 1. GP0 is UART0 TX: NO UART console or crash mirroring while connected |
| GP26 / ADC0 | Buddy 3 | Barcode analog, ADC channel 0. GP26 is exposed on both Grove 5 and Grove 6 |
| GP5 | Buddy 2 | Right encoder, channel B, Grove 3 |
| GP27 | Buddy 2 | Left encoder, channel B, Grove 6 (shared connector with GP26). Must stay a digital input: keep ADC channel 1 disabled |
| GP8, GP9 | Buddy 2 | M1 / intended left motor, PWM slice 4 A/B |
| GP10, GP11 | Buddy 2 | M2 / intended right motor, PWM slice 5 A/B |
| GP2, GP3 | Buddy 4 | IMU SDA/SCL on I2C1 (Grove 2 / Maker port); claimed at boot by the I2C driver |
| GP16, GP17 | Buddy 5 | Sonar TRIG / ECHO, Grove 4. The 5 V echo goes through a voltage divider |
| GP12 | Buddy 5 | Optional scan servo, PWM slice 6 A; mechanism unverified |
| GP20, GP21 | Shared controller adapter | User START/STOP; adapter unimplemented |
| GP18, GP22 | Carrier-reserved for planning | Onboard RGB/buzzer; no driver added |
| GP1, GP4, GP6, GP13-GP15, GP19, GP28 | Unclaimed by this application | Inspect actual harness before claiming |

The upstream demo's GP19 LED claim is not adopted. Connector number and wire
colour do not prove a GPIO. G1 remains open for power, common ground, the
sonar echo divider on GP17, IR/encoder voltage and boot/reset behavior. GP28
is no longer used, so its VBAT-sense jumper only matters if GP28 is claimed
again.

## 3 Runtime resource owners

| Resource | Planned owner | Other users / rule |
|---|---|---|
| Motor PWM slices 4/5 | Buddy 2 motion task | Sole motor-output writer; others request motion or urgent stop |
| Encoder counts | Buddy 2 acquisition/ISR adapter | Coherent snapshot/reset and SMP policy unimplemented |
| ADC, GP26 channel 0 | Buddy 3 acquisition task | Coordinate the entire ADC block if channels are added; channel 1 (GP27) is the left encoder and must stay digital |
| I2C1 ("iicb"), GP2/GP3 | Buddy 4 IMU task | No second task directly drives the block |
| Sonar and optional servo slice 6 | Buddy 5 acquisition/scan | Explicit per-peripheral ownership; GPIO IRQ routing reviewed centrally |
| Controller state | Future dedicated controller task | Other tasks submit events, not concurrent state mutations |
| USB console | Built-in kernel service | Bounded task logging, not timing-critical ISR printing |
| CYW43/lwIP/MQTT | Built-in `cyw43_utk_task` | Buddy 1 uses a reviewed bridge, never a second raw-stack task |

Application drivers belong in the owning subsystem as `drv_name.c/.h`, not
under `device/`. Global configuration and library changes are shared review items.

## 4 Tasks and scheduling

No application tasks are created by the migrated scaffold. Numeric priorities,
stack sizes, core affinity and object counts remain TBD; the LED/producer/
consumer demo schedule is not adopted as the vehicle's schedule.

Smaller numbers mean higher priority. The template reserves 1-4 for kernel
startup/services, with USB at 3 and Wi-Fi at 4. Default `SMP=1` permits simultaneous
execution on both cores. Agree periods/deadlines, worst-case execution, blocking
and stacks before selecting priorities. Service load still needs measurement.

Expected functions, not allocated tasks yet: B1 telemetry/commands; B2 wheel
control/goal execution; B3 acquisition/perception; B4 IMU/terrain; B5 detection/
scan planning; a separate shared controller. A subsystem can contain several
tasks. ISR priority and task priority are different concepts.

## 5 Interfaces

These are proposed semantics, not allocated kernel IDs or finished adapters.
The owner creates each object during init and publishes its API/ID.

| ID | Direction | Meaning / planned ownership |
|---|---|---|
| IF-01 | B1 -> controller | Validated remote request with authority/freshness; controller ingress |
| IF-02 | B3 -> controller | Steering, line/junction evidence and valid route instructions |
| IF-03 | B4 -> controller | Terrain cap, height validity and faults |
| IF-04 | B5 -> controller | Range/profile and bounded avoidance proposals |
| IF-05 | Controller -> B2 | Authorised continuous request or finite goal; B2 ingress |
| IF-06 | B2 -> controller | Accepted/progress/completed/cancelled/timeout/fault result |
| IF-07 | Safety sources -> B2/controller | Urgent stop independent of normal queue capacity; fresh authorisation before resuming |
| IF-08 | All -> B1 | Bounded telemetry snapshots/events with explicit overload policy |
| IF-09 | B1 <-> built-in network owner | Reviewed bounded service bridge; hook unimplemented |

Types are in `app_program/contracts/`. Fields, transport definitions and policies
still need detailed review. A C struct is not a queue; a constant is not an
allocated object. Never asynchronously send pointers to temporary stack storage.
Specify mailbox block lifetime, queue-full behavior and SMP locking.

## 6 Kernel object budget

Imported config limits: 32 tasks; 16 semaphores; 16 event flags; 12 each of
mailboxes, mutexes, message buffers, fixed/variable pools, cyclic and alarm
handlers. These are limits, not spare capacity. Kernel startup, USB/Wi-Fi tasks
and internal objects also consume resources. No application allocations are
made yet. Inventory enabled services, stacks and objects before changing
`config/config.h` or claiming available capacity.
