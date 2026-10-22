# Architecture Baseline

Status: inactive course-template framework candidate. Course source inspection
is complete for layout/startup/network ownership; G0 runtime qualification and
G1 remain open. No application tasks or drivers are started.

## Runtime authority

1. Perception and planning modules publish evidence and proposals.
2. The Vehicle Controller owns modes, mission state, arbitration, and recovery.
3. The motion task is the only normal runtime owner of motor-driver commands.
4. Telemetry observes snapshots and events and cannot block control.
5. An urgent stop path is independent of ordinary command queue capacity.

## Buddy boundaries

| Area | Primary owner | Published output |
|---|---|---|
| Communications | Buddy 1 | Validated remote requests and connection state |
| Motion | Buddy 2 | Goal results, speed/odometry snapshots, motion faults |
| Line/barcode/junction | Buddy 3 | Steering and junction evidence, decoded commands |
| Terrain/hump | Buddy 4 | Terrain state, speed cap, height result and validity |
| Sonar/obstacle | Buddy 5 | Range/profile evidence and avoidance proposals |
| Vehicle Controller | Shared; Buddy 5 integration lead | Authorised motion and system state |

The course implementation areas are `app_program/subsystem1` through
`subsystem5`, matching buddy numbers, with descriptive Buddy-labelled guides.
Primary ownership means accountability/review, not exclusive authorship. The
controller remains separate in `app_program/vehicle_controller`; its dedicated
RTOS task is not implemented yet. Shared contracts and vehicle configuration
live in `app_program/contracts` and `app_program/project_config`.

Raw CYW43/lwIP/MQTT ownership belongs to the course's built-in service task.
Buddy 1's application processing will exchange bounded data through a reviewed
bridge rather than creating another raw networking task. This is a refinement
of the provisional Week 6 task description, not a measured implementation.

The root `config` and kernel/device/library folders retain course responsibilities.
See `../platform/course-template-review.md` for SMP, startup and build constraints.

## Data and authority flow

```text
sensors -> acquisition -> perception/planning -> Vehicle Controller
                                              -> motion -> motor driver

all modules -> bounded telemetry snapshots/events -> communications
```

## Stable rules

- Continuous requests expire unless refreshed.
- Finite goals have IDs, timeouts, cancellation, and terminal outcomes.
- Mode changes increment an authority generation and invalidate older work.
- Stop reasons are independent and are cleared individually.
- A missing navigation command stops the vehicle only at a confirmed junction
  that requires a branch decision.
- Terrain limits compose with other limits using the lowest applicable cap.
- If bounded recovery cannot establish safe continuation, the controller stops.

## Hardware map

The team-agreed map is `app_program/project_config/pinmap.h`: motors GP8-GP11,
encoders GP27 (left) / GP5 (right) on channel B, line sensors GP7 (left) / GP0
(right), barcode GP26/ADC0, IMU GP2/GP3 (I2C1), sonar GP16/GP17 (5 V echo through
a divider), servo GP12, and start/stop GP20/GP21. It is not yet electrically
verified (G1).

No pin is configured by this framework.
