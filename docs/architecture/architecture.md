# Architecture Baseline

Status: implementation framework candidate; team confirmation and G0 remain
open.

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

The corresponding future implementation areas are `buddy1_comms`,
`buddy2_motion`, `buddy3_perception`, `buddy4_terrain`, and
`buddy5_obstacle`. Folder naming identifies primary accountability, not
exclusive authorship. The Vehicle Controller remains in the shared `app`
directory.

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

## Provisional hardware map

The Week 6 section 2 table is the baseline: motors GP8-GP11, encoders GP26/GP5,
line sensors GP0/GP1, barcode GP28/ADC2, IMU GP16/GP17, sonar GP6/GP7, servo
GP12, and start/stop GP20/GP21. The later GP2/GP3 sonar statement is not used.

No pin is configured by this framework.
