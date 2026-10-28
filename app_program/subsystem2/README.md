# Buddy 2 - Motion

Main-based publication, reproducible commands and dependency/test limits are
recorded in `../../testing/subsystem2/PUBLISH_VERIFICATION.md`. Generated firmware
is not committed; hardware qualification remains pending.

Primary responsibility: motor output, encoder snapshots, wheel control,
odometry, continuous requests, finite goals, and motion faults.

This subsystem must remain the sole normal runtime writer of motor-driver
commands. Other buddies may contribute, but Buddy 2 remains accountable for
its contracts, calibration, and evidence.

The course folder is `subsystem2`. B2-01 now provides `drv_motor`, `drv_encoder`,
an injected `motion_hal` and a course-BSP `motion_hal_rp2040` adapter. Driver C
files stay directly here because the make source glob supports one folder level.
No periodic control task, PID, speed profile, odometry or goal executor is added
by this foundation. `motion_math.c/.h` remains portable integer-mm arithmetic;
it is not the future fractional low-speed estimator.

`subsystem2_bind_hal()` must run before init, with a reviewed shared lock,
microsecond clock and encoder dispatcher. Without it init/start return
`E_NOSPT`. Bound init drives GP8-GP11 low through SIO, disables the slices,
configures GP27/5 inputs and subscribes to the shared dispatcher. Init/start
never clear inhibit, arm, move or calibrate the car. There are no allocations
or tasks in this slice. Normal `app_main` remains unchanged and starts no
vehicle tasks. This is deliberately not a hardware-test image.

The motor driver serializes all commits and persistent urgent STOP through the
same injected IRQ/core lock. It preserves the diagnostic's low-SIO-before-PWM-
disable fix; every shutdown operation is attempted even on HAL failure. HAL
success is not electrical feedback. Clear requires the current STOP sequence;
arm additionally requires a fresh generation and leaves outputs off. Nonzero
effort needs the armed generation. Direct powered sign reversal is rejected
with STOP/inhibit; later control must qualify reversal deadtime and start
authority. Effort is signed per-mille, not speed. No profile policy is stored
in the driver; the controller retains that responsibility.

Snapshots contain both unsigned U32 modulo edge counts, acquisition time, reset
epoch, per-wheel last-edge time/presence and command-direction metadata under
one lock. Counters persist across goals; explicit reset changes epoch. Stopped
fresh counts and old edge time are distinct. Future/reordered or half-domain-
ambiguous edge times are marked invalid. Coalesced rising+falling bits count
two observed edges and flag potential loss; target edge-rate/dispatch timing
still needs measurement. Commanded sign is never independently measured sign.

The HAL lock contract requires cross-core exclusion and local ISR masking
before lock acquisition; callbacks cannot block, allocate, reenter, or leave a
preempted owner holding a lock needed by STOP. No default spinlock/IRQ callback
is installed here. The integration lead owns the real primitive, sole GPIO
dispatcher, physical STOP evidence and lifecycle hookup. Do not run the manual
diagnostic concurrently with this motor owner.

Historical bench bring-up confirmed both quadrature channels on both motors
with approximately 1,200 both-edge transitions during a visually judged wheel
revolution. This confirms the signal path but is not precise calibration.
Production can use channel B only on GP5 (right) and GP27 (left), deriving sign
from the commanded motor direction, because left channel A on GP26 conflicts
with the barcode ADC. Leave both red channel-A wires insulated when unused;
never reconnect them to 3V3. Raw three-by-ten-turn runs support candidate
1271 left/1264 right both-edge counts/revolution; metadata is in
`motion_calibration.h`. The 65 mm diameter is user-measured. Neither those
polling runs nor supplied 112 mm geometry qualify effective floor/pivot scale,
an IRQ/filter convention, or PID constants.

Use shared motion contracts and explicit completion/cancellation/timeout/fault
results. Do not copy old blocking movement routines or assume local interrupt
disabling protects data across both cores. Evidence belongs in
`../../testing/subsystem2/`. The old standalone bench UF2 is not the integrated
kernel application.

See `../../docs/migration/standalone-port-inventory.md` before moving any more
files from the sibling Buddy 2/3/4 bench projects.

Software tests and platform contract: `../../testing/subsystem2/b2_01/README.md`.
28 new CTest cases and all four existing host tests pass; all five foundation
sources compile with real Cortex-M0+ course headers and the full inactive
course image links. STOP <=50 ms, hardware lock/dispatcher timing, encoder edge
loss and powered floor/load/ramp acceptance are not established by these tests.

The later runtime slice now supplies `b2_platform_rp2040`, the shared
`project_config/gpio_dispatcher`, a bounded `b2_trial` controller adapter and a
manual `b2_gesture` / bench task. It is enabled only by `B2_BENCH=1`, with a
distinct image; normal startup remains inactive. The bench task is pinned to
core 0 and uses the existing interrupt-preserving SMP lock. GP21 STOP and
alarm0 cutoff act independently of the ordinary task loop. The fixed menu
requests 15% effort for 250 ms, never automatically at boot.

38 B2 tests now pass, plus the four shared host tests; runtime target/normal
builds pass. Hardware execution/timing remain pending. See
`../../testing/subsystem2/B2_RUNTIME_TEST_GUIDE.md` before any later test.

## DRAFT motion executor (`CAR_RUN=1` only) - for Buddy 2's review

Written by Buddy 5 as integration lead so the whole car can be tested
(decision D13). Buddy 2 owns it: review, change or replace it before it is
relied on. Nothing here changes the B2-01 drivers, the bench or the default
build.

| File | Runs on | Purpose |
|---|---|---|
| `motion_executor.c/.h` | both | Portable core: continuous requests and finite goals -> wheel effort (host-tested) |
| `subsystem2_motion.c/.h` | Pico | Task (core 0, priority 8, 5 ms): the only `drv_motor_apply()` caller |
| `subsystem2.c` / `subsystem2_internal.h` | Pico | `subsystem2_executor_motor/encoder()` accessors, `APP_CAR_RUN` only |

Interfaces (types from `contracts/control_messages.h`):
- IF-05 in: `subsystem2_motion_continuous()` (line tracking, search arcs)
  and `subsystem2_motion_goal()` (forward/reverse mm, pivot mdeg).
- IF-06 out: `subsystem2_motion_goal_result()`: PROGRESS, then COMPLETED,
  CANCELLED, TIMEOUT, ABORTED or FAULT.
- `subsystem2_motion_halt()`: zero output at once (wakes the task).
  `subsystem2_urgent_stop()` is unchanged and still latches the inhibit.

Safety rules it keeps:
- Arms like `b2_trial_begin()`: refused while STOP is held or pending, fresh
  authority generation, clear inhibit, arm.
- Keeps the hardware cutoff alarm armed (100 ms) while driving: a stalled
  task stops the motors by itself.
- Commands with an older authority generation are rejected. After a driver
  refusal or STOP, the same generation is rejected too (IF-07: fresh
  authority before resuming).
- A wheel changing direction gets one zero period first.
- Continuous requests expire (150 ms from the controller) and never reverse a
  wheel. Goals end on encoder counts or their timeout.

Placeholders that need measuring (all in `motion_executor_default_config()`):

| Value | Now | How to measure |
|---|---|---|
| Track width | 130 mm (`MOTION_TRACK_WIDTH_MM_PLACEHOLDER`) | Ruler between wheel contact centres; then a 360 deg pivot on the floor |
| Effort per 100 mm/s | 133 (150 mm/s -> 200) | Raised wheels: read `speed L/R` on the `[CAR]` line at known efforts |
| Goal minimum effort | 150 | Lowest effort that starts both wheels on the floor |
| Max effort | 350 | Buddy 3's bench cap |
| Wheel diameter / counts | 65 mm, 1271/1264 | Already in `motion_calibration.h` |

Open-loop speed: there is no PID yet. Speed varies with battery and load.
Host tests: `testing/host/test_motion_executor.c`.
