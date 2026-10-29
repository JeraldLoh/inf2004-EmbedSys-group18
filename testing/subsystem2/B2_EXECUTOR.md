# B2 movement executor and controller ingress

**29 October update:** CAR_RUN is now wired to the measured engine through its
existing controller-owned callbacks and the `car_motion` compatibility adapter.
The draft is retained only for reference/tests. Gains remain unqualified and
CAR_RUN START is disabled until measured calibration is supplied. See
[CAR_RUN_MEASURED_PID.md](../integration/CAR_RUN_MEASURED_PID.md) for current
integration and test scope; the standalone typed task instructions below remain
valid for that separate runtime path.

This adds reusable movement primitives to Buddy 2. The vehicle controller owns
permission, source arbitration, generation and profile selection. B2 owns wheel
measurement, separate left/right PID, PWM commits, finite goals and results.
Buddy 3 supplies continuous speed/curvature requests; higher mission code chooses
when to turn, reverse or travel a distance. Missions are not hardcoded in B2.

On the testing branch this implementation is named `measured_motion_executor.*`
with the `measured_motion_executor_` API prefix. CAR_RUN uses it via `car_motion`.
The typed `motion_task` runtime is separate and is not started in CAR_RUN.

## Supported internal commands

`controller_motion_request_t` in `controller_motion.h` supports:

- START: create fresh authority; it leaves motor outputs zero.
- PROFILE: STOP, CAUTION or CRUISE. Moving profiles select a configured speed cap;
  selecting a profile never starts motion. STOP preserves the moving selection.
- CONTINUOUS: speed in mm/s, signed whole reciprocal-metre curvature (positive
  right), optional speed cap. Requests replace the latest continuous request.
  Negative speed is rejected; reverse uses a bounded goal.
- GOAL: forward/reverse distance in positive mm, or left/right pivot angle in
  positive millidegrees, maximum speed in mm/s and optional execution timeout.

Examples: forward 50 cm is FORWARD with `mm=500, mdeg=0`; left 90 degrees is
PIVOT_LEFT with `mm=0, mdeg=90000`. `speed_mm_s=0` on a goal selects the current
profile speed. These are parameterized commands, so a new preset does not require
a new motor routine. The controller can build presets from these fields.

This is an internal typed interface. Existing signed SET_SPEED payloads, MQTT
commands and compact wire opcodes are unchanged; no network parser is added.

Each request carries a nonzero boot session, per-source increasing ID, current
authority generation, creation time and expiry time. TTL is at most 1000 ms in
the kernel millisecond clock domain. Duplicate, stale, expired, wrong-session or
wrong-source requests cannot renew a lease. AUTO accepts onboard sources;
MANUAL accepts manual and CALIBRATION accepts calibration. Existing obstacle
mission arbitration still applies. Query current generation before START and
again after it; START advances it. A session must not be reused across boots
while old messages may still arrive.

## Measurement and finite goals

Left and right B-channel encoder counts remain independent, with separate
counts-per-revolution calibration. Distance accumulates in micrometres with
fractional carry; speed uses actual elapsed time and a configured window.
Counter/time wrap is supported within the bounded acquisition gap. Reset-epoch
changes, coalesced edges, bad acquisition and implausible rates fail closed.
Channel B supplies magnitude only: command signs and signed result distances
are inferred, not independent measurements of actual rotation direction or
ground position. Slippage and wrong physical direction need hardware checks.

Both wheels must reach the configured tolerance and settle before success.
One arrived wheel stops while the other catches up. Per-wheel stalled progress,
overshoot and a finite execution deadline prevent an indefinite goal. A timeout
of zero derives a bound from path/speed, acceleration and margin; caller bounds
shorter than that estimate are rejected. At 30 mm/s a 1 m goal necessarily needs
more than 33.3 seconds, plus these margins. Pivot wheel path uses configured
effective track width and therefore needs floor calibration.

PID gains, feedforward, saturation, integral limits, launch assistance, ramp and
stop/settling limits are supplied explicitly. Each wheel has its own PID state.
Derivative acts on measurement; integration respects saturation. There are no
tuned production defaults. Test fixtures are synthetic and do not qualify the
car. `qualified=false` permits setup but rejects motion authorization.

## Runtime integration

Normal startup is unchanged: this implementation does not enable autonomous
driving or start a new task automatically. Root integration supplies measured
configuration and a nonzero boot session, checks every return value and follows
the course all-init-before-any-start order:

1. Initialize the RP2040 B2 platform, bind its HAL, and call `subsystem2_init()`.
2. Obtain `b2_platform_executor_io()` and call `motion_task_init(config, io, session)`.
3. Initialize the controller using its existing urgent STOP output hook.
4. Obtain `motion_task_outputs()` and bind them with
   `controller_task_bind_motion(outputs, session, caution_mm_s, cruise_mm_s)`.
5. After all subsystem initialization succeeds, start the foundation,
   controller and motion task. Starting these tasks does not authorize movement.
6. Producers use `controller_post_motion()` and inspect acceptance/status;
   only a valid explicit START followed by a fresh request can produce motion.

The motion task has an explicit 4096-byte stack, provisional priority 6 and core
0 affinity. The controller has a 2048-byte stack and provisional priority 10.
Measure stack/CPU/timing with the full system before changing priorities or
claiming a resource saving. Raw bench and CAR_RUN builds refuse standalone
typed-task driver binding, preventing competing PWM owners. CAR_RUN uses its
private measured adapter instead. The typed controller
binding also rejects a legacy line-drive hook. B5_RUN remains
its existing STOP-only path. Buddy 3/5 CAR_RUN producers use the measured adapter
as described in the current integration guide.

## STOP, transport and results

The controller has a 16-entry bounded FIFO plus a separate latest continuous
slot. Ordinary finite messages reject a full FIFO and require caller retry.
STOP publishes a sticky reason and disables the driver independently of FIFO
capacity; pending commands cannot clear the driver STOP sequence. Clear/profile
selection cannot resume motion. Physical STOP evidence uses peek/ID-matched ACK
only after the controller latches it; a held GP21 continues to inhibit motion.

The same RP2040 timer alarm is armed before every nonzero commit. Loss of task
service cuts off PWM; its duration is explicitly configured and at most 500 ms.
This watchdog bound is separate from the required physical STOP latency of
50 ms. The ISR/lock/output latency has not been measured on hardware.

One terminal-result slot is reserved in the executor. No new goal overwrites it.
Failed queue submission leaves it pending for retry. Successful submission
transfers retention to the controller FIFO; matching ID/generation acknowledges
the executor slot. Results are immutable while pending, including after further
STOPs or faults. The controller retains the latest terminal result and sequence
for status consumers; it does not maintain an unlimited result history.

## Software verification (2026-10-28)

Run from the repository root with a native C11 compiler:

```powershell
cmake -S testing/subsystem2/b2_01 -B build-b2-executor -G Ninja
cmake --build build-b2-executor
ctest --test-dir build-b2-executor --output-on-failure
cmake -S testing/host -B build-host-executor -G Ninja
cmake --build build-host-executor
ctest --test-dir build-host-executor --output-on-failure
```

The B2 suite covers real driver/executor code against fake HAL input, finite
forward/reverse/pivot completion, unequal wheel progress, stall/overshoot,
STOP during watchdog/commit, lease expiry, calibration admission, retained
results, wrap/fractional measurement, PID and controller admission. Transport
tests execute the actual controller task loop with a deterministic kernel mock:
full-queue STOP, latest continuous replacement and result retry. They establish
software behavior, not actual RTOS scheduling or electrical timing.

The course Make compiles all new source files into normal, B2_BENCH and B5_RUN
images. No image was flashed and no motor trial was performed. Before a later
authorized physical session, qualify disabled boot, wheel direction/calibration,
load, STOP timing, speed gains and effective pivot width with the existing bench
guide. This document is not evidence that those hardware gates passed.
