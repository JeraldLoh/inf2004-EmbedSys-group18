# CAR_RUN measured PID integration — 29 October 2026

Based on testing ce61f9e, preserving the team's junction, curve-hold, line-loss,
ADC/timer and Buddy 4 changes. No flashing or car movement was performed.

## What runs now

`CAR_RUN=1` uses the measured wheel controller:

```
B3 line/junction -> controller service -> subsystem2_motion -> car_motion
B5 avoidance -> controller authority gate -> subsystem2_motion -> car_motion
car_motion -> measured_motion_executor -> independent wheel PID -> motor driver
encoders -> coherent capture -> wheel measurement -> measured_motion_executor
goal result -> source-qualified poll -> junction/B5 state machine
```

The old `motion_executor.*` draft remains for regression/reference; it no longer
drives CAR_RUN. The separate typed `controller_motion`/`motion_task` API remains
available, but CAR_RUN retains its existing controller-owned producer callbacks.
It does not start that second task or bypass the controller with wire commands.
Both runtime paths share the measured engine, not concurrent PWM ownership.
The old draft's open-loop right trim and minimum-effort Make definitions are
not PID calibration settings; the measured configuration is in `car_motion.c`.

All initialization now precedes subsystem starts. The CAR_RUN motor task receives
private driver pointers during init, runs on core 0 at provisional priority 6
with a static 4096-byte stack, and services actual elapsed time every nominal
20 ms. It arms the existing 100 ms hardware watchdog before motor commits.
Provisional periods, stack and scheduling still require target measurement.

## Explicit compatibility conversions

- Existing CAR_RUN steering uses micro reciprocal metres and positive LEFT.
  The measured engine uses whole reciprocal metres and positive RIGHT. The
  adapter negates and rounds the value and caps it to the no-reverse-wheel
  curvature bound. A positive Buddy 3 right-steer therefore makes the left
  wheel target faster than the right. Buddy 5 left search stays left.
- Continuous requests retain their original kernel-ms expiry. Expired/future
  envelopes do not acquire a fresh lease by passing through this adapter.
- Junction and Buddy 5 goal IDs have separate namespaces. The adapter allocates
  monotonic engine IDs and translates results back to external source/ID and
  controller generation. Both runtime consumers use source-qualified polling.
- B5 and junction defaults now request `timeout_ms=0`: B2 derives a finite
  path/speed, acceleration and settling bound. Nonzero caller deadlines are
  still checked and too-short values rejected. A 1 m goal at 30 mm/s cannot
  fit in the old fixed 4 s timeout.

## STOP and results

A controller manoeuvre halt commits zero, cancels its outstanding goal and
blocks in-flight LINE commands at that generation. It can admit a new explicit
obstacle goal under the same valid scope. Physical/urgent STOP, faults and lease
expiry retain the driver's sticky inhibit and cannot rearm that generation.
Controller mission changes cancel old work before new-generation admission.

Terminal results stay in the engine's reserved slot until polled. The adapter
then retains a copy. An explicitly abandoned scope/halt transfers its cancelled
terminal into that diagnostic copy so a discarded mission cannot permanently
block a new one. A subsequent accepted goal can replace the prior diagnostic
copy; this is a latest-result interface, not unlimited history.

Junction sequencing polls intermediate results before issuing the next goal.
The controller sees the final route result after approach plus pivot, so line
tracking cannot resume just because the approach step completed. Buddy 5 polls
its own results without consuming a junction result with the same numeric ID.

The supervisor peeks physical STOP/cutoff evidence and only acknowledges its
matching ID after controller status shows the reason latched. A full or delayed
queue cannot clear pending hardware evidence. Existing line-loss and run-limit
stops remain in place. Software shutdown timings are not physical pin timings.

## Qualification gate: this image does not yet enable driving

`car_motion_candidate_config()` supplies encoder calibration candidates and
explicit control bounds. Its gains are zero and `qualified=false` because the
new PID/load/STOP/pivot configuration has not been physically qualified. Boot
and an attempted START report this state; START is refused and outputs remain
disabled. Do not set qualified merely to get past this check. Supply measured
per-wheel gains and evidence after the motor/power and STOP checks succeed.

The user's bench traces now show independent manual encoder feedback. Powered
wheel motion was not established by those traces: the last recorded right-wheel
pulse had zero edges. This integration does not diagnose or fix that hardware
issue, and software tests do not establish distance/turn accuracy or <=50 ms
physical STOP latency. Continue the B2 raised-wheel guide in a later authorized
hardware session before floor driving.

## Software evidence

Final native result: **66/66 B2 tests and 8/8 shared host suites pass**. The
incoming branch removed Buddy 3's separate sensor suite, so the current shared
count is eight rather than the previous publication's nine.

The B2 suite includes seven CAR_RUN core scenarios using the actual controller
service, B5 motion logic, adapter, measured engine and fake motor/encoder HAL:
qualification refusal, steering conversion, obstacle takeover/line return,
forward and pivot goal completion, junction approach/turn sequencing, STOP and
source-colliding IDs, and finite deadlines/expiry. Synthetic test gains are
explicitly qualified only inside fixtures.

Four further tests execute the actual `car_run_target.c` supervisor against
bounded mocks: all-init-before-start, qualified/unqualified START, init failure
with no dependent tasks started, and delayed STOP acknowledgement. They do not
simulate a real scheduler or measure SMP latency.

The shared host perception regression was updated to call Buddy 4's current
`imu_decode_accel_mg` API after its old decoder source was removed upstream.
Buddy 4 production code is unchanged. The separate `app_program` aggregate test
runner still references the missing `testing/host/test_terrain_estimator.c` from
the incoming branch; that runner cannot configure. Its failure is recorded
separately and is not counted as a pass.

Build and test from the repository root:

```powershell
cmake -S testing/subsystem2/b2_01 -B build/b2-host -G Ninja
cmake --build build/b2-host
ctest --test-dir build/b2-host --output-on-failure
cmake -S testing/host -B build/shared-host -G Ninja
cmake --build build/shared-host
ctest --test-dir build/shared-host --output-on-failure
```

Firmware uses the existing course Make profiles: normal, B2_BENCH, B5_RUN and
CAR_RUN, with SMP=1, USB CDC, CRASH_REPORT=0 and networking disabled. No generated
image is committed or flashed as part of software verification.
