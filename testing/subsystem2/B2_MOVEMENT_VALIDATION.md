# Buddy 2 movement-test v2 software validation — 29 October 2026

This revision merges the current testing branch (a112ed4, including Buddy 1
telemetry/remote commands) with Buddy 2 measured CAR_RUN integration and the
circuit/movement diagnostic profiles. Conflict resolution preserves Buddy 1
fields and uses measured-executor state/measurement freshness for telemetry.
Main is not a publication target.

## Movement-test changes

- Individual and both-wheel raised checks:30%, up to5 seconds each.
- Long raw trials exist only in B2_MOVE_TEST, with refreshed100ms cutoff.
  Premature watchdog cutoff is a failure; default bench/circuit caps remain.
- Straight PID:100mm/s for5 seconds or500mm wheel-rotation cap.
- Forward/reverse short goals:300mm (30cm). Other optional goals:1m/90degrees.
- Retained65mm wheel diameter,1271/1264 B-channel both-edge counts/revolution,
  ruler track112mm. Untuned PID candidates; production-qualified staysfalse.

## Software evidence

111/111 native B2/circuit/movement CTests pass, including extended raw duration,
short-watchdog refresh, premature-cutoff rejection, actual5-second straight
termination, STOP/stall, startup gates and watchdog observation.19/19 shared
CTest suites pass, including incoming Buddy 1 and remote START cases.

Full RP2040 movement-test and offline CAR_RUN firmware builds pass. Strict
real-header Cortex-M0+ compilation with-Wall-Wextra-Werror passes for merged
motor task, supervisor with CAR_MQTT enabled, B3 telemetry, trial and measured
executor. Normal-profile symbol checks confirm calibration authorization and
extended movement-trial initialization are excluded. Live MQTT/network hardware
and physical car behavior were not tested.

Build movement firmware from build_make:

```sh
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=0 CIRCUIT_TEST=0 B2_MOVE_TEST=1
```

Delivered UF2:group18_b2_movement_test_v2_2026-10-29.uf2.
SHA-256:7161b49afebed3392c172e408baa77dbe868c5b3d5b417a39e62b3d37c3246d9.
The packaged UF2/ELF/map, source snapshot and detailed logs are in the local
output/firmware/b2-movement-test-v2-2026-10-29 delivery directory. Generated
firmware files remain outside Git, following the repository's build-output
ignore rules. Source and build instructions are versioned.

No flashing or motor operation was performed. Physical direction, load/speed,
floor-distance accuracy, effective pivot track and STOP latency are pending.


## Version3 raw-test ownership fix — 2 November 2026

Operator trace: first LEFT trial364edges, RIGHT trials2edges, repeated speed214
or4mm/s while stationary and zero commanded efforts. Operator confirmed GP21
was pressed for the one-second trial ending3. That stop is expected. Actual
right-motor hardware failure is not established by this trace.

Found a test-only ownership defect: the diagnostic invoked measured executor
step while a separate raw trial owned PWM. Encoder acquisition failure called
the executor finish path, stopping/inhibiting raw output and cancelling its
watchdog. Cached status.wheels could still advertise valid old speed; raw
trial later ended on its task deadline despite already stopped outputs.

Fix: use a passive wheel-measurement observer in raw/idle modes, and step the
executor only during its active PID test. Initialize a fresh executor baseline
at each new PID test without resetting encoders or clearing ambiguity evidence.
Long raw trials reject unexpected inhibit/disarm or changed motor authority.
Readiness now requires100edges, recent motion in the final500ms, healthy speed
acquisition throughout and independent other-wheel counts. A tiny twitch does
not pass. Invalid/stale speed reports0/valid0; raw results report elapsed time,
feedback validity, coalescing and stop reason mask. Raw/PID logging is100ms.

114/114 native tests and19/19 shared suites pass. Three new actual-task
regressions inject simultaneous rising/falling edges, excessive rate and
three-edge-only twitch; full raw powered duration and PID gating are checked.
Trial regression covers unexpected other-owner shutdown. Full RP2040 movement
build and strict real-header Cortex-M0+ compilation pass. No hardware flash or
movement performed. Version3 source corrections are local pending publication.


## Version4 bounded encoder batching — 2 November 2026

Operator v3 trace confirms separate powered wheel runs (L7100/R7261 over5s)
and physical STOP results (end3/reason0x1). The sequence was correct. Speed
became invalid when a lifetime coalesced flag latched; readiness remained0/0/1.
The trace lacks batch frequency, so it cannot establish how many edges, if any,
were missed, nor the hardware source of batching.

Per-wheel cumulative simultaneous R/F batch counters now retain history and
wrap like counts. Measurement uses deltas within each speed window, not the
lifetime flag. Strict CAR_RUN/default configurations reject new batches; the
unqualified movement-test candidate permits <=2 batches/100ms AND <=5% of
observed edges in batches, independently per wheel. These are experimental
quality limits, not a proved lost-count error bound. Sticky history no longer
poisons future windows or the opposite wheel. Failed windows rebase acquisition
and need a complete subsequent good window; no counters/history are cleared.
Active PID still aborts/inhibits on acquisition/quality failure; no auto restart.

GPIO ISR skips empty groups and edge-mask construction skips unused groups,
reducing work before ACK without changing subscriptions/urgent STOP ordering.
Physical IRQ/STOP timing remains unmeasured. Tests cover sparse versus dense
and excessive-fraction batches, independent wheel limits, impossible batch
counts, counter/time wrap, recovery from gaps/epochs, actual raw task readiness
with historical batching, and active PID abort with invalid odometry status.

124/124 native and19/19 shared suites pass. Full movement and offline CAR_RUN
builds pass. Strict real-header ARM compilation of all changed motor/diagnostic
modules passes. No flash, motor operation or physical PID qualification was
performed. Version3/4 corrections remain local/uncommitted pending publication.


## Version5 distance final-approach candidate — 2 November 2026

Operator case4 result:code7 at287mm encoder-estimated average. Targets reduced
to20/30mm/s, both counts stopped while nonzero effort remained, measurement
valid1 and coalesced_ever0. Consistent with low-speed rolling deadband/no-progress
failure. Case5 correctly refused; case4 had not completed (SHORT0).

Added optional linear_goal_min_speed_mm_s configuration, zero by default. It
floors only forward/reverse goal slowdown and is bounded by the goal request's
own max speed. Pivots retain their profile. V5 test candidate sets60mm/s and
5mm per-wheel settle tolerance instead of2mm; physical values remain untuned.
Normal CAR_RUN retains zero floor and its previous tolerance.10mm overshoot,
700ms no-progress,100ms watchdog, finite deadlines and30% diagnostic effort cap
remain. Diagnostic result logs now show both wheel travel values and target.

130/130 native tests and19/19 shared suites pass. Six new synthetic deadband/
coast plant cases compare the old unfinished goal (timeout near292mm in this
model) with candidate completion near298mm, forward/reverse, additional coast,
blocked-wheel fault and a40mm/s request below the candidate floor. This is not
an identification of the physical car's dynamics or proof of distance accuracy.
Source corrections remain local/uncommitted pending publication; no flash or
motor operation performed.


## Version6 independent cases and pivot approach — 2 November 2026

Removed L/R/STOP/SHORT prerequisite gates from movement diagnostic admission.
All0..9 may be selected/run directly after fresh boot, one at a time, with a
fresh manual gesture. Prior check flags are informational. Held/pending STOP,
latched fault, motor permit/inhibit, acquisition, watchdog and finite limits
still apply. After a diagnosed fault/reset, no preceding tests need repeating.

Added optional pivot_goal_min_speed_mm_s (zero default), test candidate60mm/s,
separate from the retained linear60mm/s floor and capped by request speed.
Ruler track112mm is unchanged: the operator's v5 pivot faulted short of its
encoder target, so no track-width compensation was applied. Goal tolerance5mm,
overshoot10mm, progress700ms, watchdog100ms and30% cap retained.

145/145 native tests and19/19 shared suites pass. New fresh-boot scenarios cover
all ten independent cases, held STOP and fault refusal. Synthetic pivot left/
right completion and prior-profile shortfall compare deadband/coast dynamics;
lower pivot speed cap is checked. These simulations are not measured body
angle or real turn qualification. Source remains local/uncommitted; no flash,
motor operation or push performed.
