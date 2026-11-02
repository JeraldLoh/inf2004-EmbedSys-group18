# Buddy 2 movement-test v6 — 2 November 2026

All cases0..9 are independent. You may select any case after boot without
completing earlier cases. Only one test runs at a time; every run needs a fresh
manual gesture. No boot motion, automatic sequence or queued movement.

## Flash and controls

1. Motor power OFF, disconnect USB. Hold BOOTSEL while reconnecting USB and copy
   the v6 UF2 to RPI-RP2. Open the Pico's current COM port and confirm
   `[B2 MOVEMENT TEST v6]`.
2. Raise and secure the wheels for raw/speed/watchdog checks, then enable motor
   power. Use a clear level floor for physical distance/angle measurements.
3. Tap GP20 to select0..9 (wraps). Hold >=1s until RELEASE, then release to run
   that case ONCE. Holding at boot/through another test cannot queue a run.
4. GP21 STOP interrupts movement. Releasing it never restarts. Release STOP,
   then make a fresh GP20 hold/release to start another test.

## Cases

| Case | Action | Limit/location |
|---|---|---|
|0|Left forward only|30%,5s; RAISED|
|1|Right forward only|30%,5s; RAISED|
|2|Both forward; interrupt with GP21|30%,up to5s; RAISED|
|3|Straight PID target100mm/s|5s or500mm wheel rotation; first RAISED|
|4|Forward300mm|10s outer limit; clear level floor|
|5|Forward1000mm|15s outer limit; clear level floor|
|6|Reverse300mm|10s outer limit; clear level floor|
|7|Left pivot90degrees|10s outer limit; check wheel directions raised first|
|8|Right pivot90degrees|10s outer limit; check wheel directions raised first|
|9|Watchdog cutoff probe|RAISED;100ms alarm/task withheld150ms|

Case7 can now be selected directly after boot: tap GP20 seven times, confirm
case7, then hold/release. Case8/9 work similarly. A successful30cm trial is
recommended before longer floor movement, but is no longer a software gate.

`checks L/R/STOP/SHORT (info only)` records prior checks and does not block
selection/start. A raw feedback check still reports success only for100edges,
recent motion in the final500ms and healthy independent acquisition. A twitch
is not evidence of healthy movement. B-only encoders cannot prove direction;
visually check which way each positive command turns its wheel.

Faults remain latched. After a timeout, stall, acquisition failure or other
motion fault, diagnose then reset. After reset, jump straight to the desired
case; no prerequisite tests need repeating. A fault is not automatically
cleared by selecting another case. Physical STOP, driver inhibit, watchdog,
quality/rate checks, finite deadlines and overshoot protection remain active.

## Goal approach and calibration

V5 completed30cm forward/reverse and1m in operator logs; the operator measured
30cm correctly. The v5 left pivot stopped short and faulted:67/74mm wheel
travel versus87mm target, estimated72.6deg, visible75-80deg. Targets had fallen
to27-40mm/s with stopped wheels. Do not compensate track width for that
unfinished turn.

V6 keeps the linear60mm/s rolling floor and adds a separate pivot60mm/s floor,
only until each wheel reaches its5mm tolerance band. The goal request's lower
speed cap always wins. Each wheel is commanded zero in band and must settle;
10mm overshoot cutoff,700ms no-progress and100ms watchdog remain. These are
untuned test candidates. No guarantee of90deg body angle; measure the actual
angle once the result completes, then tune effective track width if needed.

Retained wheel diameter65mm,1271left/1264right B-channel BOTH-edge counts per
wheel revolution, ruler track112mm. No gearbox multiplier. Candidate PID gains
x1000: P/I/D1500/1500/0 on each wheel, FF2163left/2689right, effort cap30%, launch
30% for<=500ms. Production-qualified stays FALSE. Normal CAR_RUN retains zero
linear/pivot rolling floors, strict batching policy and unqualified gains.

## Logs and encoder quality

Live logs show owner RAW/PID/IDLE, targets, measured speed magnitudes, validity,
efforts (300=30%), raw counts, historical coalescing and batch counters. Raw/PID
logs run at approximately100ms. A passive observer monitors raw/idle and never
writes PWM/watchdogs; only the active PID owner steps the measured executor.

The test candidate accepts <=2 simultaneous rising/falling batches per100ms
AND <=5% observed edges in batches, independently per wheel. `coalesced_ever`
is historical and does not alone fail a new test. Both latched polarities are
counted; this policy does not prove additional same-polarity edges were never
lost. Excessive batching, invalid rate/gap/epoch/acquisition aborts active PID.
The passive observer needs a complete good window to recover after a failure.

Raw `end`:2=time ended,3=STOP/fault-reason stop,4=driver failure. Inspect reasons
and feedback_ok; time-ended alone is not successful movement. Long raw runs
refresh a100ms hardware cutoff; missing service stops them early as a failure.
Normal bench/circuit pulse caps remain unchanged.

Goal `RESULT code`:3=encoder completion,4=cancelled,5=timeout,6=aborted,7=fault.
Per-wheel travel and target are printed. `mm`/`mdeg` derive from wheel rotation
and commanded direction, not independent floor position/body angle. Even code3
needs external distance/angle measurement to establish physical accuracy.

Case9 reports `WATCHDOG cutoff_observed=1` only for timer evidence plus disabled
software driver state without GP21. This is an IRQ/software functional check,
not physical pin timing or debugger-halt qualification. Physical STOP<=50ms
still needs instrumentation. Do not obstruct powered wheels with your hands.

## Software evidence and build

145 native B2/circuit/movement CTests and19 shared suites pass. New coverage
starts every case0..9 from fresh boot without prior checks, held STOP refusal,
fault-latched refusal, pivot left/right deadband/coast completion versus legacy
shortfall, and a lower pivot request speed cap. Simulated dynamics are not car
calibration. Existing encoder quality, STOP race, stall, overshoot, deadline,
result and startup tests remain.

From build_make with Pico SDK2.3.1 and the course ARM toolchain:

```sh
make -j4 SMP=1 CONSOLE=usb_cdc CRASH_REPORT=0 WIFI=none NET=none MQTT=0 TEST=0 B2_BENCH=0 B5_RUN=0 CAR_RUN=0 CIRCUIT_TEST=0 B2_MOVE_TEST=1
```

No flashing or physical motor operation was performed while creating this
image. It does not start navigation, network, sonar, barcode, IMU or servo tasks.
