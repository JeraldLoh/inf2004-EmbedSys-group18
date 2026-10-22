# Readiness Gates

## G0 - Platform

- verified source: official project template commit
  `c3b09544cb1fd6dac6ad4b8f523be0af59b44482`;
- identified startup: `usermain()`, all objects before any task starts;
- identified backend: built-in CYW43/lwIP/MQTT service owner;
- identified build discovery: app root plus one subfolder level;
- still qualify WSL/toolchain/SDK/submodules and the actual target build;
- still implement controller/subsystem lifecycle and failure rollback;
- identify supported task and ISR-context primitives;
- identify timer reservations, timeout units, and priority rules;
- verify application/service bridge details and required approval;
- agree core affinity, SMP-safe locking, priorities, stacks and object budget;
- adapt platform and build files without changing contract semantics.

G0 is partially addressed by source inspection, NOT closed. No target build or
hardware execution was used for this migration. See the course-template review.

## G1 - Electrical safety

- trace the pin/Grove harness against `app_program/project_config/pinmap.h`;
- verify common ground and absence of driven-output conflicts;
- measure HC-SR04 echo protection (voltage divider) at GP17;
- confirm GP26 (barcode, Grove 5 and 6) and GP27 (left encoder, Grove 6) are not
  wired to two devices at once;
- measure IR supplies and maximum D0/A0 voltages;
- document battery/USB arrangement and motor/servo rail behaviour;
- verify safe motor outputs during boot and reset.

## G2-G7

- G2: encoders, motion calibration, cancellation, and motion faults;
- G3: IR mounting, line tracking, and barcode acquisition;
- G4: junction confirmation, command lifecycle, and stable exit;
- G5: terrain limiting, hump detection, and height validity;
- G6: scanning, profiling, bypass, and original-line recovery;
- G7: networking, overload/fault tests, and integrated autonomous mission.

Independent work may proceed in parallel, but integration cannot claim a gate
whose entry conditions or evidence are incomplete.
