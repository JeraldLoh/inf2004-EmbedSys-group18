# Readiness Gates

## G0 - Platform

- verify the professor's micro T-Kernel template and version;
- identify the application startup hook and task-creation ownership;
- identify supported task and ISR-context primitives;
- identify timer reservations, timeout units, and priority rules;
- identify the CYW43/lwIP backend and callback context;
- adapt platform and build files without changing contract semantics.

## G1 - Electrical safety

- trace the Week 6 pin/Grove harness;
- verify common ground and absence of driven-output conflicts;
- measure HC-SR04 echo protection at GP7;
- verify the GP28 VBAT-sense jumper is isolated;
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
