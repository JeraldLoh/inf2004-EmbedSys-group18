# Decision Register

| ID | Decision | Status |
|---|---|---|
| D1 | Servo scan primary; healthy-sonar and calibrated-motion body scan fallback | Proposed |
| D2 | Missing route instruction stops only at a confirmed decision junction | Proposed |
| D3 | Contracts, task boundaries, and folder tree are the framework baseline | Proposed |
| D4 | Buddy 5 leads integration; controller remains shared and separate | Agreed baseline |
| D5 | Defaults are not measurements; calibration values require evidence | Proposed |
| D6 | G0-G7 are readiness gates; safe independent work may proceed in parallel | Proposed |
| D7 | Use official course-template root and subsystem1-5 folders, with buddy-labelled guides | Applied structural migration, team review pending |
| D8 | Keep shared controller outside all five subsystems, under app_program/vehicle_controller | Applied layout; dedicated task unimplemented |
| D9 | Built-in service owns raw CYW43/lwIP/MQTT; B1 provides bounded application bridge | Verified template constraint; bridge unimplemented |
| D10 | Keep GP0/GP1 for line sensors; USB-only console, no crash mirroring while attached | Proposed car profile; target verification pending |

G0 source identification is now addressed by the official template at commit
`c3b09544cb1fd6dac6ad4b8f523be0af59b44482`; runtime qualification and G1 remain open.
No target build/flash or physical test occurred. Application priority/core/object
allocation remains unresolved rather than copying the template's demo bands.
