# Integration test plan

All cases below are proposed and Not run. Define quantitative limits before
execution. Each case spans at least two buddies; the shared controller participates
where applicable. No target runtime adapters exist yet.

| ID | Buddies | Case | Acceptance and evidence to define |
|---|---|---|---|
| INT-10 | 2, 5 | Obstacle stop during active motion | Event-to-output response, stopping distance, no stale restart; timed log/video |
| INT-11 | 2, 3 | Steering plus line-loss/recovery | Bounded correction/recovery and confirmed stable tracking; raw inputs/log/video |
| INT-12 | 1, 2 | Communications overload/reconnect during wheel control | Agreed motor deadline/jitter under load and bounded queues; timing/overflow logs |
| INT-13 | 2, 4 | Terrain cap during movement | Agreed speed restriction and validated restoration condition; speed/terrain log |
| INT-14 | 1, 3, 5 | Shared ingress/event overload | Critical-event/STOP delivery independent of ordinary queue capacity; fault-injection log |
| INT-20..29 | 2, 3, 5, controller | Whole-car build (`CAR_RUN=1`) | Step-by-step plan with pass criteria: `CAR_RUN_TEST_PLAN.md` |

## Actual results

| Date | Test ID | Commit/profile | Owners | Result | Evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-28 | INT-H01 host: executor core, controller line tracking + gate, B5 motion | branch integration-drive, testing/host CTest, zig cc | Buddy 5 | PASS, 9/9 host suites; Buddy 2 suite 38/38; all 6 make profiles build and link, 0 app warnings | ctest + make output | Host and build only. Mutation-checked. Not flashed, no hardware run. |
