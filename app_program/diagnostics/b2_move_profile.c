#include "b2_move_profile.h"
#if (defined(APP_B2_MOVE_TEST) && APP_B2_MOVE_TEST) || (defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL)
#include "../subsystem2/car_motion.h"
void b2_move_candidate_profile(measured_motion_executor_config_t *c)
{
    car_motion_candidate_config(c);
    /* v6 values. Prior branch notes reported effort199 at ~92/74mm/s;
     * 199/92 and 199/74 provide rough FF. Standalone calibration remains bounded to 30%. The car image
     * (CAR_RUN, CAR_B2_TRIAL defaults on) runs this profile qualified after
     * Buddy 2's sign-off on 2026-11-03 (subsystem2_motion.c); it stays
     * unqualified here for B2_MOVE_TEST and the B2 tests. */
    c->pid[0]=(wheel_pid_config_t){1500U,1500U,0U,2163U,300U,100U,300U,500000U};
    c->pid[1]=(wheel_pid_config_t){1500U,1500U,0U,2689U,300U,100U,300U,500000U};
    c->max_speed_mm_s=120U; c->max_goal_mm=1000U;
    c->acceleration_um_s2=200000U;
    /* Candidate from v4's stall at287/300mm: avoid20-30mm/s crawl.
     * Stop/settle within5mm per wheel; retain10mm overshoot cutoff. */
    c->linear_goal_min_speed_mm_s=60U;
    c->pivot_goal_min_speed_mm_s=60U; /* V5 pivots stalled near40mm/s; needs physical retest. */
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
    /* Operator request: car wheel-output cap 40%; standalone calibration stays 30%. */
    c->pid[0].max_effort=400U;
    c->pid[1].max_effort=400U;
    /* Pivots get 40% (operator, 2026-11-03): on a table run the right wheel
     * stalled at 268/300 on the bypass return pivot (NO_WHEEL_PROGRESS).
     * Straight driving and line following also use a 40% PID cap. */
    c->pivot_max_effort=400U;
    /* Straight goals: the lagging wheel stops with the first within 30 mm. */
    c->linear_sync_slack_um=30000U;
    /* Pivots: 20 mm (at most ~10 deg short on a 60 deg pivot), half the
     * wheel distance at most, so a 10 deg junction probe still turns. */
    c->pivot_sync_slack_um=20000U;
    /* Table run 2026-11-03: the right wheel ran ~5 mm/s faster on straight
     * goals and the car curved left. The wheel ahead slows 2 mm/s per mm. */
    c->linear_hold_per_s=2U;
#endif
    c->tolerance_um=5000U; c->overshoot_um=10000U;
    c->settle_us=200000U; c->settle_speed_um_s=3000U;
    c->progress_timeout_us=700000U; c->startup_grace_us=1500000U;
    /* Experimental test-only batching envelope; production remains strict. */
    c->measurement.max_batched_edge_permille=50U;
    c->measurement.max_batches_per_100ms=2U;
    c->qualified=false;
}
#endif
