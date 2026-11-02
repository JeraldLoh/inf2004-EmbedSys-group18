#ifndef GROUP18_CAR_TRIAL_TUNING_H
#define GROUP18_CAR_TRIAL_TUNING_H

/* Controller/B2 adapter trial settings only. Buddy3 source and steering build
 * settings are restored to testing641ad74. Local unpublished B2 V10/V11
 * turn experiments are retained; no LINE_* perception overrides belong here. */
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
#define CONTROLLER_LINE_CRUISE_MM_S 60U  /* was 70 (operator, 2026-11-02) */
#define CONTROLLER_LINE_CURVATURE_PER_STEER 140000
#define CAR_TRIAL_LINE_INNER_SPEED_PERCENT 85U
#endif

#endif
