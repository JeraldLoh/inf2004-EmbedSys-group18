#ifndef GROUP18_CAR_TRIAL_TUNING_H
#define GROUP18_CAR_TRIAL_TUNING_H

/* Bounded controller/B2 adapter trial configuration. No Buddy3 steering
 * definitions belong here. Buddy3 source and build tuning are restored to
 * testing641ad74. Production and standalone B2 remain unchanged. */
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
#define CONTROLLER_LINE_CRUISE_MM_S 50U
#define CONTROLLER_LINE_CURVATURE_PER_STEER 120000
#endif

#endif
