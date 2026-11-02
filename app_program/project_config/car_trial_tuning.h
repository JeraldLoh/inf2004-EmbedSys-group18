#ifndef GROUP18_CAR_TRIAL_TUNING_H
#define GROUP18_CAR_TRIAL_TUNING_H

/* Local floor-tuning candidate only; production and standalone B2 unchanged.
 * With 112 mm track and whole-1/m curvature rounding, full correction at
 * 50 mm/s requests about 28/72 mm/s at sustained full correction (125 mm
 * nominal radius), while initial correction still requests about 47/53.
 * Below the previous 60 mm/s rolling candidate: floor stall needs checking.
 * Two digital sensors still need physical track validation. */
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
#define CONTROLLER_LINE_CRUISE_MM_S 50U
#define CONTROLLER_LINE_CURVATURE_PER_STEER 120000
#define LINE_STEER_BASE 12
#define LINE_STEER_RAMP_MS 10U
#define LINE_STEER_MAX 70
#define LINE_HOLD_STEER 0
#define LINE_DAMP_STEER 0
#endif

#endif
