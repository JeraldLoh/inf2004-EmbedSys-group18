#ifndef GROUP18_CAR_TRIAL_TUNING_H
#define GROUP18_CAR_TRIAL_TUNING_H

/* Local floor-tuning candidate only; production and standalone B2 unchanged.
 * With 112 mm track and whole-1/m curvature rounding, full correction at
 * 50 mm/s base plus an outside-wheel boost requests about19/96mm/s at
 * sustained full correction (84mm nominal radius). Straight remains50/50.
 * The inner wheel is below the previous 60 mm/s rolling candidate.
 * Two digital sensors still need physical track validation. */
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
#define CONTROLLER_LINE_CRUISE_MM_S 50U
#define CONTROLLER_LINE_CURVATURE_PER_STEER 120000
/* Add half the original turn speed offset to the outside wheel only.
 * Applied after base cap scaling, clipped to the existing wheel-speed cap. */
#define CAR_TRIAL_LINE_OUTER_BOOST_PERCENT 50U
#define LINE_STEER_BASE 12
#define LINE_STEER_RAMP_MS 6U
#define LINE_STEER_MAX 90
#define LINE_CURVE_MEMORY_MS 200U
#define LINE_HOLD_STEER 0
#define LINE_DAMP_STEER 0
#endif

#endif
