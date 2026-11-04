#ifndef GROUP18_S4_CONFIG_H
#define GROUP18_S4_CONFIG_H

/*
 * Buddy 4 tuning defaults.
 *
 * EVERY VALUE BELOW IS AN UNCALIBRATED STARTING POINT, not a measurement.
 * Replace each one from testing/subsystem4 evidence and record the test ID
 * next to it. Once agreed, these may move to app_program/project_config/
 * with the other vehicle tuning.
 *
 * Units: ddeg = 0.1 degree, mg = milli-g, ms = milliseconds.
 */

/* --- Kernel scheduling (provisional; TEAM.md section 4 is still TBD) ---- */
#define S4_PERIOD_MS 20U          /* 50 Hz acquisition and estimation */
#define S4_TASK_PRIORITY 12       /* below kernel services 1-4 */
#define S4_TASK_STACK_BYTES 2048U /* tm_printf needs about 1 KB */
#define S4_EVENT_QUEUE_DEPTH 8U   /* 4-byte events held for the controller */
#define S4_LOG_EVENTS 1           /* print hump/collision/cal events */

/* --- Sensor (GY-511 / LSM303DLHC on I2C1, GP2 SDA / GP3 SCL) ------------ */
#define S4_I2C_DEVICE "iicb"      /* course driver: unit 1 = I2C1 */
#define S4_ACCEL_ADDR 0x19U       /* SA0 pulled high on the GY-511 */
#define S4_MAG_ADDR 0x1EU
#define S4_ACCEL_FS_CODE 1U       /* +/-4 g: 2 mg/LSB, room for 3 g shocks */
#define S4_MAX_BUS_ERRORS 3U      /* consecutive errors before re-open */
#define S4_REOPEN_TICKS 10U       /* retry the bus every 200 ms when faulted */

/*
 * Mounting: which sensor axis points RIGHT, FORWARD and UP on the car.
 * IMU_AXIS_CODE(axis, negate) with axis 0=X, 1=Y, 2=Z.
 * Default matches the briefing figure: X right, Y forward, Z up.
 * VERIFY ON THE CAR: tilt the nose up and check pitch goes positive.
 */
#define S4_AXIS_RIGHT IMU_AXIS_CODE(0U, 0)
#define S4_AXIS_FWD IMU_AXIS_CODE(1U, 0)
#define S4_AXIS_UP IMU_AXIS_CODE(2U, 0)

/* --- Filtering ----------------------------------------------------------- */
#define S4_LPF_SHIFT 2U           /* fast EMA alpha = 1/4 (~80 ms) */
#define S4_SLOW_EXTRA_SHIFT 3U    /* slow EMA alpha = 1/32 (~640 ms) */
#define S4_PITCH_SMOOTH_SHIFT 3U  /* pitch EMA 1/8 (~140 ms) for terrain states */

/* --- Level calibration (car still on a level floor) ---------------------- */
#define S4_CAL_SAMPLES 64U        /* 1.28 s at 50 Hz */
#define S4_CAL_MAX_SPREAD_MG 40U  /* per-axis max-min allowed while "still" */
#define S4_CAL_MAX_RETRIES 5U

/* --- Terrain / hump state machine ---------------------------------------- */
/*
 * The car is two drive wheels plus a caster. With the caster and at least
 * one wheel on the floor, the height of the highest raised wheel (or of the
 * caster) follows from the attitude alone:
 *
 *   raised = caster_arm x sin|pitch| + (track / 2) x sin|roll|
 *
 * Demo ramp: about 10 mm high, 195 mm long, under the RIGHT wheel only.
 * That rolls the car 5.1 deg (10 / 112 mm track) and pitches it only
 * ~2.2 deg, with the same sign all the way across, so a pitch-only state
 * machine cannot see it. Both halves give 5 mm, so the formula reads 10 mm.
 * A full-width hump gives caster_arm x sin(pitch) + 0 = h as well.
 * The terrain states follow this raised height in mm.
 */
#define S4_RAISE_ON_MM 4U         /* raised wheel above this starts a climb */
#define S4_RAISE_OFF_MM 2U        /* below this the car is back on the floor */
#define S4_LEVEL_DDEG 10U         /* |pitch| below this counts as level (distance) */
#define S4_DIP_DDEG 50U           /* anti-climb pitch that starts an episode from
                                     LEVEL (braking reads nose-down) */
/*
 * MEASURE BOTH on the car and replace these placeholders (ruler, mm):
 *   S4_CASTER_ARM_MM  drive axle centre to the caster's floor contact,
 *                     along the car. Error moves the height by half as
 *                     much on the one-sided ramp (10% -> 0.5 mm).
 *   S4_TRACK_MM       left tyre centre to right tyre centre. 112 is Buddy 2's
 *                     MOTION_TRACK_WIDTH_MM.
 *   S4_CLIMB_NOSE_DOWN 0 when the caster is BEHIND the drive wheels (wheels
 *                     climbing tilts the nose up), 1 when it is in front.
 */
#define S4_CASTER_ARM_MM 130U
#define S4_TRACK_MM 112U
#define S4_CLIMB_NOSE_DOWN 0U
#define S4_CONFIRM_SAMPLES 3U     /* consecutive samples to change state */
#define S4_SETTLE_MS 300U         /* on the floor this long after a descent */
#define S4_EPISODE_TIMEOUT_MS 8000U
#define S4_LAUNCH_HOLDOFF_MS 500U  /* moving off reads nose-up: pitch cannot
                                      start a climb this long after rest */
#define S4_MIN_HUMP_MM 5U
#define S4_MIN_HUMP_LENGTH_MM 80U
#define S4_MAX_HUMP_LENGTH_MM 1500U
#define S4_BASE_UNCERTAINTY_MM 3U
#define S4_FALLBACK_SPEED_MM_S 60U  /* distance estimate without odometry:
                                       the CAR_RUN line cruise speed */
#define S4_ODOM_STALE_TICKS 10U     /* odometry older than 200 ms is unused */

/* --- Persistent failures and calibration retry --------------------------- */
#define S4_FAULT_PERSIST_MS 1000U   /* evidence this long = latched failure */
#define S4_CAL_RETRY_MS 2000U       /* wait before retrying a failed level cal */

/* --- Speed caps sent to the controller (0 = no cap) ---------------------- */
/* Below the controller's 150 mm/s cruise so the cap takes effect on a hump. */
#define S4_HUMP_CAP_MM_S 100U
#define S4_DEGRADED_CAP_MM_S 100U   /* unknown/invalid terrain, tilt, impact */
#define S4_TILT_LIMIT_DDEG 250U

/* --- Motion classification and collision --------------------------------- */
#define S4_STILL_VIB_MG 12U
#define S4_ROUGH_VIB_MG 150U
#define S4_LONG_ACCEL_MG 60U
#define S4_COLLISION_MG 1000U       /* horizontal jolt that counts as a hit */
#define S4_COLLISION_HOLD_MS 500U
#define S4_STALE_MS 200U

/* --- Turn detection (magnetometer heading rate) -------------------------- */
#define S4_TURN_ON_DDPS 200U        /* 20 deg/s starts a turn */
#define S4_TURN_OFF_DDPS 80U        /* below 8 deg/s ends it */
#define S4_MAG_CAL_MIN_SPREAD 150U  /* raw LSB span needed on X and Y */

#endif
