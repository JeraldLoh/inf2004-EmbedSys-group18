#ifndef GROUP18_TERRAIN_ESTIMATOR_H
#define GROUP18_TERRAIN_ESTIMATOR_H

/*
 * Buddy 4 terrain, hump, motion and turn estimator.
 *
 * One call to s4_est_step() per sample period turns a raw accelerometer and
 * magnetometer sample (plus optional Buddy 2 odometry) into:
 *   - level calibration (mounting pitch/roll offsets, gravity magnitude);
 *   - pitch, roll, tilt-compensated heading and turn rate;
 *   - terrain state LEVEL/ASCENDING/CREST/DESCENDING/SETTLING with a speed cap;
 *   - hump peak height = integral of sin(pitch) over travelled distance, with
 *     uncertainty and validity;
 *   - motion class, longitudinal acceleration/braking, collision detection;
 *   - completed-turn events.
 * Results are kept in the four packed words of s4_packed.h and discrete
 * events are returned as 4-byte messages.
 *
 * Portable and single-threaded: one task owns an s4_est_t. No kernel calls.
 */
#include "imu_math.h"
#include "s4_packed.h"

/* Task-private classifications, combined into S4_MC_* when packed. */
enum
{
    S4_MOTION_STATIONARY = 0,
    S4_MOTION_MOVING = 1,
    S4_MOTION_ROUGH = 2,
    S4_MOTION_IMPACT = 3
};

enum
{
    S4_LONG_STEADY = 0,
    S4_LONG_ACCELERATING = 1,
    S4_LONG_BRAKING = 2
};

enum
{
    S4_TURN_NONE = 0,
    S4_TURN_LEFT = 1,
    S4_TURN_RIGHT = 2
};

/*
 * Largest magnitudes actually observed, before packing (Week 8 tribunal:
 * "log real turn-rate and hump ranges, then set the bit widths"). Read them
 * after test runs to confirm each packed field is wide enough.
 */
typedef struct
{
    uint16_t turn_rate_dps;    /* max |turn rate| */
    uint16_t pitch_ddeg;       /* max |pitch| */
    uint16_t roll_ddeg;        /* max |roll| */
    uint16_t hump_height_mm;   /* max completed-hump peak, unclamped */
    uint16_t hump_length_mm;   /* max completed-hump path, unclamped */
    uint16_t shock_mg;         /* max collision jolt */
    uint16_t vibration_mg;     /* max vibration level */
    uint16_t clamp_count;      /* published messages with CLAMPED set */
} s4_range_log_t;

typedef struct
{
    /* Small fields packed into one word (Change Requirement 1). */
    unsigned int axis_right : 3;        /* IMU_AXIS_CODE */
    unsigned int axis_fwd : 3;
    unsigned int axis_up : 3;
    unsigned int accel_fs : 2;          /* IMU_ACCEL_FS_* */
    unsigned int lpf_shift : 3;         /* 1..6 */
    unsigned int slow_extra_shift : 3;  /* 1..6 */
    unsigned int confirm_samples : 4;   /* 1..15 */
    unsigned int cal_max_retries : 3;   /* 1..7 */
    unsigned int cal_samples : 8;       /* 8..255 */
    unsigned int climb_nose_down : 1;   /* 1 = caster in front of the drive axle */

    uint8_t level_ddeg;
    uint8_t dip_ddeg;          /* anti-climb pitch from LEVEL that starts an episode */
    uint8_t raise_on_mm;       /* raised wheel height that starts a climb */
    uint8_t raise_off_mm;      /* below this the car is back on the floor */
    uint8_t min_hump_mm;
    uint8_t base_uncertainty_mm;
    uint8_t cal_max_spread_mg;
    uint8_t still_vib_mg;
    uint8_t long_accel_mg;

    uint16_t settle_ms;
    uint16_t episode_timeout_ms;
    uint16_t launch_holdoff_ms;  /* after moving off, pitch cannot start a climb */
    uint16_t min_length_mm;
    uint16_t max_length_mm;
    uint16_t hump_cap_mm_s;
    uint16_t degraded_cap_mm_s;
    uint16_t tilt_limit_ddeg;
    uint16_t caster_arm_mm;    /* drive axle to caster contact, along the car */
    uint16_t track_mm;         /* left to right drive wheel contact */
    uint16_t rough_vib_mg;
    uint16_t collision_mg;
    uint16_t collision_hold_ms;
    uint16_t stale_ms;
    uint16_t fallback_speed_mm_s;
    uint16_t turn_on_ddps;
    uint16_t turn_off_ddps;
    uint16_t mag_cal_min_spread;
} s4_cfg_t;

typedef struct
{
    int16_t accel_mg[3];      /* sensor frame, decoded */
    int16_t mag[3];           /* sensor frame, decoded */
    int16_t odom_delta_mm;    /* path distance since last step, signed */
    uint16_t dt_ms;           /* measured time since last step */
    unsigned int accel_ok : 1;
    unsigned int accel_saturated : 1;
    unsigned int bus_fault : 1;
    unsigned int mag_ok : 1;
    unsigned int odom_valid : 1;
} s4_input_t;

#define S4_MAX_EVENTS 6U

/* Why the last level calibration failed (s4_est_t.cal_fail_reason). */
enum
{
    S4_CALFAIL_NONE = 0,
    S4_CALFAIL_MOVING = 1,    /* an axis varied more than cal_max_spread_mg */
    S4_CALFAIL_GRAVITY = 2,   /* |mean| outside 700..1300 mg */
    S4_CALFAIL_UPSIDE = 3     /* the UP axis does not read +1 g */
};

typedef struct
{
    uint8_t count;
    s4_event_msg_t msg[S4_MAX_EVENTS];
} s4_events_t;

typedef struct
{
    s4_cfg_t cfg;

    /* Vehicle-frame filters, mg scaled by 16. */
    int32_t acc_fast[3];
    int32_t acc_slow[3];
    int32_t mag_filt[3];
    int32_t vib_x16;

    /* Level calibration. */
    int32_t cal_sum[3];
    int16_t cal_min[3];
    int16_t cal_max[3];
    int16_t pitch0_ddeg;
    int16_t roll0_ddeg;
    uint16_t g0_mg;
    /* Evidence from the last failed level-calibration window (diagnostics). */
    int16_t cal_fail_mean[3];  /* vehicle frame: right, forward, up (mg) */
    uint16_t cal_fail_spread;  /* largest per-axis max-min (mg) */

    /* Magnetometer hard-iron calibration, sensor frame raw LSB. */
    int16_t mag_off[3];
    int16_t mag_min[3];
    int16_t mag_max[3];

    /* Attitude and turning. */
    int16_t pitch_ddeg;
    int32_t pitch_smooth_x16;  /* pitch EMA (ddeg x16): terrain states, peak */
    int32_t roll_smooth_x16;   /* roll EMA (ddeg x16): terrain states, peak */
    int16_t roll_ddeg;
    uint16_t heading_ddeg;
    int32_t turn_ddps;
    int32_t turn_acc_ddeg;

    /* Current hump episode. */
    int32_t height_um;
    int32_t peak_um;
    uint32_t path_um;
    int16_t peak_pitch_ddeg;
    int32_t peak_raise_um;     /* highest raised wheel seen this episode */

    /* Saturating millisecond timers. */
    uint16_t state_ms;
    uint16_t episode_ms;
    uint16_t no_sample_ms;
    uint16_t impact_ms;
    uint16_t moving_ms;          /* time since the car last stood still */
    uint16_t still_ms;           /* time standing still */
    uint16_t shock_mg;

    /* Task-private state flags as C bit-fields. */
    unsigned int terrain : 3;           /* terrain_state_t */
    unsigned int pending : 3;           /* candidate next state */
    unsigned int confirm : 4;           /* samples the candidate has held */
    unsigned int motion : 2;            /* S4_MOTION_* */
    unsigned int longitudinal : 2;      /* S4_LONG_* */
    unsigned int faults : 7;            /* S4_FAULT_* */
    unsigned int calibrated : 1;
    unsigned int cal_active : 1;
    unsigned int cal_retries : 3;
    unsigned int cal_fail_reason : 2;   /* S4_CALFAIL_* of the last failure */
    unsigned int mag_cal_active : 1;
    unsigned int mag_calibrated : 1;
    unsigned int heading_valid : 1;
    unsigned int filters_primed : 1;
    unsigned int pitch_smooth_primed : 1;
    unsigned int mag_primed : 1;
    unsigned int turn_state : 2;        /* S4_TURN_* */
    unsigned int turn_quiet : 4;
    unsigned int in_episode : 1;
    unsigned int saw_descent : 1;
    unsigned int used_fallback : 1;
    unsigned int dist_odometry : 1;
    unsigned int collision_count : 3;
    unsigned int collision_dir : 2;     /* S4_HIT_* */
    unsigned int hump_seq : 3;
    unsigned int sample_seq : 5;
    unsigned int cal_count : 8;

    s4_range_log_t ranges;

    /* Packed outputs, see s4_packed.h. */
    uint32_t words[S4_WORDS];
} s4_est_t;

void s4_est_default_config(s4_cfg_t *cfg);
bool s4_est_config_valid(const s4_cfg_t *cfg);

/* Returns false (and leaves est untouched) if cfg is invalid. */
bool s4_est_init(s4_est_t *est, const s4_cfg_t *cfg);

/* Car must be still on a level floor until the CAL event arrives. */
void s4_est_begin_calibration(s4_est_t *est);

/* Clear latched conditions (currently the collision bit). */
void s4_est_clear_latched(s4_est_t *est);

/* Start: spin the car slowly through 360 deg. Stop: apply hard-iron offsets. */
void s4_est_mag_calibration(s4_est_t *est, bool start, s4_events_t *events);

/* Restart the observed-range log. */
void s4_est_reset_ranges(s4_est_t *est);

/* Process one sample. events->count is reset, then filled. */
void s4_est_step(s4_est_t *est, const s4_input_t *in, s4_events_t *events);

#endif
