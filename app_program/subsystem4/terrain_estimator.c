#include "terrain_estimator.h"

#include <stddef.h>
#include <string.h>

#include "s4_config.h"

/* Filters keep 4 fractional bits so small alphas do not stall. */
#define FX 4U

static int32_t asr32(int32_t value, unsigned shift)
{
    /* Arithmetic (floor) shift without relying on implementation behaviour. */
    return (value >= 0) ? (value >> shift) : ~((~value) >> shift);
}

static int32_t abs32(int32_t value)
{
    return (value < 0) ? -value : value;
}

static uint32_t min_u32(uint32_t value, uint32_t limit)
{
    return (value < limit) ? value : limit;
}

static int32_t clamp32(int32_t value, int32_t low, int32_t high)
{
    return (value < low) ? low : ((value > high) ? high : value);
}

static uint16_t add_sat_u16(uint16_t value, uint16_t add)
{
    uint32_t sum = (uint32_t)value + add;
    return (sum > UINT16_MAX) ? UINT16_MAX : (uint16_t)sum;
}

static void log_max(uint16_t *slot, uint32_t value)
{
    if (value > *slot)
    {
        *slot = (uint16_t)min_u32(value, UINT16_MAX);
    }
}

static int32_t round_div32(int32_t numerator, int32_t denominator)
{
    return (numerator >= 0)
        ? ((numerator + (denominator / 2)) / denominator)
        : ((numerator - (denominator / 2)) / denominator);
}

static void push_event(s4_events_t *events, s4_event_msg_t msg)
{
    if ((events != NULL) && (events->count < S4_MAX_EVENTS))
    {
        events->msg[events->count] = msg;
        events->count++;
    }
}

static s4_event_msg_t event_header(uint32_t type)
{
    return s4_put(0U, S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH, type);
}

void s4_est_default_config(s4_cfg_t *cfg)
{
    if (cfg == NULL)
    {
        return;
    }
    (void)memset(cfg, 0, sizeof(*cfg));
    cfg->axis_right = S4_AXIS_RIGHT;
    cfg->axis_fwd = S4_AXIS_FWD;
    cfg->axis_up = S4_AXIS_UP;
    cfg->accel_fs = S4_ACCEL_FS_CODE;
    cfg->lpf_shift = S4_LPF_SHIFT;
    cfg->slow_extra_shift = S4_SLOW_EXTRA_SHIFT;
    cfg->confirm_samples = S4_CONFIRM_SAMPLES;
    cfg->cal_max_retries = S4_CAL_MAX_RETRIES;
    cfg->cal_samples = S4_CAL_SAMPLES;
    cfg->level_ddeg = S4_LEVEL_DDEG;
    cfg->ascend_ddeg = S4_ASCEND_DDEG;
    cfg->descend_ddeg = S4_DESCEND_DDEG;
    cfg->min_hump_mm = S4_MIN_HUMP_MM;
    cfg->base_uncertainty_mm = S4_BASE_UNCERTAINTY_MM;
    cfg->cal_max_spread_mg = S4_CAL_MAX_SPREAD_MG;
    cfg->still_vib_mg = S4_STILL_VIB_MG;
    cfg->long_accel_mg = S4_LONG_ACCEL_MG;
    cfg->settle_ms = S4_SETTLE_MS;
    cfg->crest_timeout_ms = S4_CREST_TIMEOUT_MS;
    cfg->episode_timeout_ms = S4_EPISODE_TIMEOUT_MS;
    cfg->min_length_mm = S4_MIN_HUMP_LENGTH_MM;
    cfg->max_length_mm = S4_MAX_HUMP_LENGTH_MM;
    cfg->hump_cap_mm_s = S4_HUMP_CAP_MM_S;
    cfg->degraded_cap_mm_s = S4_DEGRADED_CAP_MM_S;
    cfg->tilt_limit_ddeg = S4_TILT_LIMIT_DDEG;
    cfg->rough_vib_mg = S4_ROUGH_VIB_MG;
    cfg->collision_mg = S4_COLLISION_MG;
    cfg->collision_hold_ms = S4_COLLISION_HOLD_MS;
    cfg->stale_ms = S4_STALE_MS;
    cfg->fallback_speed_mm_s = S4_FALLBACK_SPEED_MM_S;
    cfg->turn_on_ddps = S4_TURN_ON_DDPS;
    cfg->turn_off_ddps = S4_TURN_OFF_DDPS;
    cfg->mag_cal_min_spread = S4_MAG_CAL_MIN_SPREAD;
}

bool s4_est_config_valid(const s4_cfg_t *cfg)
{
    return (cfg != NULL) &&
        imu_axis_codes_valid(
            (uint8_t)cfg->axis_right,
            (uint8_t)cfg->axis_fwd,
            (uint8_t)cfg->axis_up) &&
        (cfg->lpf_shift >= 1U) && (cfg->lpf_shift <= 6U) &&
        (cfg->slow_extra_shift >= 1U) && (cfg->slow_extra_shift <= 6U) &&
        (cfg->confirm_samples >= 1U) &&
        (cfg->cal_max_retries >= 1U) &&
        (cfg->cal_samples >= 8U) &&
        (cfg->level_ddeg > 0U) &&
        (cfg->level_ddeg < cfg->ascend_ddeg) &&
        (cfg->level_ddeg < cfg->descend_ddeg) &&
        (cfg->min_length_mm < cfg->max_length_mm) &&
        (cfg->settle_ms > 0U) &&
        (cfg->collision_mg > 0U) &&
        (cfg->stale_ms > 0U) &&
        (cfg->turn_off_ddps < cfg->turn_on_ddps);
}

static void reset_cal_window(s4_est_t *est)
{
    unsigned axis;

    est->cal_count = 0U;
    for (axis = 0U; axis < 3U; axis++)
    {
        est->cal_sum[axis] = 0;
        est->cal_min[axis] = INT16_MAX;
        est->cal_max[axis] = INT16_MIN;
    }
}

static void set_terrain(s4_est_t *est, uint8_t state)
{
    est->terrain = state;
    est->pending = state;
    est->confirm = 0U;
    est->state_ms = 0U;
}

static void rebase_height(s4_est_t *est)
{
    est->height_um = 0;
    est->peak_um = 0;
    est->path_um = 0U;
    est->used_fallback = 0U;
}

bool s4_est_init(s4_est_t *est, const s4_cfg_t *cfg)
{
    if ((est == NULL) || !s4_est_config_valid(cfg))
    {
        return false;
    }
    (void)memset(est, 0, sizeof(*est));
    est->cfg = *cfg;
    est->g0_mg = 1000U;
    est->faults = S4_FAULT_UNCAL;
    set_terrain(est, TERRAIN_UNKNOWN);
    reset_cal_window(est);
    return true;
}

void s4_est_begin_calibration(s4_est_t *est)
{
    if (est == NULL)
    {
        return;
    }
    est->cal_active = 1U;
    est->cal_retries = 0U;
    est->in_episode = 0U;
    reset_cal_window(est);
    set_terrain(est, TERRAIN_UNKNOWN);
}

void s4_est_clear_latched(s4_est_t *est)
{
    if (est != NULL)
    {
        est->faults &= ~(unsigned)S4_FAULT_COLLISION;
    }
}

void s4_est_mag_calibration(s4_est_t *est, bool start, s4_events_t *events)
{
    unsigned axis;
    unsigned up_axis;
    bool spread_ok = true;
    s4_event_msg_t msg;

    if (est == NULL)
    {
        return;
    }

    if (start)
    {
        est->mag_cal_active = 1U;
        for (axis = 0U; axis < 3U; axis++)
        {
            est->mag_min[axis] = INT16_MAX;
            est->mag_max[axis] = INT16_MIN;
        }
        return;
    }
    if (!est->mag_cal_active)
    {
        return;
    }
    est->mag_cal_active = 0U;

    /* A flat spin only exercises the two horizontal sensor axes. */
    up_axis = est->cfg.axis_up & 3U;
    for (axis = 0U; axis < 3U; axis++)
    {
        if ((axis != up_axis) &&
            (((int32_t)est->mag_max[axis] - est->mag_min[axis]) <
             (int32_t)est->cfg.mag_cal_min_spread))
        {
            spread_ok = false;
        }
    }
    if (spread_ok)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            if (axis != up_axis)
            {
                est->mag_off[axis] = (int16_t)(((int32_t)est->mag_max[axis] +
                                                est->mag_min[axis]) / 2);
            }
        }
        est->mag_calibrated = 1U;
        est->heading_valid = 0U;
        est->mag_primed = 0U;
    }

    msg = event_header(S4_EV_CAL);
    msg = s4_put(msg, S4_EV_CAL_KIND_SHIFT, 1U, 1U);
    msg = s4_put(msg, S4_EV_CAL_OK_SHIFT, 1U, spread_ok ? 1U : 0U);
    push_event(events, msg);
}

static void finish_calibration(s4_est_t *est, bool ok, s4_events_t *events)
{
    s4_event_msg_t msg = event_header(S4_EV_CAL);

    est->cal_active = 0U;
    if (!est->calibrated)
    {
        est->faults |= S4_FAULT_UNCAL;
    }
    msg = s4_put(msg, S4_EV_CAL_KIND_SHIFT, 1U, 0U);
    msg = s4_put(msg, S4_EV_CAL_OK_SHIFT, 1U, ok ? 1U : 0U);
    push_event(events, msg);
}

static void calibration_step(
    s4_est_t *est,
    const int32_t vehicle[3],
    s4_events_t *events)
{
    int32_t mean[3];
    uint32_t g_mg;
    bool still = true;
    unsigned axis;

    for (axis = 0U; axis < 3U; axis++)
    {
        int16_t value = (int16_t)clamp32(vehicle[axis], INT16_MIN, INT16_MAX);
        est->cal_sum[axis] += value;
        if (value < est->cal_min[axis])
        {
            est->cal_min[axis] = value;
        }
        if (value > est->cal_max[axis])
        {
            est->cal_max[axis] = value;
        }
        if (((int32_t)est->cal_max[axis] - est->cal_min[axis]) >
            (int32_t)est->cfg.cal_max_spread_mg)
        {
            still = false;
        }
    }
    est->cal_count++;

    if (!still)
    {
        est->cal_retries++;
        if (est->cal_retries >= est->cfg.cal_max_retries)
        {
            finish_calibration(est, false, events);
        }
        else
        {
            reset_cal_window(est);
        }
        return;
    }
    if (est->cal_count < est->cfg.cal_samples)
    {
        return;
    }

    for (axis = 0U; axis < 3U; axis++)
    {
        mean[axis] = est->cal_sum[axis] / (int32_t)est->cal_count;
    }
    g_mg = imu_isqrt_u64((uint64_t)(((int64_t)mean[0] * mean[0]) +
                                    ((int64_t)mean[1] * mean[1]) +
                                    ((int64_t)mean[2] * mean[2])));
    if ((g_mg < 700U) || (g_mg > 1300U) || (mean[IMU_UP] <= 0))
    {
        /* Not resting on its wheels, or the scale/mounting is wrong. */
        finish_calibration(est, false, events);
        return;
    }

    est->pitch0_ddeg = imu_pitch_ddeg(mean);
    est->roll0_ddeg = imu_roll_ddeg(mean);
    est->g0_mg = (uint16_t)g_mg;
    for (axis = 0U; axis < 3U; axis++)
    {
        est->acc_fast[axis] = mean[axis] * (int32_t)(1U << FX);
        est->acc_slow[axis] = est->acc_fast[axis];
    }
    est->vib_x16 = 0;
    est->calibrated = 1U;
    est->faults &= ~(unsigned)S4_FAULT_UNCAL;
    finish_calibration(est, true, events);
}

static bool collision_check(
    s4_est_t *est,
    const int32_t vehicle[3],
    bool saturated,
    s4_events_t *events)
{
    int32_t d_fwd = vehicle[IMU_FWD] - asr32(est->acc_slow[IMU_FWD], FX);
    int32_t d_right = vehicle[IMU_RIGHT] - asr32(est->acc_slow[IMU_RIGHT], FX);
    uint32_t jolt = imu_isqrt_u64((uint64_t)(((int64_t)d_fwd * d_fwd) +
                                             ((int64_t)d_right * d_right)));
    s4_event_msg_t msg;
    uint32_t direction;

    if (saturated && (jolt < est->cfg.collision_mg))
    {
        jolt = est->cfg.collision_mg;
    }
    if (jolt < est->cfg.collision_mg)
    {
        return false;
    }

    if (est->impact_ms > 0U)
    {
        /* Same impact: keep the peak. */
        if (jolt > est->shock_mg)
        {
            est->shock_mg = (uint16_t)min_u32(jolt, UINT16_MAX);
        }
        return true;
    }

    /* The car is pushed away from the side that was hit. */
    if (abs32(d_fwd) >= abs32(d_right))
    {
        direction = (d_fwd < 0) ? S4_HIT_FRONT : S4_HIT_REAR;
    }
    else
    {
        direction = (d_right < 0) ? S4_HIT_RIGHT : S4_HIT_LEFT;
    }
    est->shock_mg = (uint16_t)min_u32(jolt, UINT16_MAX);
    est->collision_dir = direction;
    est->collision_count++;
    est->faults |= S4_FAULT_COLLISION;
    est->impact_ms = est->cfg.collision_hold_ms;

    msg = event_header(S4_EV_COLLISION);
    msg = s4_put(msg, S4_EV_COLL_DIR_SHIFT, S4_ST_COLLDIR_WIDTH, direction);
    msg = s4_put(msg, S4_EV_COLL_MAG_SHIFT, S4_MO_SHOCK_WIDTH,
                 min_u32(jolt / S4_SHOCK_UNIT_MG, S4_MASK(S4_MO_SHOCK_WIDTH)));
    push_event(events, msg);
    return true;
}

static void accel_step(s4_est_t *est, const s4_input_t *in, s4_events_t *events)
{
    int32_t sensor[3];
    int32_t vehicle[3];
    int32_t filtered[3];
    uint32_t magnitude;
    int32_t deviation;
    unsigned shift = est->cfg.lpf_shift;
    unsigned slow_shift = shift + est->cfg.slow_extra_shift;
    unsigned axis;
    bool spike;

    for (axis = 0U; axis < 3U; axis++)
    {
        sensor[axis] = in->accel_mg[axis];
    }
    imu_map_axes(sensor, (uint8_t)est->cfg.axis_right,
                 (uint8_t)est->cfg.axis_fwd, (uint8_t)est->cfg.axis_up, vehicle);

    if (in->accel_saturated)
    {
        est->faults |= S4_FAULT_SATURATED;
    }
    if (!est->filters_primed)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            est->acc_fast[axis] = vehicle[axis] * (int32_t)(1U << FX);
            est->acc_slow[axis] = est->acc_fast[axis];
        }
        est->filters_primed = 1U;
    }

    /* Collision samples are kept out of the filters so a jolt does not look
       like a pitch change and start a false hump. */
    spike = collision_check(est, vehicle, in->accel_saturated, events);
    if (!spike)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            int32_t target = vehicle[axis] * (int32_t)(1U << FX);
            est->acc_fast[axis] += asr32(target - est->acc_fast[axis], shift);
            est->acc_slow[axis] += asr32(target - est->acc_slow[axis], slow_shift);
        }
        magnitude = imu_isqrt_u64((uint64_t)(((int64_t)vehicle[0] * vehicle[0]) +
                                             ((int64_t)vehicle[1] * vehicle[1]) +
                                             ((int64_t)vehicle[2] * vehicle[2])));
        deviation = abs32((int32_t)magnitude - (int32_t)est->g0_mg);
        est->vib_x16 += asr32((deviation * (int32_t)(1U << FX)) - est->vib_x16, 3U);

        if (est->cal_active)
        {
            calibration_step(est, vehicle, events);
        }
    }

    if (est->calibrated)
    {
        for (axis = 0U; axis < 3U; axis++)
        {
            filtered[axis] = asr32(est->acc_fast[axis], FX);
        }
        est->pitch_ddeg = imu_wrap_ddeg(imu_pitch_ddeg(filtered) - est->pitch0_ddeg);
        est->roll_ddeg = imu_wrap_ddeg(imu_roll_ddeg(filtered) - est->roll0_ddeg);
        if ((abs32(est->pitch_ddeg) > (int32_t)est->cfg.tilt_limit_ddeg) ||
            (abs32(est->roll_ddeg) > (int32_t)est->cfg.tilt_limit_ddeg))
        {
            est->faults |= S4_FAULT_TILT;
        }
        else
        {
            est->faults &= ~(unsigned)S4_FAULT_TILT;
        }
    }
}

static void turn_step(s4_est_t *est, int32_t delta_ddeg, s4_events_t *events)
{
    int32_t rate = est->turn_ddps;
    uint32_t speed = (uint32_t)abs32(rate);
    s4_event_msg_t msg;
    uint32_t angle_deg;

    if (est->turn_state == S4_TURN_NONE)
    {
        if (speed >= est->cfg.turn_on_ddps)
        {
            est->turn_state = (rate > 0) ? S4_TURN_RIGHT : S4_TURN_LEFT;
            est->turn_acc_ddeg = delta_ddeg;
            est->turn_quiet = 0U;
        }
        return;
    }

    est->turn_acc_ddeg += delta_ddeg;
    if (speed < est->cfg.turn_off_ddps)
    {
        if (est->turn_quiet < 15U)
        {
            est->turn_quiet++;
        }
    }
    else
    {
        est->turn_quiet = 0U;
    }
    if (est->turn_quiet < est->cfg.confirm_samples)
    {
        return;
    }

    angle_deg = min_u32(((uint32_t)abs32(est->turn_acc_ddeg) + 5U) / 10U,
                        S4_MASK(S4_EV_TURN_ANGLE_WIDTH));
    msg = event_header(S4_EV_TURN);
    msg = s4_put(msg, S4_EV_TURN_RIGHT_SHIFT, 1U, (est->turn_acc_ddeg > 0) ? 1U : 0U);
    msg = s4_put(msg, S4_EV_TURN_ANGLE_SHIFT, S4_EV_TURN_ANGLE_WIDTH, angle_deg);
    push_event(events, msg);
    est->turn_state = S4_TURN_NONE;
    est->turn_acc_ddeg = 0;
}

static void heading_step(
    s4_est_t *est,
    const s4_input_t *in,
    uint16_t dt_ms,
    s4_events_t *events)
{
    int32_t sensor[3];
    int32_t vehicle[3];
    int32_t accel[3];
    int32_t mag[3];
    uint16_t heading;
    unsigned axis;

    for (axis = 0U; axis < 3U; axis++)
    {
        int16_t raw = in->mag[axis];
        if (est->mag_cal_active)
        {
            if (raw < est->mag_min[axis])
            {
                est->mag_min[axis] = raw;
            }
            if (raw > est->mag_max[axis])
            {
                est->mag_max[axis] = raw;
            }
        }
        sensor[axis] = (int32_t)raw - est->mag_off[axis];
    }
    imu_map_axes(sensor, (uint8_t)est->cfg.axis_right,
                 (uint8_t)est->cfg.axis_fwd, (uint8_t)est->cfg.axis_up, vehicle);

    for (axis = 0U; axis < 3U; axis++)
    {
        int32_t target = vehicle[axis] * (int32_t)(1U << FX);
        if (!est->mag_primed)
        {
            est->mag_filt[axis] = target;
        }
        else
        {
            est->mag_filt[axis] += asr32(target - est->mag_filt[axis],
                                         est->cfg.lpf_shift);
        }
        accel[axis] = asr32(est->acc_fast[axis], FX);
        mag[axis] = asr32(est->mag_filt[axis], FX);
    }
    est->mag_primed = 1U;

    if (!est->calibrated || !imu_heading_ddeg(accel, mag, &heading))
    {
        est->heading_valid = 0U;
        est->turn_ddps = 0;
        return;
    }

    if (est->heading_valid)
    {
        int32_t delta = imu_wrap_ddeg((int32_t)heading - (int32_t)est->heading_ddeg);
        int32_t rate = (delta * 1000) / (int32_t)dt_ms;
        est->turn_ddps += asr32(rate - est->turn_ddps, 2U);
        turn_step(est, delta, events);
    }
    est->heading_ddeg = heading;
    est->heading_valid = 1U;
}

static void motion_step(s4_est_t *est, const s4_input_t *in)
{
    uint32_t vibration = (uint32_t)asr32(est->vib_x16, FX);
    int32_t linear;

    if (est->impact_ms > 0U)
    {
        est->motion = S4_MOTION_IMPACT;
    }
    else if (vibration >= est->cfg.rough_vib_mg)
    {
        est->motion = S4_MOTION_ROUGH;
    }
    else if ((vibration <= est->cfg.still_vib_mg) &&
             (!in->odom_valid || (in->odom_delta_mm == 0)))
    {
        est->motion = S4_MOTION_STATIONARY;
    }
    else
    {
        est->motion = S4_MOTION_MOVING;
    }

    est->longitudinal = S4_LONG_STEADY;
    if (est->calibrated && (est->terrain == TERRAIN_LEVEL) &&
        (est->motion != S4_MOTION_IMPACT))
    {
        linear = asr32(est->acc_fast[IMU_FWD] - est->acc_slow[IMU_FWD], FX);
        if (linear > (int32_t)est->cfg.long_accel_mg)
        {
            est->longitudinal = S4_LONG_ACCELERATING;
        }
        else if (linear < -(int32_t)est->cfg.long_accel_mg)
        {
            est->longitudinal = S4_LONG_BRAKING;
        }
    }
}

static int32_t distance_step_um(s4_est_t *est, const s4_input_t *in, uint16_t dt_ms)
{
    if (in->odom_valid)
    {
        est->dist_odometry = 1U;
        return (int32_t)in->odom_delta_mm * 1000;
    }
    est->dist_odometry = 0U;
    /* Vibration alone cannot always tell quiet driving from standing still,
       but pitching through a hump means the wheels are rolling. */
    if ((est->motion == S4_MOTION_STATIONARY) && !est->in_episode &&
        (abs32(est->pitch_ddeg) < (int32_t)est->cfg.level_ddeg))
    {
        return 0;
    }
    /* mm/s * ms = micrometres */
    return (int32_t)est->cfg.fallback_speed_mm_s * (int32_t)dt_ms;
}

static void finish_episode(s4_est_t *est, bool aborted, s4_events_t *events)
{
    uint32_t peak_mm;
    uint32_t residual_mm;
    uint32_t path_mm;
    uint32_t uncertainty;
    uint32_t residual_limit;
    uint32_t word = 0U;
    bool valid;

    if (!est->in_episode)
    {
        return;
    }
    est->in_episode = 0U;

    peak_mm = (est->peak_um > 0) ? (uint32_t)((est->peak_um + 500) / 1000) : 0U;
    residual_mm = (uint32_t)(abs32(est->height_um) / 1000);
    path_mm = est->path_um / 1000U;
    residual_limit = (peak_mm / 2U > 10U) ? (peak_mm / 2U) : 10U;

    uncertainty = (uint32_t)est->cfg.base_uncertainty_mm + residual_mm + (path_mm / 100U);
    if (est->used_fallback)
    {
        uncertainty += peak_mm / 2U;
    }

    valid = !aborted &&
        est->saw_descent &&
        (peak_mm >= est->cfg.min_hump_mm) &&
        (peak_mm <= S4_MASK(S4_HU_HEIGHT_WIDTH)) &&
        (path_mm >= est->cfg.min_length_mm) &&
        (path_mm <= est->cfg.max_length_mm) &&
        (residual_mm <= residual_limit);

    est->hump_seq++;
    word = s4_put(word, S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH,
                  min_u32(peak_mm, S4_MASK(S4_HU_HEIGHT_WIDTH)));
    word = s4_put(word, S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH,
                  min_u32(uncertainty, S4_MASK(S4_HU_UNC_WIDTH)));
    word = s4_put(word, S4_HU_PITCH_SHIFT, S4_HU_PITCH_WIDTH,
                  min_u32((uint32_t)(est->peak_pitch_ddeg + 5) / 10U,
                          S4_MASK(S4_HU_PITCH_WIDTH)));
    word = s4_put(word, S4_HU_LEN_SHIFT, S4_HU_LEN_WIDTH,
                  min_u32(path_mm / 10U, S4_MASK(S4_HU_LEN_WIDTH)));
    word = s4_put(word, S4_HU_VALID_SHIFT, 1U, valid ? 1U : 0U);
    word = s4_put(word, S4_HU_SEQ_SHIFT, S4_HU_SEQ_WIDTH, est->hump_seq);
    est->words[S4_W_HUMP] = word;
    log_max(&est->ranges.hump_height_mm, peak_mm);
    log_max(&est->ranges.hump_length_mm, path_mm);

    push_event(events, event_header(S4_EV_HUMP) |
               ((word & S4_MASK(S4_HU_RESULT_WIDTH)) << S4_EV_PAYLOAD_SHIFT));
}

static void begin_episode(s4_est_t *est)
{
    /* Height and path keep the climb seen before the state was confirmed. */
    est->in_episode = 1U;
    est->episode_ms = 0U;
    est->saw_descent = 0U;
    est->peak_um = (est->height_um > 0) ? est->height_um : 0;
    est->peak_pitch_ddeg = (int16_t)abs32(est->pitch_ddeg);
}

static uint8_t terrain_candidate(const s4_est_t *est, int32_t pitch)
{
    const s4_cfg_t *cfg = &est->cfg;
    int32_t level = (int32_t)cfg->level_ddeg;
    int32_t ascend = (int32_t)cfg->ascend_ddeg;
    int32_t descend = -(int32_t)cfg->descend_ddeg;

    switch (est->terrain)
    {
        case TERRAIN_LEVEL:
            if (pitch >= ascend)
            {
                return TERRAIN_ASCENDING;
            }
            if (pitch <= descend)
            {
                return TERRAIN_DESCENDING;
            }
            break;
        case TERRAIN_ASCENDING:
            if (pitch < level)
            {
                return TERRAIN_CREST;
            }
            break;
        case TERRAIN_CREST:
            if (pitch <= descend)
            {
                return TERRAIN_DESCENDING;
            }
            if (pitch >= ascend)
            {
                return TERRAIN_ASCENDING;
            }
            break;
        case TERRAIN_DESCENDING:
            if (pitch > -level)
            {
                return TERRAIN_SETTLING;
            }
            break;
        case TERRAIN_SETTLING:
            if (pitch >= ascend)
            {
                return TERRAIN_ASCENDING;
            }
            if (pitch <= descend)
            {
                return TERRAIN_DESCENDING;
            }
            break;
        default:
            break;
    }
    return (uint8_t)est->terrain;
}

static void change_terrain(s4_est_t *est, uint8_t next, s4_events_t *events)
{
    if ((est->terrain == TERRAIN_SETTLING) && (next == TERRAIN_ASCENDING))
    {
        /* A second hump straight after the first. */
        finish_episode(est, false, events);
        rebase_height(est);
        begin_episode(est);
    }
    else if (est->terrain == TERRAIN_LEVEL)
    {
        begin_episode(est);
    }
    if (next == TERRAIN_DESCENDING)
    {
        est->saw_descent = 1U;
    }
    set_terrain(est, next);
}

static void terrain_step(
    s4_est_t *est,
    int32_t ds_um,
    uint16_t dt_ms,
    s4_events_t *events)
{
    const s4_cfg_t *cfg = &est->cfg;
    int32_t pitch = est->pitch_ddeg;
    bool in_level_band = abs32(pitch) < (int32_t)cfg->level_ddeg;
    uint8_t candidate;

    if ((est->faults & S4_FAULT_BLOCKING) != 0U)
    {
        finish_episode(est, true, events);
        set_terrain(est, est->calibrated ? TERRAIN_INVALID : TERRAIN_UNKNOWN);
        return;
    }
    if (!est->calibrated || est->cal_active)
    {
        est->in_episode = 0U;
        set_terrain(est, TERRAIN_UNKNOWN);
        return;
    }
    if ((est->terrain == TERRAIN_UNKNOWN) || (est->terrain == TERRAIN_INVALID))
    {
        if (in_level_band)
        {
            rebase_height(est);
            set_terrain(est, TERRAIN_LEVEL);
        }
        return;
    }
    if (est->impact_ms > 0U)
    {
        return;
    }

    /* Height gained = sin(pitch) x distance along the surface. */
    if (ds_um != 0)
    {
        est->height_um += (int32_t)(((int64_t)ds_um * imu_sin_q15(pitch)) / 32768);
        est->path_um += (uint32_t)abs32(ds_um);
        if (!est->dist_odometry)
        {
            est->used_fallback = 1U;
        }
    }
    est->state_ms = add_sat_u16(est->state_ms, dt_ms);
    if (est->in_episode)
    {
        est->episode_ms = add_sat_u16(est->episode_ms, dt_ms);
        if (est->height_um > est->peak_um)
        {
            est->peak_um = est->height_um;
        }
        if (abs32(pitch) > est->peak_pitch_ddeg)
        {
            est->peak_pitch_ddeg = (int16_t)abs32(pitch);
        }
    }

    if ((est->terrain == TERRAIN_LEVEL) && in_level_band)
    {
        rebase_height(est);
    }

    if (((est->terrain == TERRAIN_SETTLING) && (est->state_ms >= cfg->settle_ms)) ||
        ((est->terrain == TERRAIN_CREST) && (est->state_ms >= cfg->crest_timeout_ms)))
    {
        finish_episode(est, false, events);
        set_terrain(est, TERRAIN_LEVEL);
        return;
    }
    if (est->in_episode && (est->episode_ms >= cfg->episode_timeout_ms))
    {
        finish_episode(est, true, events);
        set_terrain(est, TERRAIN_LEVEL);
        return;
    }

    candidate = terrain_candidate(est, pitch);
    if (candidate == est->terrain)
    {
        est->pending = candidate;
        est->confirm = 0U;
        return;
    }
    if (candidate == est->pending)
    {
        if (est->confirm < 15U)
        {
            est->confirm++;
        }
    }
    else
    {
        est->pending = candidate;
        est->confirm = 1U;
    }
    if (est->confirm >= cfg->confirm_samples)
    {
        change_terrain(est, candidate, events);
    }
}

static uint32_t speed_cap_mm_s(const s4_est_t *est)
{
    uint32_t cap;
    uint32_t degraded = est->cfg.degraded_cap_mm_s;

    switch (est->terrain)
    {
        case TERRAIN_LEVEL:
            cap = 0U;
            break;
        case TERRAIN_ASCENDING:
        case TERRAIN_CREST:
        case TERRAIN_DESCENDING:
        case TERRAIN_SETTLING:
            cap = est->cfg.hump_cap_mm_s;
            break;
        default:
            cap = degraded;
            break;
    }
    if (((est->faults & S4_FAULT_TILT) != 0U) || (est->motion == S4_MOTION_IMPACT))
    {
        if ((cap == 0U) || (degraded < cap))
        {
            cap = degraded;
        }
    }
    return cap;
}

static uint32_t cap_units(uint32_t cap_mm_s)
{
    uint32_t units;

    if (cap_mm_s == 0U)
    {
        return 0U;
    }
    /* Round down (stricter); never let a real cap encode as "no cap". */
    units = cap_mm_s / S4_CAP_UNIT_MM_S;
    if (units == 0U)
    {
        units = 1U;
    }
    return min_u32(units, S4_MASK(S4_ST_CAP_WIDTH));
}

/* Validate before packing: saturate to the field and remember that we did. */
static uint32_t fit_unsigned(uint32_t value, unsigned width, bool *clamped)
{
    if (value > S4_MASK(width))
    {
        *clamped = true;
        return S4_MASK(width);
    }
    return value;
}

static uint32_t fit_signed(int32_t value, unsigned width, bool *clamped)
{
    int32_t high = (int32_t)S4_MASK(width - 1U);
    int32_t low = -high - 1;

    if ((value > high) || (value < low))
    {
        *clamped = true;
        value = clamp32(value, low, high);
    }
    return (uint32_t)value;
}

/* One 3-bit class from the private motion, longitudinal and turn states. */
static uint32_t motion_class(const s4_est_t *est)
{
    if (est->motion == S4_MOTION_IMPACT)
    {
        return S4_MC_IMPACT;
    }
    if (est->motion == S4_MOTION_ROUGH)
    {
        return S4_MC_ROUGH;
    }
    if (est->turn_state == S4_TURN_LEFT)
    {
        return S4_MC_TURN_LEFT;
    }
    if (est->turn_state == S4_TURN_RIGHT)
    {
        return S4_MC_TURN_RIGHT;
    }
    if (est->longitudinal == S4_LONG_ACCELERATING)
    {
        return S4_MC_ACCELERATING;
    }
    if (est->longitudinal == S4_LONG_BRAKING)
    {
        return S4_MC_BRAKING;
    }
    return (est->motion == S4_MOTION_STATIONARY) ? S4_MC_STATIONARY : S4_MC_MOVING;
}

static uint32_t calibration_status(const s4_est_t *est)
{
    if (est->cal_active)
    {
        return S4_CAL_RUNNING;
    }
    if (!est->calibrated)
    {
        return S4_CAL_NONE;
    }
    return est->mag_calibrated ? S4_CAL_LEVEL_MAG : S4_CAL_LEVEL;
}

static void pack_words(s4_est_t *est)
{
    uint32_t status = 0U;
    uint32_t attitude = 0U;
    uint32_t motion = 0U;
    uint32_t vibration_mg = (uint32_t)asr32(est->vib_x16, FX);
    int32_t turn_dps = round_div32(est->turn_ddps, 10);
    int32_t live_mm = 0;
    bool clamped = false;
    bool on_hump = (est->terrain == TERRAIN_ASCENDING) ||
        (est->terrain == TERRAIN_CREST) ||
        (est->terrain == TERRAIN_DESCENDING) ||
        (est->terrain == TERRAIN_SETTLING);
    bool data_valid = est->calibrated &&
        ((est->faults & S4_FAULT_BLOCKING) == 0U);

    if (est->in_episode && (est->height_um > 0))
    {
        live_mm = est->height_um / 1000;
    }

    attitude = s4_put(attitude, S4_AT_PITCH_SHIFT, S4_AT_PITCH_WIDTH,
                      fit_signed(round_div32(est->pitch_ddeg, 5), S4_AT_PITCH_WIDTH,
                                 &clamped));
    attitude = s4_put(attitude, S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH,
                      fit_signed(round_div32(est->roll_ddeg, 5), S4_AT_ROLL_WIDTH,
                                 &clamped));
    attitude = s4_put(attitude, S4_AT_HEADING_SHIFT, S4_AT_HEADING_WIDTH,
                      (((uint32_t)est->heading_ddeg + 5U) / 10U) % 360U);
    attitude = s4_put(attitude, S4_AT_SEQ_SHIFT, S4_AT_SEQ_WIDTH, est->sample_seq);

    motion = s4_put(motion, S4_MO_TURNRATE_SHIFT, S4_MO_TURNRATE_WIDTH,
                    fit_signed(round_div32(turn_dps, (int32_t)S4_TURNRATE_UNIT_DPS),
                               S4_MO_TURNRATE_WIDTH, &clamped));
    motion = s4_put(motion, S4_MO_LIVEH_SHIFT, S4_MO_LIVEH_WIDTH,
                    fit_unsigned((uint32_t)live_mm, S4_MO_LIVEH_WIDTH, &clamped));
    motion = s4_put(motion, S4_MO_SHOCK_SHIFT, S4_MO_SHOCK_WIDTH,
                    fit_unsigned(est->shock_mg / S4_SHOCK_UNIT_MG, S4_MO_SHOCK_WIDTH,
                                 &clamped));
    motion = s4_put(motion, S4_MO_VIB_SHIFT, S4_MO_VIB_WIDTH,
                    fit_unsigned(vibration_mg / S4_VIB_UNIT_MG, S4_MO_VIB_WIDTH,
                                 &clamped));

    status = s4_put(status, S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH, est->terrain);
    status = s4_put(status, S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH,
                    cap_units(speed_cap_mm_s(est)));
    status = s4_put(status, S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH, motion_class(est));
    status = s4_put(status, S4_ST_FAULT_SHIFT, S4_ST_FAULT_WIDTH, est->faults);
    status = s4_put(status, S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH, calibration_status(est));
    status = s4_put(status, S4_ST_ODOM_SHIFT, 1U, est->dist_odometry);
    status = s4_put(status, S4_ST_HUMP_SHIFT, 1U, on_hump ? 1U : 0U);
    status = s4_put(status, S4_ST_COLL_SHIFT, 1U, (est->impact_ms > 0U) ? 1U : 0U);
    status = s4_put(status, S4_ST_COLLCNT_SHIFT, S4_ST_COLLCNT_WIDTH, est->collision_count);
    status = s4_put(status, S4_ST_COLLDIR_SHIFT, S4_ST_COLLDIR_WIDTH, est->collision_dir);
    status = s4_put(status, S4_ST_CLAMPED_SHIFT, 1U, clamped ? 1U : 0U);
    status = s4_put(status, S4_ST_VALID_SHIFT, 1U, data_valid ? 1U : 0U);

    /* Observed ranges, before any clamping. */
    log_max(&est->ranges.turn_rate_dps, (uint32_t)abs32(turn_dps));
    log_max(&est->ranges.pitch_ddeg, (uint32_t)abs32(est->pitch_ddeg));
    log_max(&est->ranges.roll_ddeg, (uint32_t)abs32(est->roll_ddeg));
    log_max(&est->ranges.shock_mg, est->shock_mg);
    log_max(&est->ranges.vibration_mg, vibration_mg);
    if (clamped && (est->ranges.clamp_count < UINT16_MAX))
    {
        est->ranges.clamp_count++;
    }

    est->words[S4_W_STATUS] = status;
    est->words[S4_W_ATTITUDE] = attitude;
    est->words[S4_W_MOTION] = motion;
}

void s4_est_reset_ranges(s4_est_t *est)
{
    if (est != NULL)
    {
        (void)memset(&est->ranges, 0, sizeof(est->ranges));
    }
}

void s4_est_step(s4_est_t *est, const s4_input_t *in, s4_events_t *events)
{
    const unsigned reportable = ~(unsigned)(S4_FAULT_SATURATED | S4_FAULT_COLLISION);
    uint32_t prev_status;
    unsigned prev_faults;
    uint16_t dt_ms;
    int32_t ds_um;
    s4_event_msg_t msg;

    if (events != NULL)
    {
        events->count = 0U;
    }
    if ((est == NULL) || (in == NULL))
    {
        return;
    }
    prev_status = est->words[S4_W_STATUS];
    prev_faults = est->faults;
    dt_ms = in->dt_ms;
    if (dt_ms == 0U)
    {
        dt_ms = 1U;
    }
    else if (dt_ms > 1000U)
    {
        dt_ms = 1000U;
    }

    est->faults &= ~(unsigned)S4_FAULT_SATURATED;
    est->impact_ms = (est->impact_ms > dt_ms) ? (uint16_t)(est->impact_ms - dt_ms) : 0U;
    if (in->accel_ok)
    {
        est->no_sample_ms = 0U;
        est->faults &= ~(unsigned)(S4_FAULT_I2C | S4_FAULT_STALE);
        est->sample_seq++;
    }
    else
    {
        est->no_sample_ms = add_sat_u16(est->no_sample_ms, dt_ms);
        if (in->bus_fault)
        {
            est->faults |= S4_FAULT_I2C;
        }
        if (est->no_sample_ms >= est->cfg.stale_ms)
        {
            est->faults |= S4_FAULT_STALE;
            est->filters_primed = 0U;
        }
    }
    if (in->mag_ok)
    {
        est->faults &= ~(unsigned)S4_FAULT_MAG;
    }
    else
    {
        est->faults |= S4_FAULT_MAG;
        est->heading_valid = 0U;
        est->mag_primed = 0U;
        est->turn_ddps = 0;
    }

    if (in->accel_ok)
    {
        accel_step(est, in, events);
        if (in->mag_ok)
        {
            heading_step(est, in, dt_ms, events);
        }
    }
    motion_step(est, in);
    ds_um = distance_step_um(est, in, dt_ms);
    terrain_step(est, ds_um, dt_ms, events);
    pack_words(est);

    if ((s4_get(prev_status, S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH) != est->terrain) ||
        (s4_get(prev_status, S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) !=
         s4_get(est->words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH)))
    {
        msg = event_header(S4_EV_TERRAIN);
        msg = s4_put(msg, S4_EV_TERRAIN_STATE_SHIFT, S4_ST_TERRAIN_WIDTH, est->terrain);
        msg = s4_put(msg, S4_EV_TERRAIN_CAP_SHIFT, S4_ST_CAP_WIDTH,
                     s4_get(est->words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH));
        push_event(events, msg);
    }
    if ((prev_faults & reportable) != (est->faults & reportable))
    {
        msg = event_header(S4_EV_FAULT);
        msg = s4_put(msg, S4_EV_FAULT_SHIFT, S4_FAULT_WIDTH, est->faults);
        push_event(events, msg);
    }
}
