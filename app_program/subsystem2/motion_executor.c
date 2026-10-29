#include "motion_executor.h"

#include <stddef.h>
#include <string.h>

#include "drv_motor.h"
#include "motion_calibration.h"

#define GOAL_MAX_MM 5000
#define GOAL_MAX_MDEG 360000
#define SPEED_WINDOW_MS 50U
#define PI_U6 3141593ULL

/* b is newer than a: 1 .. 2^31-1 steps ahead (wrap-safe, like drv_motor). */
static bool newer(authority_generation_t b, authority_generation_t a)
{
    uint32_t delta = (uint32_t)(b - a);
    return (delta != 0U) && (delta < 0x80000000UL);
}

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static int16_t clamp_effort(int32_t value, int16_t limit)
{
    if (value > limit) { return limit; }
    if (value < -limit) { return (int16_t)-limit; }
    return (int16_t)value;
}

/* Right-wheel cap: max_effort scaled by the trim, never above full power. */
static int16_t right_limit(const motion_executor_config_t *config)
{
    int32_t limit = ((int32_t)config->max_effort *
        (int32_t)config->right_trim_permille) / 1000;
    return (int16_t)((limit > MOTION_EFFORT_MAX) ? MOTION_EFFORT_MAX : limit);
}

static int32_t feedforward(const motion_executor_config_t *config,
    int32_t speed_mm_s)
{
    return (speed_mm_s * config->effort_per_100_mm_s) / 100;
}

static int8_t sign_of(int32_t value)
{
    return (value > 0) ? 1 : ((value < 0) ? -1 : 0);
}

void motion_executor_default_config(motion_executor_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    (void)memset(config, 0, sizeof(*config));
    config->calibration.wheel_diameter_mm = MOTION_WHEEL_DIAMETER_MM_CANDIDATE;
    config->calibration.wheelbase_mm = MOTION_TRACK_WIDTH_MM;
    config->calibration.left_counts_per_revolution =
        MOTION_LEFT_BOTH_EDGE_COUNTS_PER_REV;
    config->calibration.right_counts_per_revolution =
        MOTION_RIGHT_BOTH_EDGE_COUNTS_PER_REV;
    /* Matches Buddy 3's line-follow bench: 150 mm/s -> effort 200 (20%). */
    config->effort_per_100_mm_s = 133;
    config->goal_min_effort = MOTION_GOAL_MIN_EFFORT; /* B2 bench: 15% may not start a wheel */
    config->max_effort = 350;        /* Buddy 3 bench cap (35%) */
    config->straight_gain_per_mm = 8;
    config->straight_max = 60;
    config->default_goal_speed_mm_s = 120U;
    config->max_speed_mm_s = 250U;
    config->default_goal_timeout_ms = 5000U;
    config->slow_zone_permille = 750U;
    config->right_trim_permille = MOTION_RIGHT_TRIM_PERMILLE;
}

bool motion_executor_init(motion_executor_t *executor,
    const motion_executor_config_t *config)
{
    if ((executor == NULL) || (config == NULL) ||
        !motion_calibration_is_valid(&config->calibration) ||
        (config->effort_per_100_mm_s <= 0) || (config->max_effort <= 0) ||
        (config->max_effort > MOTION_EFFORT_MAX) ||
        (config->goal_min_effort < 0) ||
        (config->goal_min_effort > config->max_effort) ||
        (config->default_goal_speed_mm_s == 0U) ||
        (config->default_goal_speed_mm_s > config->max_speed_mm_s) ||
        (config->default_goal_timeout_ms == 0U) ||
        (config->slow_zone_permille > 1000U) ||
        (config->right_trim_permille < 500U) ||
        (config->right_trim_permille > 1500U))
    {
        return false;
    }
    (void)memset(executor, 0, sizeof(*executor));
    executor->config = *config;
    return true;
}

/* Stale: older than the current command, or at/below an abort floor. */
static bool authority_ok(const motion_executor_t *executor,
    authority_generation_t generation, motion_source_t source)
{
    if (generation == 0U)
    {
        return false;
    }
    if (executor->floor_set && !newer(generation, executor->floor))
    {
        return false;
    }
    if ((source == MOTION_SOURCE_LINE) && executor->line_floor_set &&
        !newer(generation, executor->line_floor))
    {
        return false;
    }
    return !executor->generation_seen ||
        (generation == executor->generation) ||
        newer(generation, executor->generation);
}

static void finish_goal(motion_executor_t *executor, motion_result_code_t code,
    uint32_t left_count, uint32_t right_count);

static void take_authority(motion_executor_t *executor,
    authority_generation_t generation)
{
    executor->generation = generation;
    executor->generation_seen = true;
    executor->accepted++;
}

bool motion_executor_continuous(motion_executor_t *executor,
    const motion_continuous_request_t *request, uint32_t now_ms)
{
    if ((executor == NULL) || (request == NULL))
    {
        return false;
    }
    if (!authority_ok(executor, request->authority_generation, request->source) ||
        (request->speed_mm_s > (int32_t)executor->config.max_speed_mm_s) ||
        (request->speed_mm_s < -(int32_t)executor->config.max_speed_mm_s) ||
        time_reached(now_ms, request->expires_ms))
    {
        executor->rejected++;
        return false;
    }
    if (executor->mode == MOTION_EXECUTOR_GOAL)
    {
        finish_goal(executor, MOTION_RESULT_CANCELLED, executor->start_left,
            executor->start_right);
    }
    take_authority(executor, request->authority_generation);
    executor->request = *request;
    executor->mode = MOTION_EXECUTOR_CONTINUOUS;
    return true;
}

bool motion_executor_goal(motion_executor_t *executor,
    const motion_goal_request_t *goal, uint32_t now_ms,
    uint32_t left_count, uint32_t right_count)
{
    const motion_calibration_t *cal;
    uint32_t arc_mm;
    bool valid;

    if ((executor == NULL) || (goal == NULL))
    {
        return false;
    }
    cal = &executor->config.calibration;
    switch (goal->type)
    {
        case MOTION_GOAL_FORWARD:
        case MOTION_GOAL_REVERSE:
            valid = (goal->target_mm > 0) && (goal->target_mm <= GOAL_MAX_MM);
            arc_mm = valid ? (uint32_t)goal->target_mm : 0U;
            break;
        case MOTION_GOAL_PIVOT_LEFT:
        case MOTION_GOAL_PIVOT_RIGHT:
            valid = (goal->target_mdeg > 0) && (goal->target_mdeg <= GOAL_MAX_MDEG);
            arc_mm = valid ?
                motion_pivot_wheel_distance_mm((uint32_t)goal->target_mdeg, cal) : 0U;
            break;
        default:
            valid = false;
            arc_mm = 0U;
            break;
    }
    if (!valid || (arc_mm == 0U) || (goal->goal_id == 0U) ||
        (goal->max_speed_mm_s > executor->config.max_speed_mm_s) ||
        !authority_ok(executor, goal->authority_generation, goal->source))
    {
        executor->rejected++;
        return false;
    }
    if (executor->mode == MOTION_EXECUTOR_GOAL)
    {
        finish_goal(executor, MOTION_RESULT_CANCELLED, left_count, right_count);
    }
    take_authority(executor, goal->authority_generation);
    executor->goal = *goal;
    executor->goal_started_ms = now_ms;
    executor->start_left = left_count;
    executor->start_right = right_count;
    executor->target_left = motion_mm_to_counts(arc_mm,
        cal->left_counts_per_revolution, cal);
    executor->target_right = motion_mm_to_counts(arc_mm,
        cal->right_counts_per_revolution, cal);
    executor->mode = MOTION_EXECUTOR_GOAL;
    return true;
}

static uint32_t travelled_mm(const motion_executor_t *executor,
    uint32_t left_count, uint32_t right_count)
{
    const motion_calibration_t *cal = &executor->config.calibration;
    uint32_t left = motion_counts_to_mm(left_count - executor->start_left,
        cal->left_counts_per_revolution, cal);
    uint32_t right = motion_counts_to_mm(right_count - executor->start_right,
        cal->right_counts_per_revolution, cal);
    return (left + right) / 2U;
}

static void finish_goal(motion_executor_t *executor, motion_result_code_t code,
    uint32_t left_count, uint32_t right_count)
{
    motion_goal_result_t *result = &executor->result;
    uint32_t mm = travelled_mm(executor, left_count, right_count);
    uint64_t mdeg;

    (void)memset(result, 0, sizeof(*result));
    result->goal_id = executor->goal.goal_id;
    result->authority_generation = executor->goal.authority_generation;
    result->code = code;
    result->odometry_valid = true;
    switch (executor->goal.type)
    {
        case MOTION_GOAL_FORWARD:
            result->travelled_mm = (int32_t)mm;
            break;
        case MOTION_GOAL_REVERSE:
            result->travelled_mm = -(int32_t)mm;
            break;
        default:
            /* Each wheel's arc is (pi * track / 360) mm per degree. */
            mdeg = ((uint64_t)mm * 360000ULL * 1000000ULL) /
                (PI_U6 * executor->config.calibration.wheelbase_mm);
            if (mdeg > (uint64_t)GOAL_MAX_MDEG * 2ULL)
            {
                mdeg = (uint64_t)GOAL_MAX_MDEG * 2ULL;
            }
            result->turned_mdeg = (executor->goal.type == MOTION_GOAL_PIVOT_LEFT)
                ? (int32_t)mdeg : -(int32_t)mdeg;
            break;
    }
    executor->result_valid = true;
    executor->mode = MOTION_EXECUTOR_IDLE;
}

void motion_executor_halt(motion_executor_t *executor)
{
    if (executor == NULL)
    {
        return;
    }
    if (executor->generation_seen)
    {
        executor->line_floor = executor->generation;
        executor->line_floor_set = true;
    }
    if (executor->mode == MOTION_EXECUTOR_GOAL)
    {
        /* Counts at the start: the true distance is unknown here. */
        finish_goal(executor, MOTION_RESULT_CANCELLED, executor->start_left,
            executor->start_right);
        executor->result.odometry_valid = false;
    }
    executor->mode = MOTION_EXECUTOR_IDLE;
}

void motion_executor_abort(motion_executor_t *executor,
    motion_result_code_t code)
{
    if (executor == NULL)
    {
        return;
    }
    motion_executor_halt(executor);
    if (executor->result_valid &&
        (executor->result.code == MOTION_RESULT_CANCELLED) &&
        (executor->result.goal_id == executor->goal.goal_id) &&
        (executor->result.authority_generation == executor->generation))
    {
        executor->result.code = code;
    }
    if (executor->generation_seen)
    {
        executor->floor = executor->generation;
        executor->floor_set = true;
    }
}

static void measure_speed(motion_executor_t *executor, uint32_t now_ms,
    uint32_t left_count, uint32_t right_count)
{
    const motion_calibration_t *cal = &executor->config.calibration;
    uint32_t elapsed;

    if (!executor->speed_started)
    {
        executor->speed_started = true;
    }
    else
    {
        elapsed = now_ms - executor->speed_ms;
        if (elapsed < SPEED_WINDOW_MS)
        {
            return;
        }
        executor->left_mm_s = (int32_t)((motion_counts_to_mm(
            left_count - executor->speed_left_count,
            cal->left_counts_per_revolution, cal) * 1000U) / elapsed);
        executor->right_mm_s = (int32_t)((motion_counts_to_mm(
            right_count - executor->speed_right_count,
            cal->right_counts_per_revolution, cal) * 1000U) / elapsed);
        /* Single-channel encoders: the sign comes from the command. */
        if (executor->last_left < 0) { executor->left_mm_s = -executor->left_mm_s; }
        if (executor->last_right < 0) { executor->right_mm_s = -executor->right_mm_s; }
    }
    executor->speed_left_count = left_count;
    executor->speed_right_count = right_count;
    executor->speed_ms = now_ms;
}

static void continuous_effort(motion_executor_t *executor,
    int32_t *left, int32_t *right)
{
    const motion_continuous_request_t *request = &executor->request;
    int32_t speed = request->speed_mm_s;
    int32_t base;
    int64_t k;   /* curvature * track/2, per-mille */
    int32_t limit = executor->config.max_effort;

    if ((request->speed_cap_mm_s > 0U) &&
        ((uint32_t)((speed < 0) ? -speed : speed) > request->speed_cap_mm_s))
    {
        speed = (speed < 0) ? -(int32_t)request->speed_cap_mm_s
            : (int32_t)request->speed_cap_mm_s;
    }
    base = feedforward(&executor->config, speed);
    /* curvature [1e-6 / m] * half track [mm] = 1e-9; * 1000 per-mille. */
    k = ((int64_t)request->curvature_u1_m *
        (int64_t)(executor->config.calibration.wheelbase_mm / 2U)) / 1000000LL;
    *left = (int32_t)(((int64_t)base * (1000LL - k)) / 1000LL);
    *right = (int32_t)(((int64_t)base * (1000LL + k)) / 1000LL);
    *right = (int32_t)(((int64_t)*right *
        (int64_t)executor->config.right_trim_permille) / 1000LL);
    /* Never reverse a wheel in continuous mode: clamp toward zero. */
    if (sign_of(*left) != sign_of(base)) { *left = 0; }
    if (sign_of(*right) != sign_of(base)) { *right = 0; }
    if (*left > limit) { *left = limit; }
    if (*left < -limit) { *left = -limit; }
    /* The weak wheel's cap scales with its trim, or full steer clips it away. */
    limit = right_limit(&executor->config);
    if (*right > limit) { *right = limit; }
    if (*right < -limit) { *right = -limit; }
}

static void goal_effort(motion_executor_t *executor, uint32_t left_count,
    uint32_t right_count, int32_t *left, int32_t *right)
{
    const motion_executor_config_t *config = &executor->config;
    const motion_calibration_t *cal = &config->calibration;
    uint32_t done_left = left_count - executor->start_left;
    uint32_t done_right = right_count - executor->start_right;
    uint32_t speed = (executor->goal.max_speed_mm_s != 0U)
        ? executor->goal.max_speed_mm_s : config->default_goal_speed_mm_s;
    uint32_t progress_left = (uint32_t)(((uint64_t)done_left * 1000ULL) /
        executor->target_left);
    uint32_t progress_right = (uint32_t)(((uint64_t)done_right * 1000ULL) /
        executor->target_right);
    int32_t magnitude;
    int32_t correction = 0;
    int32_t mismatch_mm;
    int8_t left_sign;
    int8_t right_sign;

    if (((progress_left < progress_right) ? progress_left : progress_right) >=
        config->slow_zone_permille)
    {
        speed /= 2U;
    }
    magnitude = feedforward(config, (int32_t)speed);
    if (magnitude < config->goal_min_effort)
    {
        magnitude = config->goal_min_effort;
    }

    switch (executor->goal.type)
    {
        case MOTION_GOAL_FORWARD: left_sign = 1; right_sign = 1; break;
        case MOTION_GOAL_REVERSE: left_sign = -1; right_sign = -1; break;
        case MOTION_GOAL_PIVOT_LEFT: left_sign = -1; right_sign = 1; break;
        default: left_sign = 1; right_sign = -1; break;
    }

    /* Keep both wheels level: slow the one that is ahead. */
    mismatch_mm = (int32_t)motion_counts_to_mm(done_left,
        cal->left_counts_per_revolution, cal) -
        (int32_t)motion_counts_to_mm(done_right,
        cal->right_counts_per_revolution, cal);
    correction = mismatch_mm * config->straight_gain_per_mm;
    if (correction > config->straight_max) { correction = config->straight_max; }
    if (correction < -config->straight_max) { correction = -config->straight_max; }

    *left = (done_left >= executor->target_left) ? 0 :
        left_sign * (magnitude - ((correction > 0) ? correction : 0));
    *right = (done_right >= executor->target_right) ? 0 :
        right_sign * (magnitude + ((correction < 0) ? correction : 0));
}

motion_executor_output_t motion_executor_step(motion_executor_t *executor,
    uint32_t now_ms, uint32_t left_count, uint32_t right_count)
{
    motion_executor_output_t out = {0, 0, 0U, false};
    int32_t left = 0;
    int32_t right = 0;
    uint32_t timeout;

    if (executor == NULL)
    {
        return out;
    }
    measure_speed(executor, now_ms, left_count, right_count);

    if ((executor->mode == MOTION_EXECUTOR_CONTINUOUS) &&
        time_reached(now_ms, executor->request.expires_ms))
    {
        executor->mode = MOTION_EXECUTOR_IDLE;   /* not refreshed: stop */
    }
    if (executor->mode == MOTION_EXECUTOR_GOAL)
    {
        timeout = (executor->goal.timeout_ms != 0U) ? executor->goal.timeout_ms
            : executor->config.default_goal_timeout_ms;
        if (((left_count - executor->start_left) >= executor->target_left) &&
            ((right_count - executor->start_right) >= executor->target_right))
        {
            finish_goal(executor, MOTION_RESULT_COMPLETED, left_count, right_count);
        }
        else if (time_reached(now_ms, executor->goal_started_ms + timeout))
        {
            finish_goal(executor, MOTION_RESULT_TIMEOUT, left_count, right_count);
        }
    }

    if (executor->mode == MOTION_EXECUTOR_CONTINUOUS)
    {
        continuous_effort(executor, &left, &right);
    }
    else if (executor->mode == MOTION_EXECUTOR_GOAL)
    {
        goal_effort(executor, left_count, right_count, &left, &right);
    }

    /* Direction change: one zero period first (drv_motor_apply rule). */
    if ((sign_of(left) * sign_of(executor->last_left)) < 0) { left = 0; }
    if ((sign_of(right) * sign_of(executor->last_right)) < 0) { right = 0; }

    out.left = clamp_effort(left, executor->config.max_effort);
    out.right = clamp_effort(right, right_limit(&executor->config));
    out.generation = executor->generation;
    out.active = executor->mode != MOTION_EXECUTOR_IDLE;
    executor->last_left = out.left;
    executor->last_right = out.right;
    return out;
}

bool motion_executor_goal_result(const motion_executor_t *executor,
    goal_id_t goal_id, motion_goal_result_t *result)
{
    if ((executor == NULL) || (result == NULL) || (goal_id == 0U))
    {
        return false;
    }
    if ((executor->mode == MOTION_EXECUTOR_GOAL) &&
        (executor->goal.goal_id == goal_id))
    {
        (void)memset(result, 0, sizeof(*result));
        result->goal_id = goal_id;
        result->authority_generation = executor->goal.authority_generation;
        result->code = MOTION_RESULT_PROGRESS;
        return true;
    }
    if (executor->result_valid && (executor->result.goal_id == goal_id))
    {
        *result = executor->result;
        return true;
    }
    return false;
}
