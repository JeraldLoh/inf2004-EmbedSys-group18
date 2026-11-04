#include "b5_motion.h"

#include <stddef.h>
#include <string.h>

static bool time_reached(uint32_t now, uint32_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

bool b5_motion_ops_present(const b5_motion_ops_t *ops)
{
    return (ops != NULL) && (ops->authority != NULL) && (ops->goal != NULL) &&
        (ops->continuous != NULL) && (ops->goal_result != NULL) &&
        (ops->halt != NULL) && (ops->line_seen != NULL) &&
        (ops->line_reacquire != NULL);
}

bool b5_motion_ops_valid(const b5_motion_ops_t *ops)
{
    bool any;

    if (ops == NULL)
    {
        return false;
    }
    any = (ops->authority != NULL) || (ops->goal != NULL) ||
        (ops->continuous != NULL) || (ops->goal_result != NULL) ||
        (ops->halt != NULL) || (ops->line_seen != NULL) ||
        (ops->line_reacquire != NULL);
    return !any || b5_motion_ops_present(ops);
}

void b5_motion_default_config(b5_motion_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    /* Same speed as the trial line cruise (50 mm/s), so avoidance never
     * drives faster than line following. Buddy 2's measured executor holds
     * this as a wheel speed and applies its own minimum pivot/drive speeds. */
    config->goal_speed_mm_s = 50U;
    config->goal_timeout_ms = 0U; /* Measured B2 derives a finite path-dependent bound. */
    config->wait_ms = 300U;
    config->search_speed_mm_s = 50;
    config->search_curvature_u1_m = 3000000;   /* ~33 cm radius */
    config->search_life_ms = 150U;
    /* Car trial 2026-11-02: pivots stalled at the 30% effort cap (the wheel
     * going backwards never moved; NO_WHEEL_PROGRESS twice). Arc instead at
     * 11 /m (R ~9 cm): wheels ~19/81 mm/s, the same split line following
     * uses at that cap. 50 mm/s x 11 /m = ~31.5 deg/s. */
    /* Back to pivots (2026-11-02, later): arcs carried the car well off the
     * line and the arc's slow inner wheel stalled at 19 mm/s. Pivots run at
     * the trial's 30% cap; Buddy 2 stops both wheels together when one lags
     * slightly. The arc settings stay for arc_turns = true. */
    config->arc_turns = false;
    config->turn_speed_mm_s = 50;
    config->turn_curvature_u1_m = 11000000;
    config->turn_lag_ms = 200U;   /* 50 mm/s at 200 mm/s^2 ramps in 0.25 s */
}

/* Arc time for a turn: angle / (speed x curvature), plus the ramp lag. */
static uint32_t arc_time_ms(const b5_motion_config_t *c, int32_t mdeg)
{
    uint64_t deg_per_s_x1e3;
    uint32_t mag = (uint32_t)((mdeg < 0) ? -mdeg : mdeg);

    /* rad/s = (mm/s / 1000) * (u1/m / 1e6); deg = rad * 57296 / 1000 */
    deg_per_s_x1e3 = ((uint64_t)c->turn_speed_mm_s * (uint64_t)c->turn_curvature_u1_m
        * 57296ULL) / 1000000000ULL;
    if (deg_per_s_x1e3 == 0U)
    {
        return 0U;
    }
    return (uint32_t)(((uint64_t)mag * 1000ULL) / deg_per_s_x1e3) + c->turn_lag_ms;
}

void b5_motion_init(b5_motion_t *motion, const b5_motion_ops_t *ops,
    const b5_motion_config_t *config)
{
    if (motion == NULL)
    {
        return;
    }
    (void)memset(motion, 0, sizeof(*motion));
    if (ops != NULL)
    {
        motion->ops = *ops;
    }
    if (config != NULL)
    {
        motion->config = *config;
    }
    else
    {
        b5_motion_default_config(&motion->config);
    }
}

static void start_goal(b5_motion_t *motion, motion_goal_type_t type,
    int32_t mm, int32_t mdeg, uint32_t now_ms)
{
    (void)memset(&motion->goal, 0, sizeof(motion->goal));
    motion->goal.goal_id = ++motion->next_id;
    if (motion->goal.goal_id == 0U)
    {
        motion->goal.goal_id = ++motion->next_id;   /* 0 is "no goal" */
    }
    motion->goal.source = MOTION_SOURCE_OBSTACLE;
    motion->goal.type = type;
    motion->goal.target_mm = mm;
    motion->goal.target_mdeg = mdeg;
    motion->goal.max_speed_mm_s = motion->config.goal_speed_mm_s;
    motion->goal.timeout_ms = motion->config.goal_timeout_ms;
    motion->deadline_ms = now_ms + motion->config.wait_ms;
    motion->state = B5_MOVE_PENDING;
}

void b5_motion_turn(b5_motion_t *motion, int32_t mdeg, uint32_t now_ms)
{
    if (motion == NULL)
    {
        return;
    }
    if (mdeg == 0)
    {
        motion->state = B5_MOVE_DONE;
        return;
    }
    if (motion->config.arc_turns)
    {
        motion->arc_left = mdeg > 0;
        motion->arc_started = false;
        motion->arc_ms = arc_time_ms(&motion->config, mdeg);
        motion->deadline_ms = now_ms + motion->config.wait_ms;
        motion->state = B5_MOVE_ARC;
        return;
    }
    start_goal(motion, (mdeg > 0) ? MOTION_GOAL_PIVOT_LEFT : MOTION_GOAL_PIVOT_RIGHT,
        0, (mdeg > 0) ? mdeg : -mdeg, now_ms);
}

void b5_motion_straight(b5_motion_t *motion, int32_t mm, bool reverse,
    uint32_t now_ms)
{
    if (motion == NULL)
    {
        return;
    }
    if (mm <= 0)
    {
        motion->state = B5_MOVE_DONE;
        return;
    }
    start_goal(motion, reverse ? MOTION_GOAL_REVERSE : MOTION_GOAL_FORWARD,
        mm, 0, now_ms);
}

void b5_motion_search(b5_motion_t *motion, bool toward_left)
{
    if (motion == NULL)
    {
        return;
    }
    motion->search_left = toward_left;
    motion->ops.line_reacquire(toward_left);
    motion->state = B5_MOVE_SEARCH;
}

void b5_motion_stop(b5_motion_t *motion)
{
    if (motion == NULL)
    {
        return;
    }
    motion->state = B5_MOVE_NONE;
    motion->ops.halt();
}

void b5_motion_resume(b5_motion_t *motion)
{
    if (motion != NULL)
    {
        motion->state = B5_MOVE_NONE;
    }
}

static void service_pending(b5_motion_t *motion, uint32_t now_ms)
{
    authority_generation_t generation;

    if (motion->ops.authority(MOTION_SOURCE_OBSTACLE, &generation))
    {
        motion->goal.authority_generation = generation;
        if (motion->ops.goal(&motion->goal))
        {
            motion->state = B5_MOVE_GOAL;
            return;
        }
    }
    if (time_reached(now_ms, motion->deadline_ms))
    {
        motion->last_result = MOTION_RESULT_REJECTED;
        motion->state = B5_MOVE_FAILED;
    }
}

static void service_goal(b5_motion_t *motion)
{
    motion_goal_result_t result;

    if (!motion->ops.goal_result(motion->goal.goal_id, &result))
    {
        motion->last_result = MOTION_RESULT_FAULT;   /* executor lost it */
        motion->state = B5_MOVE_FAILED;
        return;
    }
    if ((result.code == MOTION_RESULT_PROGRESS) ||
        (result.code == MOTION_RESULT_ACCEPTED))
    {
        return;
    }
    motion->last_result = result.code;
    motion->state = (result.code == MOTION_RESULT_COMPLETED)
        ? B5_MOVE_DONE : B5_MOVE_FAILED;
}

static void service_search(b5_motion_t *motion, uint32_t now_ms)
{
    motion_continuous_request_t request;
    authority_generation_t generation;

    if (!motion->ops.authority(MOTION_SOURCE_OBSTACLE, &generation))
    {
        return;   /* the avoidance logic's own search timeout applies */
    }
    (void)memset(&request, 0, sizeof(request));
    request.authority_generation = generation;
    request.source = MOTION_SOURCE_OBSTACLE;
    request.speed_mm_s = motion->config.search_speed_mm_s;
    request.curvature_u1_m = motion->search_left
        ? motion->config.search_curvature_u1_m
        : -motion->config.search_curvature_u1_m;
    request.created_ms = now_ms;
    request.expires_ms = now_ms + motion->config.search_life_ms;
    (void)motion->ops.continuous(&request);
}

static bool send_arc(b5_motion_t *motion, uint32_t now_ms)
{
    motion_continuous_request_t request;
    authority_generation_t generation;

    if (!motion->ops.authority(MOTION_SOURCE_OBSTACLE, &generation))
    {
        return false;
    }
    (void)memset(&request, 0, sizeof(request));
    request.authority_generation = generation;
    request.source = MOTION_SOURCE_OBSTACLE;
    request.speed_mm_s = motion->config.turn_speed_mm_s;
    request.curvature_u1_m = motion->arc_left
        ? motion->config.turn_curvature_u1_m
        : -motion->config.turn_curvature_u1_m;
    request.created_ms = now_ms;
    request.expires_ms = now_ms + motion->config.search_life_ms;
    return motion->ops.continuous(&request);
}

static void service_arc(b5_motion_t *motion, uint32_t now_ms)
{
    if (!motion->arc_started)
    {
        /* Same authority wait as a goal: see the header. */
        if (send_arc(motion, now_ms))
        {
            motion->arc_started = true;
            motion->deadline_ms = now_ms + motion->arc_ms;
        }
        else if (time_reached(now_ms, motion->deadline_ms))
        {
            motion->last_result = MOTION_RESULT_REJECTED;
            motion->state = B5_MOVE_FAILED;
        }
        return;
    }
    if (time_reached(now_ms, motion->deadline_ms))
    {
        motion->ops.halt();
        motion->last_result = MOTION_RESULT_COMPLETED;
        motion->state = B5_MOVE_DONE;
        return;
    }
    if (!send_arc(motion, now_ms))
    {
        /* Authority withdrawn or Buddy 2 refused mid-arc (STOP, fault). */
        motion->ops.halt();
        motion->last_result = MOTION_RESULT_CANCELLED;
        motion->state = B5_MOVE_FAILED;
    }
}

void b5_motion_service(b5_motion_t *motion, uint32_t now_ms)
{
    if ((motion == NULL) || !b5_motion_ops_present(&motion->ops))
    {
        return;
    }
    switch (motion->state)
    {
        case B5_MOVE_PENDING: service_pending(motion, now_ms); break;
        case B5_MOVE_GOAL: service_goal(motion); break;
        case B5_MOVE_SEARCH: service_search(motion, now_ms); break;
        case B5_MOVE_ARC: service_arc(motion, now_ms); break;
        default: break;
    }
}

bool b5_motion_done(const b5_motion_t *motion)
{
    return (motion != NULL) &&
        ((motion->state == B5_MOVE_NONE) || (motion->state == B5_MOVE_DONE));
}

bool b5_motion_failed(const b5_motion_t *motion)
{
    return (motion != NULL) && (motion->state == B5_MOVE_FAILED);
}

bool b5_motion_line_seen(const b5_motion_t *motion)
{
    return (motion != NULL) && (motion->ops.line_seen != NULL) &&
        motion->ops.line_seen();
}
