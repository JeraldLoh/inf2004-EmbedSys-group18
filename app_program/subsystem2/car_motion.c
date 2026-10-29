#include "car_motion.h"
#include "motion_calibration.h"
#include <stddef.h>
#include <string.h>

static bool newer(uint32_t id, uint32_t old)
{ return id != old && id-old < 0x80000000UL; }

void car_motion_candidate_config(measured_motion_executor_config_t *c)
{
    if (c == NULL) { return; }
    (void)memset(c, 0, sizeof(*c));
    c->measurement = (wheel_measurement_config_t){MOTION_WHEEL_DIAMETER_MM_CANDIDATE,
        {MOTION_LEFT_BOTH_EDGE_COUNTS_PER_REV, MOTION_RIGHT_BOTH_EDGE_COUNTS_PER_REV},
        100000U, 200000U, 20000U};
    c->pid[0].max_effort = 350U; c->pid[0].integral_limit = 80U;
    c->pid[1] = c->pid[0];
    c->period_us = 20000U; c->watchdog_us = 100000U;
    c->track_width_mm = 112U; c->max_speed_mm_s = 150U;
    c->acceleration_um_s2 = 100000U;
    c->reversal_us = 100000U; c->settle_us = 100000U;
    c->settle_speed_um_s = 1000U; c->tolerance_um = 1000U; c->overshoot_um = 5000U;
    c->progress_timeout_us = 500000U; c->startup_grace_us = 500000U;
    c->goal_margin_ms = 2000U; c->max_goal_mm = 10000U;
    c->qualified = false;
}

bool car_motion_init(car_motion_t *m, drv_motor_t *motor,
    drv_encoder_t *encoder, const measured_motion_executor_config_t *c,
    const measured_motion_executor_io_t *io)
{
    if (m == NULL) { return false; }
    (void)memset(m, 0, sizeof(*m));
    return measured_motion_executor_init(&m->executor, motor, encoder, c, io);
}

static bool authority(car_motion_t *m, uint32_t generation)
{
    measured_motion_executor_t *e = &m->executor;
    if (generation == 0U) { return false; }
    if (generation == e->status.generation) { return e->status.authorized; }
    if (!newer(generation, e->status.generation)) { return false; }
    car_motion_generation(m, generation);
    return measured_motion_executor_authorize(e, generation) == MOTION_OK;
}

bool car_motion_continuous(car_motion_t *m,
    const motion_continuous_request_t *r, uint32_t ms, uint32_t us)
{
    motion_continuous_request_t converted;
    int64_t curvature;
    uint32_t life;
    if (m == NULL || r == NULL) { return false; }
    life = r->expires_ms-r->created_ms;
    if ((r->source != MOTION_SOURCE_LINE && r->source != MOTION_SOURCE_OBSTACLE) ||
        life == 0U || life > 1000U || ms-r->created_ms >= life ||
        (m->line_floor_set && r->source == MOTION_SOURCE_LINE &&
         !newer(r->authority_generation, m->line_floor))) { ++m->rejected; return false; }
    converted = *r;
    curvature = -(int64_t)r->curvature_u1_m;
    /* Nearest whole 1/m, symmetric rounding; INT32_MIN handled in int64. */
    converted.curvature_u1_m = (int32_t)((curvature + (curvature < 0 ? -500000 : 500000))/1000000);
    {
        int32_t limit = (int32_t)(2000U/m->executor.config.track_width_mm);
        if (converted.curvature_u1_m > limit) { converted.curvature_u1_m = limit; }
        if (converted.curvature_u1_m < -limit) { converted.curvature_u1_m = -limit; }
    }
    if (m->next_command == UINT32_MAX || !authority(m, r->authority_generation))
    { ++m->rejected; return false; }
    converted.command_id = ++m->next_command;
    if (measured_motion_executor_continuous(&m->executor, &converted, us,
        (r->expires_ms-ms)*1000U) != MOTION_OK) { ++m->rejected; return false; }
    ++m->accepted; return true;
}

bool car_motion_goal(car_motion_t *m, const motion_goal_request_t *r, uint32_t us)
{
    motion_goal_request_t converted;
    if (m == NULL || r == NULL) { return false; }
    if ((r->source != MOTION_SOURCE_JUNCTION && r->source != MOTION_SOURCE_OBSTACLE) ||
        r->goal_id == 0U || m->next_goal == UINT32_MAX ||
        (m->last_goal_id[r->source] != 0U && !newer(r->goal_id, m->last_goal_id[r->source])) ||
        !authority(m, r->authority_generation)) { ++m->rejected; return false; }
    converted = *r; converted.goal_id = ++m->next_goal;
    if (measured_motion_executor_goal(&m->executor, &converted, us, 1000000U) != MOTION_OK)
    { ++m->rejected; return false; }
    m->external_goal = *r; m->retained_valid = false;
    m->last_goal_id[r->source] = r->goal_id;
    ++m->accepted; return true;
}

void car_motion_step(car_motion_t *m)
{ if (m != NULL) { (void)measured_motion_executor_step(&m->executor); } }

void car_motion_halt(car_motion_t *m)
{
    motion_goal_result_t result;
    if (m == NULL) { return; }
    m->line_floor = m->executor.status.generation; m->line_floor_set = true;
    measured_motion_executor_pause(&m->executor);
    (void)car_motion_result(m, m->external_goal.source, m->external_goal.goal_id, &result);
}
void car_motion_cancel(car_motion_t *m, motion_result_code_t code)
{ if (m != NULL) { measured_motion_executor_cancel(&m->executor, code); } }

void car_motion_generation(car_motion_t *m, uint32_t generation)
{
    motion_goal_result_t result;
    if (m == NULL || generation == m->executor.status.generation) { return; }
    if (generation == 0U) { car_motion_cancel(m, MOTION_RESULT_FAULT); return; }
    if (m->executor.status.generation != 0U &&
        !newer(generation, m->executor.status.generation)) { return; }
    if (m->executor.status.authorized)
    { measured_motion_executor_cancel(&m->executor, MOTION_RESULT_CANCELLED); }
    /* The controller already revoked this scope; transfer the terminal from
     * the engine to the retained diagnostic copy even if its old producer
     * has abandoned the manoeuvre. A new scope must not deadlock on it. */
    (void)car_motion_result(m, m->external_goal.source, m->external_goal.goal_id, &result);
}

bool car_motion_result(car_motion_t *m, motion_source_t source, goal_id_t id, motion_goal_result_t *r)
{
    motion_goal_result_t terminal;
    if (m == NULL || r == NULL || id == 0U) { return false; }
    if (m->external_goal.goal_id != id || m->external_goal.source != source) { return false; }
    if (measured_motion_executor_result(&m->executor, &terminal))
    {
        terminal.goal_id = id;
        terminal.authority_generation = m->external_goal.authority_generation;
        m->retained = terminal; m->retained_valid = true;
        (void)measured_motion_executor_ack(&m->executor, m->executor.terminal.goal_id,
            m->executor.terminal.authority_generation);
    }
    if (m->retained_valid) { *r = m->retained; return true; }
    if (m->executor.status.state != MOTION_EXEC_GOAL) { return false; }
    *r = (motion_goal_result_t){id, m->external_goal.authority_generation,
        MOTION_RESULT_PROGRESS, 0, 0, false};
    return true;
}
