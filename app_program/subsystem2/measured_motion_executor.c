#include "measured_motion_executor.h"
#include <stddef.h>
#include <string.h>

#define EXEC_MAX_MS 1800000U /* < half of the raw U32 microsecond domain. */
static bool reached(uint32_t now, uint32_t deadline)
{ return (now - deadline) < 0x80000000UL; }
static bool newer(uint32_t id, uint32_t old)
{ return id != 0U && (old == 0U || ((id - old) != 0U && (id - old) < 0x80000000UL)); }
static uint32_t magnitude(int32_t value) { return (uint32_t)(value < 0 ? -(int64_t)value : value); }

bool measured_motion_executor_init(measured_motion_executor_t *e, drv_motor_t *motor,
    drv_encoder_t *encoder, const measured_motion_executor_config_t *c,
    const measured_motion_executor_io_t *io)
{
    unsigned i;
    if ((e == NULL) || (motor == NULL) || (encoder == NULL) || (c == NULL) ||
        (io == NULL) || (io->stop_pending == NULL) || (io->arm_watchdog == NULL) ||
        (io->cancel_watchdog == NULL) || (c->period_us < 1000U) ||
        (c->period_us > 100000U) || (c->watchdog_us < 2U*c->period_us) ||
        (c->watchdog_us > 500000U) || (c->track_width_mm == 0U) ||
        (c->track_width_mm > 1000U) || (c->max_speed_mm_s == 0U) ||
        (c->max_speed_mm_s > 500U) || (c->acceleration_um_s2 == 0U) ||
        (c->acceleration_um_s2 > 2000000U) || (c->reversal_us < c->period_us) ||
        (c->reversal_us > 1000000U) || (c->settle_us < c->measurement.speed_window_us) ||
        (c->settle_us > 2000000U) || (c->settle_speed_um_s > 10000U) ||
        (c->tolerance_um == 0U) || (c->tolerance_um > 10000U) ||
        (c->overshoot_um < c->tolerance_um) || (c->overshoot_um > 50000U) ||
        (c->progress_timeout_us < c->measurement.speed_window_us) ||
        (c->progress_timeout_us > 5000000U) || (c->startup_grace_us > 5000000U) ||
        (c->goal_margin_ms < (c->settle_us+c->reversal_us)/1000U) ||
        (c->goal_margin_ms > 60000U) || (c->max_goal_mm == 0U) ||
        (c->max_goal_mm > 100000U)) { return false; }
    (void)memset(e, 0, sizeof(*e));
    if (!wheel_measurement_init(&e->measurement, &c->measurement)) { return false; }
    for (i = 0U; i < 2U; ++i)
    {
        if (!wheel_pid_init(&e->pid[i], &c->pid[i]) ||
            (c->qualified && c->pid[i].kp_milli == 0U && c->pid[i].ki_milli == 0U &&
             c->pid[i].feedforward_milli == 0U)) { return false; }
    }
    e->motor = motor; e->encoder = encoder; e->config = *c; e->io = *io;
    return true;
}

static motion_status_t stop_output(measured_motion_executor_t *e, bool inhibit)
{
    unsigned i;
    motion_status_t result = inhibit ? drv_motor_stop(e->motor) :
        drv_motor_apply(e->motor, e->status.generation, 0, 0);
    if (result != MOTION_OK)
    { (void)drv_motor_stop(e->motor); e->status.authorized = false; }
    e->io.cancel_watchdog(e->io.context);
    for (i = 0U; i < 2U; ++i)
    {
        wheel_pid_reset(&e->pid[i]); e->ramp_um_s[i] = 0U;
        e->status.target_um_s[i] = 0;
        e->ramp_remainder[i] = 0U;
    }
    return result;
}

static void finish(measured_motion_executor_t *e, motion_result_code_t code)
{
    bool reporting = e->status.state == MOTION_EXEC_GOAL;
    if (reporting)
    {
        uint64_t l = e->goal_started ? e->measurement.sample.travel_um[0] - e->goal_base_um[0] : 0U;
        uint64_t r = e->goal_started ? e->measurement.sample.travel_um[1] - e->goal_base_um[1] : 0U;
        e->terminal = (motion_goal_result_t){0};
        e->terminal.goal_id = e->goal.goal_id;
        e->terminal.authority_generation = e->goal.authority_generation;
        e->terminal.code = code;
        if (e->goal.type == MOTION_GOAL_FORWARD || e->goal.type == MOTION_GOAL_REVERSE)
        {
            e->terminal.travelled_mm = (int32_t)((l+r)/2000U);
            if (e->goal.type == MOTION_GOAL_REVERSE) { e->terminal.travelled_mm = -e->terminal.travelled_mm; }
        }
        else
        {
            e->terminal.turned_mdeg = (int32_t)((l+r)*180000000U /
                ((uint64_t)e->config.track_width_mm*3141593U));
            if (e->goal.type == MOTION_GOAL_PIVOT_RIGHT) { e->terminal.turned_mdeg = -e->terminal.turned_mdeg; }
        }
        e->terminal.odometry_valid = e->goal_started && e->measurement.sample.speed_valid;
        e->status.terminal_pending = true;
    }
    {
        motion_status_t stopped = stop_output(e, code != MOTION_RESULT_COMPLETED);
        if (code == MOTION_RESULT_COMPLETED && stopped != MOTION_OK)
        { code = stopped == MOTION_INHIBITED ? MOTION_RESULT_CANCELLED : MOTION_RESULT_FAULT; }
        if (reporting) { e->terminal.code = code; }
    }
    e->status.last_stop = code;
    if (code != MOTION_RESULT_COMPLETED) { e->status.authorized = false; }
    e->status.state = e->status.authorized ? MOTION_EXEC_IDLE : MOTION_EXEC_DISABLED;
    e->transition = false; e->settling = false;
}

void measured_motion_executor_cancel(measured_motion_executor_t *e, motion_result_code_t code)
{
    if (e != NULL && e->motor != NULL) { finish(e, code); }
}

motion_status_t measured_motion_executor_authorize(measured_motion_executor_t *e, uint32_t generation)
{
    motion_motor_state_t state;
    if ((e == NULL) || (e->motor == NULL)) { return MOTION_INVALID; }
    if (!e->config.qualified) { return MOTION_UNAVAILABLE; }
    if (e->status.terminal_pending || e->status.state == MOTION_EXEC_GOAL ||
        e->status.state == MOTION_EXEC_CONTINUOUS) { return MOTION_STATE; }
    if (!newer(generation, e->status.generation) || e->io.stop_pending(e->io.context))
    { return MOTION_STALE; }
    if (drv_motor_read(e->motor, &state) != MOTION_OK || !state.ready || state.io_fault || state.sequence_exhausted)
    { return MOTION_UNAVAILABLE; }
    e->permit_ticket = state.stop_sequence;
    e->status.generation = generation;
    e->status.authorized = true; e->status.state = MOTION_EXEC_IDLE;
    e->last_command_id = 0U; e->last_goal_id = 0U;
    return MOTION_OK; /* No arm, PWM or motion at authorization. */
}

motion_status_t measured_motion_executor_continuous(measured_motion_executor_t *e,
    const motion_continuous_request_t *r, uint32_t now, uint32_t ttl)
{
    if (e == NULL || r == NULL) { return MOTION_INVALID; }
    if (!e->status.authorized || r->authority_generation != e->status.generation)
    { return MOTION_INHIBITED; }
    if (e->status.terminal_pending || e->status.state == MOTION_EXEC_GOAL) { return MOTION_STATE; }
    if (!newer(r->command_id, e->last_command_id)) { return MOTION_STALE; }
    if (ttl == 0U || ttl > 1000000U || r->speed_mm_s < 0 ||
        (uint32_t)r->speed_mm_s > e->config.max_speed_mm_s ||
        magnitude(r->curvature_u1_m) > 2000U/e->config.track_width_mm)
    { return MOTION_INVALID; }
    if (e->status.state != MOTION_EXEC_CONTINUOUS)
    { e->transition = true; e->transition_until_us = now+e->config.reversal_us; }
    e->continuous = *r;
    e->last_command_id = r->command_id;
    e->status.command_id = r->command_id;
    e->continuous_until_us = now+ttl;
    e->status.state = MOTION_EXEC_CONTINUOUS;
    return MOTION_OK; /* Latest value replaces only continuous work. */
}

motion_status_t measured_motion_executor_goal(measured_motion_executor_t *e,
    const motion_goal_request_t *r, uint32_t now, uint32_t ttl)
{
    uint64_t target, nominal_ms, ramp_ms, needed_ms;
    if (e == NULL || r == NULL) { return MOTION_INVALID; }
    if (!e->status.authorized || r->authority_generation != e->status.generation)
    { return MOTION_INHIBITED; }
    if (e->status.terminal_pending || e->status.state == MOTION_EXEC_GOAL) { return MOTION_STATE; }
    if (!newer(r->goal_id, e->last_goal_id)) { return MOTION_STALE; }
    if (ttl == 0U || ttl > 1000000U || r->max_speed_mm_s == 0U ||
        r->max_speed_mm_s > e->config.max_speed_mm_s || (unsigned)r->type > MOTION_GOAL_PIVOT_RIGHT)
    { return MOTION_INVALID; }
    if (r->type == MOTION_GOAL_FORWARD || r->type == MOTION_GOAL_REVERSE)
    {
        if (r->target_mm <= 0 || (uint32_t)r->target_mm > e->config.max_goal_mm || r->target_mdeg != 0)
        { return MOTION_INVALID; }
        target = (uint64_t)r->target_mm*1000U;
    }
    else
    {
        if (r->target_mdeg <= 0 || r->target_mdeg > 360000 || r->target_mm != 0)
        { return MOTION_INVALID; }
        target = (uint64_t)e->config.track_width_mm*3141593U*(uint32_t)r->target_mdeg/360000000U;
    }
    if (target <= e->config.tolerance_um) { return MOTION_INVALID; }
    nominal_ms = (target+r->max_speed_mm_s-1U)/r->max_speed_mm_s;
    ramp_ms = ((uint64_t)r->max_speed_mm_s*1000000U+e->config.acceleration_um_s2-1U)/e->config.acceleration_um_s2;
    needed_ms = nominal_ms+2U*ramp_ms+e->config.goal_margin_ms;
    if (needed_ms > EXEC_MAX_MS || r->timeout_ms > EXEC_MAX_MS ||
        (r->timeout_ms != 0U && r->timeout_ms < needed_ms)) { return MOTION_INVALID; }
    e->goal = *r;
    e->goal.timeout_ms = r->timeout_ms == 0U ? (uint32_t)needed_ms : r->timeout_ms;
    e->goal_target_um = target; e->goal_started = false;
    e->last_goal_id = r->goal_id; e->status.command_id = r->goal_id;
    e->goal_until_us = now+e->goal.timeout_ms*1000U;
    e->admission_until_us = now+ttl;
    e->transition = true; e->transition_until_us = now+e->config.reversal_us;
    e->status.state = MOTION_EXEC_GOAL; e->settling = false;
    return MOTION_OK;
}

static bool driver_ready(measured_motion_executor_t *e)
{
    motion_motor_state_t s;
    if (drv_motor_read(e->motor, &s) != MOTION_OK || s.io_fault || !s.ready ||
        s.stop_sequence != e->permit_ticket || e->io.stop_pending(e->io.context)) { return false; }
    if (s.inhibited)
    {
        if (drv_motor_clear_inhibit(e->motor, e->permit_ticket) != MOTION_OK ||
            drv_motor_arm(e->motor, e->permit_ticket, e->status.generation) != MOTION_OK) { return false; }
    }
    return true;
}

motion_status_t measured_motion_executor_step(measured_motion_executor_t *e)
{
    motion_encoder_snapshot_t raw;
    uint32_t now, dt;
    uint32_t desired[2] = {0U, 0U};
    int8_t sign[2] = {1, 1};
    int16_t effort[2] = {0, 0};
    bool complete = true;
    unsigned i;
    if (e == NULL || e->motor == NULL) { return MOTION_INVALID; }
    if (drv_encoder_capture(e->encoder, e->motor, &raw) != MOTION_OK)
    { finish(e, MOTION_RESULT_FAULT); return MOTION_UNAVAILABLE; }
    now = raw.acquired_us;
    dt = e->primed ? now-e->last_step_us : e->config.period_us;
    if (e->primed && dt < e->config.period_us) { return MOTION_OK; }
    if (wheel_measurement_update(&e->measurement, &raw) != MOTION_OK)
    { finish(e, MOTION_RESULT_ABORTED); return MOTION_UNAVAILABLE; }
    e->last_step_us = now; e->primed = true;
    e->status.wheels = e->measurement.sample;
    if (e->status.state != MOTION_EXEC_CONTINUOUS && e->status.state != MOTION_EXEC_GOAL)
    {
        motion_motor_state_t idle;
        if (e->status.authorized && (drv_motor_read(e->motor, &idle) != MOTION_OK ||
            idle.stop_sequence != e->permit_ticket || e->io.stop_pending(e->io.context)))
        { finish(e, MOTION_RESULT_CANCELLED); }
        return MOTION_OK;
    }
    if (!driver_ready(e)) { finish(e, MOTION_RESULT_CANCELLED); return MOTION_INHIBITED; }
    if ((e->status.state == MOTION_EXEC_CONTINUOUS && reached(now, e->continuous_until_us)) ||
        (e->status.state == MOTION_EXEC_GOAL && reached(now, e->goal_until_us)))
    { finish(e, MOTION_RESULT_TIMEOUT); return MOTION_OK; }
    if (e->status.state == MOTION_EXEC_GOAL && !e->goal_started && reached(now, e->admission_until_us))
    { finish(e, MOTION_RESULT_REJECTED); return MOTION_OK; }
    if (e->transition)
    {
        if (drv_motor_apply(e->motor, e->status.generation, 0, 0) != MOTION_OK)
        { finish(e, MOTION_RESULT_FAULT); return MOTION_IO_ERROR; }
        if (!reached(now, e->transition_until_us) || !e->measurement.sample.speed_valid ||
            e->measurement.sample.speed_um_s[0] > e->config.settle_speed_um_s ||
            e->measurement.sample.speed_um_s[1] > e->config.settle_speed_um_s) { return MOTION_OK; }
        for (i = 0U; i < 2U; ++i)
        {
            e->goal_base_um[i] = e->measurement.sample.travel_um[i];
            e->progress_um[i] = 0U; e->progress_us[i] = now;
            e->ramp_um_s[i] = 0U; wheel_pid_reset(&e->pid[i]);
        }
        e->started_us = now; e->transition = false;
        if (e->status.state == MOTION_EXEC_GOAL) { e->goal_started = true; }
    }
    if (!e->measurement.sample.speed_valid) { return MOTION_OK; }
    if (e->status.state == MOTION_EXEC_CONTINUOUS)
    {
        uint32_t cap = e->config.max_speed_mm_s*1000U;
        int64_t speed = (int64_t)e->continuous.speed_mm_s*1000;
        int64_t correction = speed*e->continuous.curvature_u1_m*e->config.track_width_mm/2000;
        uint32_t largest;
        desired[0] = (uint32_t)(speed+correction);
        desired[1] = (uint32_t)(speed-correction);
        if (e->continuous.speed_cap_mm_s != 0U && e->continuous.speed_cap_mm_s < cap/1000U)
        { cap = e->continuous.speed_cap_mm_s*1000U; }
        largest = desired[0] > desired[1] ? desired[0] : desired[1];
        if (largest > cap)
        {
            desired[0] = (uint32_t)((uint64_t)desired[0]*cap/largest);
            desired[1] = (uint32_t)((uint64_t)desired[1]*cap/largest);
        }
    }
    else
    {
        if (e->goal.type == MOTION_GOAL_REVERSE) { sign[0] = -1; sign[1] = -1; }
        if (e->goal.type == MOTION_GOAL_PIVOT_LEFT) { sign[0] = -1; }
        if (e->goal.type == MOTION_GOAL_PIVOT_RIGHT) { sign[1] = -1; }
        for (i = 0U; i < 2U; ++i)
        {
            uint64_t p = e->measurement.sample.travel_um[i]-e->goal_base_um[i];
            uint64_t remain = p < e->goal_target_um ? e->goal_target_um-p : 0U;
            if (p > e->goal_target_um+e->config.overshoot_um)
            { finish(e, MOTION_RESULT_FAULT); return MOTION_OK; }
            if (p+e->config.tolerance_um < e->goal_target_um)
            {
                uint64_t slowed = remain*1000000U/500000U;
                desired[i] = e->goal.max_speed_mm_s*1000U;
                if (slowed < desired[i]) { desired[i] = (uint32_t)slowed; }
                complete = false;
            }
            if (p > e->goal_target_um+e->config.tolerance_um) { complete = false; }
        }
    }
    for (i = 0U; i < 2U; ++i)
    {
        uint64_t ramp_delta = (uint64_t)e->config.acceleration_um_s2*dt+e->ramp_remainder[i];
        uint32_t step = (uint32_t)(ramp_delta/1000000U);
        uint16_t duty;
        uint64_t travel = e->measurement.sample.travel_um[i];
        e->ramp_remainder[i] = (uint32_t)(ramp_delta%1000000U);
        if (desired[i] == 0U) { e->ramp_um_s[i] = 0U; }
        else if (e->ramp_um_s[i] < desired[i])
        { e->ramp_um_s[i] += (desired[i]-e->ramp_um_s[i] < step) ? desired[i]-e->ramp_um_s[i] : step; }
        else { e->ramp_um_s[i] = desired[i]; }
        if (!wheel_pid_step(&e->pid[i], e->ramp_um_s[i], e->measurement.sample.speed_um_s[i], dt, &duty))
        { finish(e, MOTION_RESULT_FAULT); return MOTION_UNAVAILABLE; }
        effort[i] = (int16_t)(sign[i]*(int32_t)duty);
        e->status.target_um_s[i] = sign[i]*(int32_t)e->ramp_um_s[i];
        if (travel != e->progress_um[i]) { e->progress_um[i] = travel; e->progress_us[i] = now; }
        if (desired[i] > 0U && duty > 0U && now-e->started_us > e->config.startup_grace_us &&
            now-e->progress_us[i] > e->config.progress_timeout_us)
        { finish(e, MOTION_RESULT_FAULT); return MOTION_OK; }
    }
    /* Every nonzero path checks the sticky inhibit in drv_motor_apply under
     * the same IRQ/core lock. Watchdog is always armed BEFORE the commit. */
    if (!e->io.arm_watchdog(e->io.context, e->config.watchdog_us) ||
        drv_motor_apply(e->motor, e->status.generation, effort[0], effort[1]) != MOTION_OK)
    { finish(e, MOTION_RESULT_FAULT); return MOTION_IO_ERROR; }
    if (e->status.state == MOTION_EXEC_GOAL && complete &&
        e->measurement.sample.speed_um_s[0] <= e->config.settle_speed_um_s &&
        e->measurement.sample.speed_um_s[1] <= e->config.settle_speed_um_s)
    {
        if (!e->settling) { e->settled_since_us = now; e->settling = true; }
        else if (now-e->settled_since_us >= e->config.settle_us) { finish(e, MOTION_RESULT_COMPLETED); }
    }
    else { e->settling = false; }
    return MOTION_OK;
}

bool measured_motion_executor_result(const measured_motion_executor_t *e, motion_goal_result_t *result)
{
    if (e == NULL || result == NULL || !e->status.terminal_pending) { return false; }
    *result = e->terminal; return true;
}
bool measured_motion_executor_ack(measured_motion_executor_t *e, goal_id_t id, authority_generation_t gen)
{
    if (e == NULL || !e->status.terminal_pending || e->terminal.goal_id != id ||
        e->terminal.authority_generation != gen) { return false; }
    e->status.terminal_pending = false; return true;
}
