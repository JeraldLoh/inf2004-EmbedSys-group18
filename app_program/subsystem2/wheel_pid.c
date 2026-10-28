#include "wheel_pid.h"
#include <stddef.h>
#include <string.h>

bool wheel_pid_init(wheel_pid_t *p, const wheel_pid_config_t *c)
{
    if ((p == NULL) || (c == NULL) || (c->kp_milli > 1000000U) ||
        (c->ki_milli > 1000000U) || (c->kd_milli > 1000000U) ||
        (c->feedforward_milli > 1000000U) || (c->max_effort == 0U) ||
        (c->max_effort > 1000U) || (c->integral_limit > c->max_effort) ||
        (c->launch_effort > c->max_effort) || (c->launch_us > 500000U)) { return false; }
    (void)memset(p, 0, sizeof(*p)); p->config = *c; return true;
}
void wheel_pid_reset(wheel_pid_t *p)
{
    p->integral_um_us = 0; p->running_us = 0U; p->primed = false;
}
bool wheel_pid_step(wheel_pid_t *p, uint32_t target, uint32_t actual,
    uint32_t dt, uint16_t *effort)
{
    int64_t error, proportional, derivative = 0, feedforward, integral, candidate, limit, output;
    if ((p == NULL) || (effort == NULL) || (dt < 1000U) || (dt > 1000000U) ||
        (target > 500000U) || (actual > 2000000U)) { return false; }
    if (target == 0U) { wheel_pid_reset(p); *effort = 0U; return true; }
    error = (int64_t)target - actual;
    proportional = error * p->config.kp_milli / 1000000;
    feedforward = (int64_t)target * p->config.feedforward_milli / 1000000;
    if (p->primed)
    {
        derivative = ((int64_t)p->previous_speed - actual) * 1000000 / dt;
        derivative = derivative * p->config.kd_milli / 1000000;
    }
    candidate = p->integral_um_us;
    if (p->config.ki_milli != 0U)
    {
        limit = (int64_t)p->config.integral_limit * 1000000000000LL / p->config.ki_milli;
        candidate += error * dt;
        if (candidate > limit) { candidate = limit; }
        if (candidate < -limit) { candidate = -limit; }
    }
    else { candidate = 0; }
    integral = candidate * p->config.ki_milli / 1000000000000LL;
    output = feedforward + proportional + integral + derivative;
    /* Conditional integration: saturation cannot wind I farther into a rail. */
    if (((output > p->config.max_effort) && (error > 0)) || ((output < 0) && (error < 0)))
    {
        integral = p->integral_um_us * p->config.ki_milli / 1000000000000LL;
        output = feedforward + proportional + integral + derivative;
    }
    else { p->integral_um_us = candidate; }
    if ((p->running_us < p->config.launch_us) && (actual < 1000U) &&
        (output < p->config.launch_effort)) { output = p->config.launch_effort; }
    if (output < 0) { output = 0; }
    if (output > p->config.max_effort) { output = p->config.max_effort; }
    if (p->running_us < 1000000U) { p->running_us += dt; }
    p->previous_speed = actual; p->primed = true;
    *effort = (uint16_t)output; return true;
}
