#ifndef GROUP18_WHEEL_PID_H
#define GROUP18_WHEEL_PID_H
#include <stdbool.h>
#include <stdint.h>
typedef struct
{
    /* Gains x1000; output is actuator per-mille. Input physical units mm/s,
     * integral mm, derivative mm/s^2. D is on measured magnitude. */
    uint32_t kp_milli, ki_milli, kd_milli, feedforward_milli;
    uint16_t max_effort, integral_limit, launch_effort;
    uint32_t launch_us;
} wheel_pid_config_t;
typedef struct
{
    wheel_pid_config_t config;
    int64_t integral_um_us;
    uint32_t previous_speed;
    uint32_t running_us;
    bool primed;
} wheel_pid_t;
bool wheel_pid_init(wheel_pid_t *pid, const wheel_pid_config_t *config);
void wheel_pid_reset(wheel_pid_t *pid);
bool wheel_pid_step(wheel_pid_t *pid, uint32_t target_um_s,
    uint32_t actual_um_s, uint32_t dt_us, uint16_t *effort);
#endif
