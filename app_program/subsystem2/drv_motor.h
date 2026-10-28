#ifndef GROUP18_DRV_MOTOR_H
#define GROUP18_DRV_MOTOR_H

#include "motion_hal.h"

/* Initial candidate, 20 kHz ONLY at 125 MHz PWM clock/divider 1. */
#define MOTION_PWM_WRAP_CANDIDATE 6249U
#define MOTION_EFFORT_MAX 1000

typedef struct
{
    bool ready;
    bool inhibited;
    bool armed;
    bool io_fault;
    bool sequence_exhausted;
    /* HAL accepted the low/SIO/disable sequence, NOT electrical feedback. */
    bool safe_command_applied;
    uint32_t stop_sequence;
    uint32_t authority_generation;
    uint32_t direction_epoch;
    int16_t left_effort;
    int16_t right_effort;
} motion_motor_state_t;

/* Zero-initialize once. Storage is public for static allocation, not mutation.
 * No copying/live rebinding. All subsequent state access uses driver methods.
 * No dynamic allocation, tasks, queues, or boot calibration movement. */
typedef struct
{
    motion_hal_t hal;
    motion_motor_state_t state;
    uint16_t wrap;
    bool bound;
    bool generation_seen;
} drv_motor_t;

motion_status_t drv_motor_init(drv_motor_t *motor, const motion_hal_t *hal,
    uint16_t wrap);
motion_status_t drv_motor_deinit(drv_motor_t *motor);
/* Callable from the agreed IRQ/task context; independent of ordinary queues. */
motion_status_t drv_motor_stop(drv_motor_t *motor);
motion_status_t drv_motor_read(drv_motor_t *motor, motion_motor_state_t *state);
/* Controller-owned seams. Clear is conditional on the observed STOP ticket;
 * it never arms. Arm needs a fresh generation and never writes nonzero PWM.
 * Generations must advance by 1..INT32_MAX within one boot/session. Root must
 * reject old boot/session commands BEFORE calling this low-level interface. */
motion_status_t drv_motor_clear_inhibit(drv_motor_t *motor,
    uint32_t observed_stop_sequence);
motion_status_t drv_motor_arm(drv_motor_t *motor,
    uint32_t observed_stop_sequence, uint32_t authority_generation);
/* Sole normal PWM path for the future Buddy 2 executor. Signed per-mille
 * actuator effort, NOT mm/s. A direct nonzero sign reversal stops/inhibits and
 * rejects; later control must schedule a zero/deadtime/fresh-start sequence. */
motion_status_t drv_motor_apply(drv_motor_t *motor,
    uint32_t authority_generation, int16_t left_effort, int16_t right_effort);

#endif
