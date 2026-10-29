#ifndef GROUP18_SUBSYSTEM2_MOTION_H
#define GROUP18_SUBSYSTEM2_MOTION_H

/* DRAFT for Buddy 2's review (written by Buddy 5 as integration lead).
 *
 * Buddy 2 motion executor task (CAR_RUN=1 only): the single motor writer.
 * Other tasks hand it a continuous request or a finite goal (IF-05) and read
 * goal results back (IF-06). The decision logic is motion_executor.c
 * (host-tested); this file is the kernel task around it.
 *
 * Every 5 ms it captures both encoders, steps the executor and applies the
 * result with drv_motor_apply(). While the wheels are driven it keeps
 * Buddy 2's hardware cutoff alarm armed (100 ms), so a stalled task stops the
 * motors by itself. Arming follows b2_trial_begin(): refused while STOP is
 * held or pending, fresh authority generation, clear inhibit, arm.
 *
 * Lifecycle: after subsystem2_init()/start(), subsystem2_motion_init() with
 * the other inits, subsystem2_motion_start() afterwards. Core 0 only.
 */
#include "../app.h"
#include "b2_trial.h"

typedef struct
{
    uint32_t sampled_ms;      /* kernel clock, updated only after capture succeeds */
    bool valid;               /* at least one successful producer sample */
    int16_t left_effort;
    int16_t right_effort;
    int32_t left_mm_s;        /* measured; sign from the command */
    int32_t right_mm_s;
    uint32_t mode;            /* motion_executor_mode_t */
    uint32_t armed_generation;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t driver_errors;
} subsystem2_motion_telemetry_t;

ER subsystem2_motion_init(const b2_trial_io_t *io);
ER subsystem2_motion_start(void);

/* Any task. Never block for long; false if rejected (invalid, stale, or the
 * executor is not running). */
bool subsystem2_motion_continuous(const motion_continuous_request_t *request);
bool subsystem2_motion_goal(const motion_goal_request_t *goal);

/* Any task. Zero output within one executor wake-up (the executor task has
 * the higher priority). Ends an active goal CANCELLED. For emergencies use
 * subsystem2_urgent_stop(), which latches the driver inhibit at once. */
void subsystem2_motion_halt(void);

/* PROGRESS while goal_id runs, then its final result. */
bool subsystem2_motion_goal_result(goal_id_t goal_id, motion_goal_result_t *result);

bool subsystem2_motion_telemetry(subsystem2_motion_telemetry_t *telemetry);

#endif
