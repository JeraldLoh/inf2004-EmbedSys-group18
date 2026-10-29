#ifndef GROUP18_SUBSYSTEM2_MOTION_H
#define GROUP18_SUBSYSTEM2_MOTION_H

/* CAR_RUN measured wheel controller.
 *
 * Buddy 2 motion executor task (CAR_RUN=1 only): the single motor writer.
 * Other tasks hand it a continuous request or a finite goal (IF-05) and read
 * goal results back (IF-06). car_motion.c converts legacy steering units and
 * source-scopes result IDs; measured_motion_executor.c owns wheel PID/goals.
 *
 * Every 20 ms it captures both encoders and steps the measured executor.
 * While the wheels are driven it keeps
 * Buddy 2's hardware cutoff alarm armed (100 ms), so a stalled task stops the
 * motors by itself. Arming follows b2_trial_begin(): refused while STOP is
 * held or pending, fresh authority generation, clear inhibit, arm.
 *
 * Lifecycle: after subsystem2_init(), before subsystem2_start(), init with
 * the other inits, subsystem2_motion_start() afterwards. Core 0 only.
 */
#include "../app.h"
#include "b2_trial.h"

typedef struct
{
    int16_t left_effort;
    int16_t right_effort;
    int32_t left_mm_s;        /* measured magnitude, B-only encoders */
    int32_t right_mm_s;
    uint32_t mode;            /* measured_motion_executor_state_t */
    uint32_t armed_generation;
    uint32_t accepted;
    uint32_t rejected;
    uint32_t driver_errors;
    int32_t left_target_mm_s;
    int32_t right_target_mm_s;
    bool speed_valid;
    bool qualified;
} subsystem2_motion_telemetry_t;

ER subsystem2_motion_init(const b2_trial_io_t *io);
ER subsystem2_motion_start(void);
bool subsystem2_motion_qualified(void);

/* Any task. Never block for long; false if rejected (invalid, stale, or the
 * executor is not running). */
bool subsystem2_motion_continuous(const motion_continuous_request_t *request);
bool subsystem2_motion_goal(const motion_goal_request_t *goal);

/* Any task. Zero output within one executor wake-up (the executor task has
 * the higher priority). Ends an active goal CANCELLED. For emergencies use
 * subsystem2_urgent_stop(), which latches the driver inhibit at once. */
void subsystem2_motion_halt(void);

/* Source-qualified PROGRESS while goal_id runs, then its final result.
 * Junction and obstacle producers may reuse the same external numeric ID. */
bool subsystem2_motion_goal_result_for(motion_source_t source, goal_id_t goal_id,
    motion_goal_result_t *result);

bool subsystem2_motion_telemetry(subsystem2_motion_telemetry_t *telemetry);

#endif
