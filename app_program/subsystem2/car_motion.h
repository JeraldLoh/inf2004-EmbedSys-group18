#ifndef GROUP18_CAR_MOTION_H
#define GROUP18_CAR_MOTION_H
#include "measured_motion_executor.h"

/* Portable CAR_RUN compatibility boundary. Existing IF-05 curvature is micro
 * reciprocal metres, positive LEFT; measured executor uses whole reciprocal
 * metres, positive RIGHT. Conversion happens here, never in the wire codec.
 * Controller owns source/generation admission; the runtime checks its current
 * authority before invoking these methods. Calls are externally serialized. */
typedef enum {
    CAR_REJECT_NONE = 0, CAR_REJECT_SOURCE, CAR_REJECT_LIFETIME,
    CAR_REJECT_LINE_PAUSED, CAR_REJECT_ID, CAR_REJECT_AUTHORITY,
    CAR_REJECT_EXECUTOR, CAR_REJECT_ENCODER_CAPTURE
} car_motion_reject_t;
const char *car_motion_reject_name(car_motion_reject_t reason);

typedef struct
{
    measured_motion_executor_t executor;
    motion_goal_request_t external_goal;
    motion_goal_result_t retained;
    uint32_t next_command, next_goal;
    uint32_t last_bend_ms;
    int32_t last_bend_curvature;
    uint32_t trial_limit_us; /* Zero in normal production builds. */
    uint32_t last_goal_id[6];
    uint32_t line_floor;
    uint32_t accepted, rejected;
    car_motion_reject_t last_reject;
    motion_status_t last_reject_status;
    uint32_t rejected_id, rejected_generation;
    motion_source_t rejected_source;
    bool retained_valid, line_floor_set;
} car_motion_t;

/* Caller must hold the same external serialization as the motion methods. */
bool car_motion_record_rejection(car_motion_t *motion, car_motion_reject_t reason,
    motion_status_t status, motion_source_t source, uint32_t id, uint32_t generation);

/* Candidate configuration is deliberately unqualified, with zero gains.
 * Supply qualified measured gains before authorizing real movement. */
void car_motion_candidate_config(measured_motion_executor_config_t *config);
bool car_motion_init(car_motion_t *motion, drv_motor_t *motor,
    drv_encoder_t *encoder, const measured_motion_executor_config_t *config,
    const measured_motion_executor_io_t *io);
#if (defined(APP_B2_MOVE_TEST) && APP_B2_MOVE_TEST) || (defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL)
/* Startup-only opt-in, <=15s per authority or MEASURED_MOTION_NO_RUN_LIMIT. */
bool car_motion_enable_trial(car_motion_t *motion, uint32_t limit_us);
#endif
bool car_motion_continuous(car_motion_t *motion,
    const motion_continuous_request_t *request, uint32_t now_ms, uint32_t now_us);
bool car_motion_goal(car_motion_t *motion, const motion_goal_request_t *request,
    uint32_t now_us);
void car_motion_step(car_motion_t *motion);
/* Scope pause, not global STOP. Blocks old LINE work until new generation. */
void car_motion_halt(car_motion_t *motion);
void car_motion_cancel(car_motion_t *motion, motion_result_code_t code);
/* Controller changed mission/STOP generation: retire old work into the
 * retained result before accepting a request from the new authority. */
void car_motion_generation(car_motion_t *motion, uint32_t generation);
/* Retains terminal results until polled by their owner. Poll acknowledgement
 * frees the executor's reserved slot; retained copy remains until next goal. */
bool car_motion_result(car_motion_t *motion, motion_source_t source, goal_id_t id,
    motion_goal_result_t *result);
#endif
