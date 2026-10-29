#ifndef GROUP18_CAR_MOTION_H
#define GROUP18_CAR_MOTION_H
#include "measured_motion_executor.h"

/* Portable CAR_RUN compatibility boundary. Existing IF-05 curvature is micro
 * reciprocal metres, positive LEFT; measured executor uses whole reciprocal
 * metres, positive RIGHT. Conversion happens here, never in the wire codec.
 * Controller owns source/generation admission; the runtime checks its current
 * authority before invoking these methods. Calls are externally serialized. */
typedef struct
{
    measured_motion_executor_t executor;
    motion_goal_request_t external_goal;
    motion_goal_result_t retained;
    uint32_t next_command, next_goal;
    uint32_t last_goal_id[6];
    uint32_t line_floor;
    uint32_t accepted, rejected;
    bool retained_valid, line_floor_set;
} car_motion_t;

/* Candidate configuration is deliberately unqualified, with zero gains.
 * Supply qualified measured gains before authorizing real movement. */
void car_motion_candidate_config(measured_motion_executor_config_t *config);
bool car_motion_init(car_motion_t *motion, drv_motor_t *motor,
    drv_encoder_t *encoder, const measured_motion_executor_config_t *config,
    const measured_motion_executor_io_t *io);
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
