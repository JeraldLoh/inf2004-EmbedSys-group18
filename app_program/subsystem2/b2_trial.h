#ifndef GROUP18_B2_TRIAL_H
#define GROUP18_B2_TRIAL_H
#include "drv_encoder.h"
#include "../vehicle_controller/vehicle_controller.h"

#define B2_TRIAL_MAX_EFFORT 150
#define B2_TRIAL_MAX_US 500000U
typedef struct
{
    void *context;
    bool (*stop_pressed)(void *context);
    bool (*stop_pending)(void *context);
    bool (*arm_cutoff)(void *context, uint32_t duration_us);
    void (*cancel_cutoff)(void *context);
} b2_trial_io_t;
typedef enum
{
    B2_TRIAL_IDLE = 0,
    B2_TRIAL_RUNNING,
    B2_TRIAL_TIME_ENDED,
    B2_TRIAL_STOPPED,
    B2_TRIAL_FAILED
} b2_trial_state_t;
typedef struct
{
    uint32_t trial_id;
    uint32_t generation;
    uint32_t left_edges;
    uint32_t right_edges;
    b2_trial_state_t state;
    bool counts_valid;
} b2_trial_result_t;
typedef struct
{
    drv_motor_t *motor;
    drv_encoder_t *encoder;
    vehicle_controller_t *controller;
    b2_trial_io_t io;
    motion_encoder_snapshot_t start;
    uint32_t duration_us;
    b2_trial_result_t result;
    bool result_pending;
} b2_trial_t;

bool b2_trial_init(b2_trial_t *trial, drv_motor_t *motor,
    drv_encoder_t *encoder, vehicle_controller_t *controller,
    const b2_trial_io_t *io);
/* Call only on a fresh manual gesture; no auto retry/queue. One retained result
 * must be consumed before another trial. Counts are rotation, not distance. */
motion_status_t b2_trial_begin(b2_trial_t *trial, int16_t left_effort,
    int16_t right_effort, uint32_t duration_us);
/* ISR evidence is reconciled into controller ONLY in this task-context call. */
void b2_trial_service(b2_trial_t *trial, stop_reason_mask_t stop_reasons,
    bool cutoff_fired);
bool b2_trial_take_result(b2_trial_t *trial, b2_trial_result_t *result);
#endif
