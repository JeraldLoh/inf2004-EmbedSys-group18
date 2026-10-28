#include "b2_trial.h"
#include <stddef.h>

bool b2_trial_init(b2_trial_t *trial, drv_motor_t *motor,
    drv_encoder_t *encoder, vehicle_controller_t *controller,
    const b2_trial_io_t *io)
{
    if ((trial == NULL) || (motor == NULL) || (encoder == NULL) ||
        (controller == NULL) || (io == NULL) || (io->stop_pressed == NULL) ||
        (io->stop_pending == NULL) || (io->arm_cutoff == NULL) ||
        (io->cancel_cutoff == NULL)) { return false; }
    *trial = (b2_trial_t){0};
    trial->motor = motor;
    trial->encoder = encoder;
    trial->controller = controller;
    trial->io = *io;
    return true;
}

motion_status_t b2_trial_begin(b2_trial_t *trial, int16_t left_effort,
    int16_t right_effort, uint32_t duration_us)
{
    motion_motor_state_t state;
    motion_status_t result;
    controller_action_t action;
    if ((trial == NULL) || (trial->motor == NULL)) { return MOTION_INVALID; }
    if (trial->result_pending || (trial->result.state == B2_TRIAL_RUNNING))
    {
        return MOTION_STATE;
    }
    if ((duration_us == 0U) || (duration_us > B2_TRIAL_MAX_US) ||
        (left_effort < -B2_TRIAL_MAX_EFFORT) || (left_effort > B2_TRIAL_MAX_EFFORT) ||
        (right_effort < -B2_TRIAL_MAX_EFFORT) || (right_effort > B2_TRIAL_MAX_EFFORT) ||
        ((left_effort == 0) && (right_effort == 0)) ||
        (trial->result.trial_id == UINT32_MAX)) { return MOTION_INVALID; }
    if (trial->io.stop_pressed(trial->io.context) ||
        trial->io.stop_pending(trial->io.context)) { return MOTION_INHIBITED; }
    /* Only a released physical STOP may be cleared by this fresh gesture.
     * Never clear unrelated controller reasons or hardware failures. */
    if (!vehicle_controller_clear_stop(trial->controller, STOP_REASON_PHYSICAL) ||
        (vehicle_controller_hardware_faults(trial->controller) != 0U))
    {
        return MOTION_INHIBITED;
    }
    action = vehicle_controller_set_mode(trial->controller, SYSTEM_MODE_CALIBRATION);
    if (drv_motor_stop(trial->motor) != MOTION_OK) { return MOTION_IO_ERROR; }
    if (drv_encoder_capture(trial->encoder, trial->motor, &trial->start) != MOTION_OK)
    {
        return MOTION_UNAVAILABLE;
    }
    if (drv_motor_read(trial->motor, &state) != MOTION_OK) { return MOTION_STATE; }
    result = drv_motor_clear_inhibit(trial->motor, state.stop_sequence);
    if (result == MOTION_OK)
    {
        result = drv_motor_arm(trial->motor, state.stop_sequence,
            action.authority_generation);
    }
    /* Timer armed BEFORE any nonzero PWM. A racing STOP invalidates apply. */
    if ((result == MOTION_OK) &&
        !trial->io.arm_cutoff(trial->io.context, duration_us))
    {
        result = MOTION_INHIBITED;
    }
    if (result == MOTION_OK)
    {
        result = drv_motor_apply(trial->motor, action.authority_generation,
            left_effort, right_effort);
    }
    if (result != MOTION_OK)
    {
        trial->io.cancel_cutoff(trial->io.context);
        (void)drv_motor_stop(trial->motor);
        return result;
    }
    ++trial->result.trial_id;
    trial->result.generation = action.authority_generation;
    trial->duration_us = duration_us;
    trial->result.state = B2_TRIAL_RUNNING;
    return MOTION_OK;
}

void b2_trial_service(b2_trial_t *trial, stop_reason_mask_t stop_reasons,
    bool cutoff_fired)
{
    motion_encoder_snapshot_t end;
    bool stopped;
    if ((trial == NULL) || (trial->motor == NULL)) { return; }
    if (trial->result.state == B2_TRIAL_RUNNING)
    {
        /* Secondary task deadline; alarm0 still stops if this task/logging
         * stalls. No dependence on the task for the hardware cutoff. */
        if (drv_encoder_capture(trial->encoder, trial->motor, &end) != MOTION_OK)
        { stop_reasons |= STOP_REASON_CONTROLLER_FAULT; }
        else if ((end.acquired_us - trial->start.acquired_us) >= trial->duration_us)
        { cutoff_fired = true; }
    }
    if (trial->io.stop_pressed(trial->io.context)) { stop_reasons |= STOP_REASON_PHYSICAL; }
    if (stop_reasons != STOP_REASON_NONE)
    {
        (void)vehicle_controller_latch_stop(trial->controller, stop_reasons);
    }
    if ((stop_reasons == STOP_REASON_NONE) && !cutoff_fired) { return; }
    stopped = drv_motor_stop(trial->motor) == MOTION_OK;
    trial->io.cancel_cutoff(trial->io.context);
    if (trial->result.state != B2_TRIAL_RUNNING) { return; }
    trial->result.state = !stopped ? B2_TRIAL_FAILED :
        ((stop_reasons != STOP_REASON_NONE) ? B2_TRIAL_STOPPED : B2_TRIAL_TIME_ENDED);
    if (stop_reasons == STOP_REASON_NONE)
    {
        (void)vehicle_controller_set_mode(trial->controller, SYSTEM_MODE_IDLE);
    }
    trial->result.counts_valid =
        drv_encoder_capture(trial->encoder, trial->motor, &end) == MOTION_OK &&
        (end.reset_epoch == trial->start.reset_epoch);
    trial->result.left_edges = trial->result.counts_valid
        ? end.left_count - trial->start.left_count : 0U;
    trial->result.right_edges = trial->result.counts_valid
        ? end.right_count - trial->start.right_count : 0U;
    trial->result_pending = true;
}

bool b2_trial_take_result(b2_trial_t *trial, b2_trial_result_t *result)
{
    if ((trial == NULL) || (result == NULL) || !trial->result_pending) { return false; }
    *result = trial->result;
    trial->result_pending = false;
    return true;
}
