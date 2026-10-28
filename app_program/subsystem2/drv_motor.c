#include "drv_motor.h"

#include <stddef.h>

#include "../project_config/pinmap.h"

static const uint8_t motor_pins[4] =
{
    PIN_MOTOR_LEFT_A, PIN_MOTOR_LEFT_B,
    PIN_MOTOR_RIGHT_A, PIN_MOTOR_RIGHT_B
};

static int sign_of(int16_t effort)
{
    return (effort > 0) - (effort < 0);
}

/* Always attempt every operation, including when an earlier one failed. CC
 * updates alone can remain buffered; low SIO latches + mux precede disable. */
static bool safe_locked(drv_motor_t *motor)
{
    bool ok = true;
    size_t i;
    for (i = 0U; i < 4U; ++i)
    {
        if (!motor->hal.pwm_compare(motor->hal.context, motor_pins[i], 0U))
        {
            ok = false;
        }
    }
    for (i = 0U; i < 4U; ++i)
    {
        if (!motor->hal.gpio_write(motor->hal.context, motor_pins[i], false))
        {
            ok = false;
        }
    }
    for (i = 0U; i < 4U; ++i)
    {
        if (!motor->hal.gpio_output(motor->hal.context, motor_pins[i]))
        {
            ok = false;
        }
    }
    if (!motor->hal.pwm_enable(motor->hal.context, PIN_MOTOR_LEFT_A, false))
    {
        ok = false;
    }
    if (!motor->hal.pwm_enable(motor->hal.context, PIN_MOTOR_RIGHT_A, false))
    {
        ok = false;
    }
    if ((motor->state.left_effort != 0) || (motor->state.right_effort != 0))
    {
        if (motor->state.direction_epoch == UINT32_MAX)
        {
            motor->state.sequence_exhausted = true;
            motor->state.inhibited = true;
            motor->state.armed = false;
            motor->state.ready = false;
        }
        else
        {
            ++motor->state.direction_epoch;
        }
    }
    motor->state.left_effort = 0;
    motor->state.right_effort = 0;
    motor->state.safe_command_applied = ok;
    if (!ok)
    {
        motor->state.io_fault = true;
        motor->state.ready = false;
    }
    return ok;
}

static bool stop_locked(drv_motor_t *motor)
{
    motor->state.inhibited = true;
    motor->state.armed = false;
    if (motor->state.stop_sequence == UINT32_MAX)
    {
        motor->state.sequence_exhausted = true;
    }
    else
    {
        ++motor->state.stop_sequence;
    }
    return safe_locked(motor);
}

motion_status_t drv_motor_init(drv_motor_t *motor, const motion_hal_t *hal,
    uint16_t wrap)
{
    motion_lock_key_t key;
    bool ok;
    motion_status_t result;
    if ((motor == NULL) || !motion_hal_is_valid(hal) ||
        (wrap == 0U) || (wrap == UINT16_MAX))
    {
        if ((motor != NULL) && motor->bound)
        {
            (void)drv_motor_stop(motor);
        }
        return MOTION_INVALID;
    }
    /* Binding is startup-only. Never replace a live HAL or reset its tokens. */
    if (motor->bound && ((motor->hal.context != hal->context) ||
        (motor->hal.enter != hal->enter) || (motor->hal.leave != hal->leave)))
    {
        (void)drv_motor_stop(motor);
        return MOTION_STATE;
    }
    if (!motor->bound)
    {
        motor->hal = *hal;
        motor->bound = true;
    }
    key = motor->hal.enter(motor->hal.context);
    if (motor->state.ready)
    {
        (void)stop_locked(motor);
        motor->hal.leave(motor->hal.context, key);
        return MOTION_STATE;
    }
    motor->state.io_fault = false;
    ok = stop_locked(motor);
    motor->wrap = wrap;
    if (ok && !motor->hal.pwm_wrap(motor->hal.context, PIN_MOTOR_LEFT_A, wrap))
    {
        ok = false;
    }
    if (ok && !motor->hal.pwm_wrap(motor->hal.context, PIN_MOTOR_RIGHT_A, wrap))
    {
        ok = false;
    }
    if (!ok)
    {
        motor->state.io_fault = true;
        (void)safe_locked(motor);
    }
    motor->state.ready = ok && !motor->state.sequence_exhausted;
    result = ok ? (motor->state.ready ? MOTION_OK : MOTION_STATE) : MOTION_IO_ERROR;
    motor->hal.leave(motor->hal.context, key);
    return result;
}

motion_status_t drv_motor_stop(drv_motor_t *motor)
{
    motion_lock_key_t key;
    bool ok;
    if ((motor == NULL) || !motor->bound)
    {
        return MOTION_STATE;
    }
    key = motor->hal.enter(motor->hal.context);
    ok = stop_locked(motor);
    motor->hal.leave(motor->hal.context, key);
    return ok ? MOTION_OK : MOTION_IO_ERROR;
}

motion_status_t drv_motor_deinit(drv_motor_t *motor)
{
    motion_lock_key_t key;
    bool ok;
    if ((motor == NULL) || !motor->bound)
    {
        return MOTION_STATE;
    }
    key = motor->hal.enter(motor->hal.context);
    ok = stop_locked(motor);
    motor->state.ready = false;
    motor->hal.leave(motor->hal.context, key);
    return ok ? MOTION_OK : MOTION_IO_ERROR;
}

motion_status_t drv_motor_read(drv_motor_t *motor, motion_motor_state_t *state)
{
    motion_lock_key_t key;
    if ((motor == NULL) || !motor->bound || (state == NULL))
    {
        return MOTION_INVALID;
    }
    key = motor->hal.enter(motor->hal.context);
    *state = motor->state;
    motor->hal.leave(motor->hal.context, key);
    return MOTION_OK;
}

motion_status_t drv_motor_clear_inhibit(drv_motor_t *motor,
    uint32_t observed_stop_sequence)
{
    motion_lock_key_t key;
    motion_status_t result = MOTION_OK;
    if ((motor == NULL) || !motor->bound)
    {
        return MOTION_STATE;
    }
    key = motor->hal.enter(motor->hal.context);
    if (!motor->state.ready || motor->state.io_fault ||
        motor->state.sequence_exhausted || motor->state.armed)
    {
        result = MOTION_STATE;
    }
    else if (observed_stop_sequence != motor->state.stop_sequence)
    {
        result = MOTION_STALE;
    }
    else
    {
        motor->state.inhibited = false;
    }
    motor->hal.leave(motor->hal.context, key);
    return result;
}

motion_status_t drv_motor_arm(drv_motor_t *motor,
    uint32_t observed_stop_sequence, uint32_t authority_generation)
{
    motion_lock_key_t key;
    motion_status_t result = MOTION_OK;
    uint32_t delta;
    if ((motor == NULL) || !motor->bound)
    {
        return MOTION_STATE;
    }
    key = motor->hal.enter(motor->hal.context);
    delta = authority_generation - motor->state.authority_generation;
    if (!motor->state.ready || motor->state.io_fault ||
        motor->state.sequence_exhausted || motor->state.armed)
    {
        result = MOTION_STATE;
    }
    else if (motor->state.inhibited)
    {
        result = MOTION_INHIBITED;
    }
    else if ((observed_stop_sequence != motor->state.stop_sequence) ||
        (authority_generation == 0U) ||
        (motor->generation_seen && ((delta == 0U) || (delta >= 0x80000000UL))))
    {
        result = MOTION_STALE;
    }
    else
    {
        motor->generation_seen = true;
        motor->state.authority_generation = authority_generation;
        motor->state.armed = true;
    }
    motor->hal.leave(motor->hal.context, key);
    return result;
}

motion_status_t drv_motor_apply(drv_motor_t *motor,
    uint32_t authority_generation, int16_t left_effort, int16_t right_effort)
{
    motion_lock_key_t key;
    motion_status_t result = MOTION_OK;
    int16_t efforts[2];
    uint16_t compares[4] = {0U, 0U, 0U, 0U};
    bool ok = true;
    size_t i;
    if ((motor == NULL) || !motor->bound)
    {
        return MOTION_STATE;
    }
    key = motor->hal.enter(motor->hal.context);
    if ((left_effort < -MOTION_EFFORT_MAX) || (left_effort > MOTION_EFFORT_MAX) ||
        (right_effort < -MOTION_EFFORT_MAX) || (right_effort > MOTION_EFFORT_MAX))
    {
        (void)stop_locked(motor);
        result = MOTION_INVALID;
    }
    else if (motor->state.inhibited)
    {
        result = MOTION_INHIBITED;
    }
    else if (!motor->state.ready || !motor->state.armed)
    {
        result = MOTION_STATE;
    }
    else if (authority_generation != motor->state.authority_generation)
    {
        result = MOTION_STALE;
    }
    else if (((left_effort * (int32_t)motor->state.left_effort) < 0) ||
        ((right_effort * (int32_t)motor->state.right_effort) < 0))
    {
        (void)stop_locked(motor);
        result = MOTION_REVERSAL_REQUIRED;
    }
    else if ((left_effort == 0) && (right_effort == 0))
    {
        ok = safe_locked(motor);
    }
    else
    {
        if ((sign_of(left_effort) != sign_of(motor->state.left_effort)) ||
            (sign_of(right_effort) != sign_of(motor->state.right_effort)))
        {
            if (motor->state.direction_epoch == UINT32_MAX)
            {
                motor->state.sequence_exhausted = true;
                (void)stop_locked(motor);
                result = MOTION_STATE;
            }
            else
            {
                ++motor->state.direction_epoch;
            }
        }
        if (result == MOTION_OK)
        {
            efforts[0] = left_effort;
            efforts[1] = right_effort;
            for (i = 0U; i < 2U; ++i)
            {
                uint32_t magnitude = (uint32_t)((efforts[i] < 0)
                    ? -efforts[i] : efforts[i]);
                compares[(2U * i) + ((efforts[i] < 0) ? 1U : 0U)] =
                    (uint16_t)((((uint32_t)motor->wrap + 1U) * magnitude) / 1000U);
            }
            for (i = 0U; (i < 4U) && ok; ++i)
            {
                ok = motor->hal.pwm_compare(motor->hal.context, motor_pins[i],
                    compares[i]);
            }
            for (i = 0U; (i < 4U) && ok; ++i)
            {
                ok = motor->hal.pwm_function(motor->hal.context, motor_pins[i]);
            }
            if (ok)
            {
                ok = motor->hal.pwm_enable(motor->hal.context,
                    PIN_MOTOR_LEFT_A, true);
            }
            if (ok)
            {
                ok = motor->hal.pwm_enable(motor->hal.context,
                    PIN_MOTOR_RIGHT_A, true);
            }
            if (ok)
            {
                motor->state.left_effort = left_effort;
                motor->state.right_effort = right_effort;
                motor->state.safe_command_applied = false;
            }
        }
    }
    if (!ok || motor->state.io_fault)
    {
        motor->state.io_fault = true;
        motor->state.ready = false;
        (void)stop_locked(motor);
        result = MOTION_IO_ERROR;
    }
    motor->hal.leave(motor->hal.context, key);
    return result;
}
