#include "drv_encoder.h"

#include <stddef.h>
#include <string.h>

#include "../project_config/pinmap.h"

motion_status_t drv_encoder_init(drv_encoder_t *encoder,
    const motion_hal_t *hal)
{
    motion_lock_key_t key;
    bool ok;
    if ((encoder == NULL) || !motion_hal_is_valid(hal))
    {
        return MOTION_INVALID;
    }
    if (encoder->bound && ((encoder->hal.context != hal->context) ||
        (encoder->hal.enter != hal->enter) || (encoder->hal.leave != hal->leave)))
    {
        return MOTION_STATE;
    }
    if (!encoder->bound)
    {
        encoder->hal = *hal;
        encoder->bound = true;
    }
    key = encoder->hal.enter(encoder->hal.context);
    if (encoder->ready || (encoder->sample.reset_epoch == UINT32_MAX))
    {
        encoder->hal.leave(encoder->hal.context, key);
        return MOTION_STATE;
    }
    ++encoder->sample.reset_epoch;
    encoder->sample.left_count = 0U;
    encoder->sample.right_count = 0U;
    encoder->sample.left_last_edge_us = 0U;
    encoder->sample.right_last_edge_us = 0U;
    encoder->sample.left_edge_seen = false;
    encoder->sample.right_edge_seen = false;
    encoder->sample.edge_times_valid = true;
    encoder->sample.coalesced_edges_seen = false;
    ok = encoder->hal.gpio_input(encoder->hal.context, PIN_ENCODER_LEFT);
    if (!encoder->hal.gpio_input(encoder->hal.context, PIN_ENCODER_RIGHT))
    {
        ok = false;
    }
    encoder->ready = ok;
    encoder->hal.leave(encoder->hal.context, key);
    return ok ? MOTION_OK : MOTION_IO_ERROR;
}

motion_status_t drv_encoder_deinit(drv_encoder_t *encoder)
{
    motion_lock_key_t key;
    if ((encoder == NULL) || !encoder->bound)
    {
        return MOTION_STATE;
    }
    key = encoder->hal.enter(encoder->hal.context);
    encoder->ready = false;
    encoder->hal.leave(encoder->hal.context, key);
    return MOTION_OK;
}

static void count_edges(uint32_t rising, uint32_t falling, uint8_t pin,
    uint32_t observed_us, uint32_t *count, uint32_t *last_edge, bool *seen,
    motion_encoder_snapshot_t *sample)
{
    uint32_t mask = (uint32_t)1U << pin;
    uint32_t edges = ((rising & mask) != 0U) ? 1U : 0U;
    edges += ((falling & mask) != 0U) ? 1U : 0U;
    if (edges != 0U)
    {
        if (*seen && ((observed_us - *last_edge) >= 0x80000000UL))
        {
            sample->edge_times_valid = false;
        }
        *count += edges; /* Intentional U32 modulo cumulative counter. */
        *last_edge = observed_us;
        *seen = true;
        if (edges == 2U)
        {
            sample->coalesced_edges_seen = true;
        }
    }
}

void drv_encoder_on_edges(void *user, uint32_t rising_pins,
    uint32_t falling_pins, uint32_t observed_us)
{
    drv_encoder_t *encoder = user;
    motion_lock_key_t key;
    if ((encoder == NULL) || !encoder->bound)
    {
        return;
    }
    key = encoder->hal.enter(encoder->hal.context);
    if (encoder->ready)
    {
        count_edges(rising_pins, falling_pins, PIN_ENCODER_LEFT, observed_us,
            &encoder->sample.left_count, &encoder->sample.left_last_edge_us,
            &encoder->sample.left_edge_seen, &encoder->sample);
        count_edges(rising_pins, falling_pins, PIN_ENCODER_RIGHT, observed_us,
            &encoder->sample.right_count, &encoder->sample.right_last_edge_us,
            &encoder->sample.right_edge_seen, &encoder->sample);
    }
    encoder->hal.leave(encoder->hal.context, key);
}

motion_status_t drv_encoder_capture(drv_encoder_t *encoder,
    const drv_motor_t *motor, motion_encoder_snapshot_t *snapshot)
{
    motion_lock_key_t key;
    uint32_t now;
    if (snapshot == NULL)
    {
        return MOTION_INVALID;
    }
    (void)memset(snapshot, 0, sizeof(*snapshot));
    if ((encoder == NULL) || !encoder->bound)
    {
        return MOTION_STATE;
    }
    if ((motor != NULL) && (!motor->bound ||
        (motor->hal.context != encoder->hal.context) ||
        (motor->hal.enter != encoder->hal.enter) ||
        (motor->hal.leave != encoder->hal.leave)))
    {
        return MOTION_INVALID;
    }
    key = encoder->hal.enter(encoder->hal.context);
    if (!encoder->ready || !encoder->hal.clock_us(encoder->hal.context, &now))
    {
        encoder->hal.leave(encoder->hal.context, key);
        return MOTION_UNAVAILABLE;
    }
    *snapshot = encoder->sample;
    snapshot->acquired_us = now;
    snapshot->acquisition_valid = true;
    if ((snapshot->left_edge_seen &&
        ((now - snapshot->left_last_edge_us) >= 0x80000000UL)) ||
        (snapshot->right_edge_seen &&
        ((now - snapshot->right_last_edge_us) >= 0x80000000UL)))
    {
        /* Future/reordered or older-than-half-domain edge time: do not
         * advertise an unambiguous age. Raw captured counts remain available. */
        snapshot->edge_times_valid = false;
    }
    if (motor != NULL)
    {
        snapshot->direction_epoch = motor->state.direction_epoch;
        snapshot->left_command_sign = (int8_t)((motor->state.left_effort > 0) -
            (motor->state.left_effort < 0));
        snapshot->right_command_sign = (int8_t)((motor->state.right_effort > 0) -
            (motor->state.right_effort < 0));
        snapshot->command_basis_valid = motor->state.ready &&
            motor->state.armed && !motor->state.inhibited &&
            !motor->state.io_fault && !motor->state.sequence_exhausted;
    }
    encoder->hal.leave(encoder->hal.context, key);
    return MOTION_OK;
}

motion_status_t drv_encoder_reset(drv_encoder_t *encoder)
{
    motion_lock_key_t key;
    uint32_t epoch;
    if ((encoder == NULL) || !encoder->bound)
    {
        return MOTION_STATE;
    }
    key = encoder->hal.enter(encoder->hal.context);
    if (!encoder->ready || (encoder->sample.reset_epoch == UINT32_MAX))
    {
        encoder->hal.leave(encoder->hal.context, key);
        return MOTION_STATE;
    }
    epoch = encoder->sample.reset_epoch + 1U;
    (void)memset(&encoder->sample, 0, sizeof(encoder->sample));
    encoder->sample.reset_epoch = epoch;
    encoder->sample.edge_times_valid = true;
    encoder->hal.leave(encoder->hal.context, key);
    return MOTION_OK;
}
