#ifndef GROUP18_DRV_ENCODER_H
#define GROUP18_DRV_ENCODER_H

#include "drv_motor.h"

typedef struct
{
    uint32_t left_count;
    uint32_t right_count;
    uint32_t acquired_us;
    uint32_t reset_epoch;
    uint32_t left_last_edge_us;
    uint32_t right_last_edge_us;
    uint32_t direction_epoch;
    int8_t left_command_sign;
    int8_t right_command_sign;
    bool left_edge_seen;
    bool right_edge_seen;
    bool edge_times_valid;
    bool acquisition_valid;
    /* Command metadata only, NOT measured direction or signed odometry. */
    bool command_basis_valid;
    bool coalesced_edges_seen;
} motion_encoder_snapshot_t;

/* Zero-initialize once; no copying/live rebinding or external mutation. */
typedef struct
{
    motion_hal_t hal;
    motion_encoder_snapshot_t sample;
    bool bound;
    bool ready;
} drv_encoder_t;

motion_status_t drv_encoder_init(drv_encoder_t *encoder,
    const motion_hal_t *hal);
motion_status_t drv_encoder_deinit(drv_encoder_t *encoder);
/* Dispatcher supplies distinct latched rising/falling masks and the same
 * U32 microsecond domain as clock_us. It owns acknowledgement and exactly-once
 * delivery. Both bits on one pin count as TWO observed edges, but flag possible
 * coalescing; hardware can lose additional edges before dispatch. Never ACK
 * sonar/other pins here. This callback does not perform motor writes. */
void drv_encoder_on_edges(void *user, uint32_t rising_pins,
    uint32_t falling_pins, uint32_t observed_us);
/* Timestamp and BOTH wheels copied under the common HAL lock. Failed captures
 * explicitly zero the returned object and clear acquisition_valid. */
motion_status_t drv_encoder_capture(drv_encoder_t *encoder,
    const drv_motor_t *motor, motion_encoder_snapshot_t *snapshot);
/* Explicit maintenance reset, not per-goal. Epoch cannot silently wrap. */
motion_status_t drv_encoder_reset(drv_encoder_t *encoder);

#endif
