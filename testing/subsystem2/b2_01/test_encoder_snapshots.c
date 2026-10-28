#include "fake_motion_hal.h"

#include <assert.h>
#include <stdio.h>

#define LEFT_EDGE ((uint32_t)1U << 27)
#define RIGHT_EDGE ((uint32_t)1U << 5)
#define SONAR_EDGE ((uint32_t)1U << 17)

int main(void)
{
    fake_motion_hal_t fake;
    drv_encoder_t encoder = {0};
    drv_motor_t motor = {0};
    motion_hal_t hal;
    motion_encoder_snapshot_t snapshot;
    uint32_t epoch;
    fake_motion_init(&fake);
    hal = fake_motion_hal(&fake);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    assert(drv_encoder_init(&encoder, &hal) == MOTION_OK);
    assert(hal.attach_encoders(hal.context, drv_encoder_on_edges, &encoder));
    fake.now_us = 100U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.acquisition_valid && snapshot.edge_times_valid);
    assert(!snapshot.left_edge_seen && !snapshot.right_edge_seen);
    assert(!snapshot.command_basis_valid && snapshot.left_count == 0U);
    epoch = snapshot.reset_epoch;
    assert(drv_encoder_init(&encoder, &hal) == MOTION_STATE);
    fake_dispatch(&fake, LEFT_EDGE | RIGHT_EDGE | SONAR_EDGE, 0U, 101U);
    fake_dispatch(&fake, 0U, RIGHT_EDGE, 102U);
    fake.now_us = 103U;
    fake_arm(&motor, 1U);
    assert(drv_motor_apply(&motor, 1U, 200, -300) == MOTION_OK);
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.right_count == 2U);
    assert(snapshot.left_last_edge_us == 101U && snapshot.right_last_edge_us == 102U);
    assert(snapshot.acquired_us == 103U && snapshot.reset_epoch == epoch);
    assert(snapshot.command_basis_valid && snapshot.left_command_sign == 1);
    assert(snapshot.right_command_sign == -1 && snapshot.direction_epoch != 0U);
    assert(fake.unrelated_sonar_events == 1U);
    fake.now_us = 100000U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.acquired_us == 100000U);
    assert(snapshot.left_last_edge_us == 101U); /* Fresh sample, old edge. */
    assert(drv_motor_stop(&motor) == MOTION_OK);
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(!snapshot.command_basis_valid && snapshot.left_command_sign == 0);
    fake.clock_valid = false;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_UNAVAILABLE);
    assert(!snapshot.acquisition_valid && snapshot.left_count == 0U);
    fake.clock_valid = true;
    assert(drv_encoder_reset(&encoder) == MOTION_OK);
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.reset_epoch == epoch + 1U && snapshot.left_count == 0U);
    assert(!snapshot.left_edge_seen);

    /* Boundary fixtures, not a production counter mutation API. */
    encoder.sample.left_count = UINT32_MAX;
    encoder.sample.right_count = UINT32_MAX - 1U;
    fake_dispatch(&fake, LEFT_EDGE | RIGHT_EDGE, RIGHT_EDGE, UINT32_MAX - 2U);
    fake_dispatch(&fake, 0U, LEFT_EDGE, 3U);
    fake.now_us = 4U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.right_count == 0U);
    assert(snapshot.left_last_edge_us == 3U && snapshot.edge_times_valid);
    assert(snapshot.coalesced_edges_seen);
    fake_dispatch(&fake, LEFT_EDGE, 0U, 2U);
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(!snapshot.edge_times_valid && snapshot.left_count == 2U);

    assert(drv_encoder_reset(&encoder) == MOTION_OK);
    /* Edge arrives while snapshot holds lock: it belongs to NEXT capture. */
    fake.inject_at = fake.operations + 1U; /* clock operation */
    fake.rising = LEFT_EDGE | RIGHT_EDGE;
    fake.falling = 0U;
    fake.edge_us = 5U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 0U && snapshot.right_count == 0U);
    fake.now_us = 6U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.right_count == 1U);
    assert(snapshot.edge_times_valid);
    epoch = snapshot.reset_epoch;
    fake_dispatch(&fake, LEFT_EDGE, 0U, 6U);
    assert(drv_encoder_reset(&encoder) == MOTION_OK);
    fake_dispatch(&fake, RIGHT_EDGE, 0U, 7U);
    fake.now_us = 8U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.reset_epoch == epoch + 1U);
    assert(snapshot.left_count == 0U && snapshot.right_count == 1U);
    /* Waiting IRQ linearizes AFTER reset and belongs to its new epoch. */
    fake.pending_edges = true;
    fake.rising = LEFT_EDGE;
    fake.falling = 0U;
    fake.edge_us = 9U;
    assert(drv_encoder_reset(&encoder) == MOTION_OK);
    fake.now_us = 10U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.reset_epoch == epoch + 2U && snapshot.left_count == 1U);
    assert(snapshot.right_count == 0U && snapshot.edge_times_valid);
    /* Future time cannot silently become a plausible unsigned edge age. */
    fake_dispatch(&fake, LEFT_EDGE, 0U, 20U);
    fake.now_us = 19U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.acquisition_valid && !snapshot.edge_times_valid);
    encoder.sample.reset_epoch = UINT32_MAX;
    assert(drv_encoder_reset(&encoder) == MOTION_STATE);
    assert(drv_encoder_deinit(&encoder) == MOTION_OK);
    fake_dispatch(&fake, LEFT_EDGE, 0U, 8U);
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_UNAVAILABLE);
    assert(!snapshot.acquisition_valid);
    assert(hal.detach_encoders(hal.context));
    fake_assert_safe(&fake);
    (void)puts("B2-01 encoder masks/snapshot/epoch/wrap/direction checks passed");
    return 0;
}
