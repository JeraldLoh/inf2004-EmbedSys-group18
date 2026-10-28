#include "fake_motion_hal.h"
#include "motion_hal_rp2040.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

extern fake_motion_hal_t *bsp_fake;

int main(void)
{
    fake_motion_hal_t fake;
    motion_hal_t base;
    motion_hal_t target = {0};
    motion_hal_t unchanged;
    motion_rp2040_port_t port;
    drv_motor_t motor = {0};
    drv_encoder_t encoder = {0};
    motion_encoder_snapshot_t snapshot;
    fake_motion_init(&fake);
    bsp_fake = &fake;
    base = fake_motion_hal(&fake);
    port = (motion_rp2040_port_t){&fake, base.enter, base.leave, base.clock_us,
        base.attach_encoders, base.detach_encoders};
    unchanged = target;
    assert(!motion_hal_rp2040_create(NULL, &port));
    assert(!motion_hal_rp2040_create(&target, NULL));
    assert(memcmp(&target, &unchanged, sizeof(target)) == 0);
    assert(fake.operations == 0U);
    assert(motion_hal_rp2040_create(&target, &port));
    assert(fake.operations == 0U); /* Binding never touches hardware. */
    assert(drv_motor_init(&motor, &target, 6249U) == MOTION_OK);
    assert(drv_encoder_init(&encoder, &target) == MOTION_OK);
    assert(target.attach_encoders(target.context, drv_encoder_on_edges, &encoder));
    fake_dispatch(&fake, (uint32_t)1U << 27, 0U, 1U);
    fake.now_us = 2U;
    assert(drv_encoder_capture(&encoder, &motor, &snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.acquired_us == 2U);
    fake_arm(&motor, 1U);
    assert(drv_motor_apply(&motor, 1U, -100, 200) == MOTION_OK);
    assert(fake.compare[9] == 625U && fake.compare[10] == 1250U);
    fake.fail_at = fake.operations + 1U;
    assert(drv_motor_apply(&motor, 1U, -200, 300) == MOTION_IO_ERROR);
    fake_assert_safe(&fake);
    assert(target.detach_encoders(target.context));
    (void)puts("B2-01 RP2040 adapter checked against BSP call mocks only");
    return 0;
}
