#include "fake_motion_hal.h"
#include "subsystem2.h"

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
    fake_motion_hal_t fake;
    motion_hal_t hal;
    motion_motor_state_t motor;
    motion_encoder_snapshot_t snapshot;
    fake_motion_init(&fake);
    hal = fake_motion_hal(&fake);
    assert(subsystem2_init() == E_NOSPT);
    assert(subsystem2_start() == E_NOSPT);
    assert(subsystem2_bind_hal(NULL) == E_PAR);
    assert(subsystem2_bind_hal(&hal) == E_OK);
    assert(subsystem2_bind_hal(&hal) == E_OBJ);
    if ((argc == 2) && (argv[1][0] >= '0') && (argv[1][0] <= '9'))
    {
        fake.fail_at = (unsigned)strtoul(argv[1], NULL, 10);
        assert(fake.fail_at >= 1U && fake.fail_at <= 20U);
        assert(subsystem2_init() == E_SYS);
        assert(subsystem2_start() == E_OBJ);
        assert(!fake.subscribed);
        fake_assert_safe(&fake);
        assert(subsystem2_read_motor(&motor) == MOTION_OK);
        assert(motor.inhibited && !motor.armed);
        fake.fail_at = 0U;
    }
    assert(subsystem2_init() == E_OK);
    assert(fake.subscribed);
    if ((argc == 2) && ((strcmp(argv[1], "start_clock") == 0) ||
        (strcmp(argv[1], "start_output") == 0)))
    {
        if (strcmp(argv[1], "start_clock") == 0)
        {
            fake.clock_valid = false;
        }
        else
        {
            fake.fail_at = fake.operations + 1U;
            assert(subsystem2_urgent_stop() == MOTION_IO_ERROR);
        }
        assert(subsystem2_start() == E_SYS);
        assert(!fake.subscribed);
        fake.clock_valid = true;
        fake.fail_at = 0U;
        fake_assert_safe(&fake);
        assert(subsystem2_init() == E_OK);
    }
    assert(subsystem2_start() == E_OK);
    fake_assert_safe(&fake);
    assert(subsystem2_read_motor(&motor) == MOTION_OK);
    assert(motor.inhibited && !motor.armed && motor.ready);
    fake_dispatch(&fake, (uint32_t)1U << 27, (uint32_t)1U << 5, 123U);
    fake.now_us = 124U;
    assert(subsystem2_read_encoders(&snapshot) == MOTION_OK);
    assert(snapshot.left_count == 1U && snapshot.right_count == 1U);
    assert(subsystem2_init() == E_OBJ);
    assert(subsystem2_start() == E_OBJ);
    assert(subsystem2_urgent_stop() == MOTION_OK);
    fake_assert_safe(&fake);
    if ((argc == 2) && (strcmp(argv[1], "detach") == 0))
    {
        fake.fail_at = fake.operations + 15U; /* 14 motor-off calls + detach */
        assert(subsystem2_shutdown() == E_SYS);
        assert(fake.subscribed);
        assert(subsystem2_init() == E_OBJ);
        fake.fail_at = 0U;
    }
    assert(subsystem2_shutdown() == E_OK);
    assert(!fake.subscribed);
    assert(subsystem2_read_encoders(&snapshot) == MOTION_UNAVAILABLE);
    assert(!snapshot.acquisition_valid);
    fake_assert_safe(&fake);
    assert(subsystem2_init() == E_OK);
    assert(subsystem2_start() == E_OK);
    fake_assert_safe(&fake);
    assert(subsystem2_shutdown() == E_OK);
    (void)puts("B2-01 course lifecycle/rollback/registration checks passed");
    return 0;
}
