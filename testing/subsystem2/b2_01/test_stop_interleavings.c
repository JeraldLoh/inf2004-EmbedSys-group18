#include "fake_motion_hal.h"

#include <assert.h>
#include <stdio.h>

static void test_stop_at_each_commit_boundary(void)
{
    unsigned boundary;
    /* 4 compare + 4 mux + 2 slice-enable operations. */
    for (boundary = 0U; boundary <= 10U; ++boundary)
    {
        fake_motion_hal_t fake;
        drv_motor_t motor = {0};
        motion_motor_state_t state;
        motion_hal_t hal;
        size_t after_stop;
        fake_motion_init(&fake);
        hal = fake_motion_hal(&fake);
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
        fake_arm(&motor, 10U);
        fake.stop_motor = &motor;
        if (boundary == 0U)
        {
            fake.stop_before_enter = true;
            assert(drv_motor_apply(&motor, 10U, 300, 400) == MOTION_INHIBITED);
        }
        else
        {
            fake.inject_at = fake.operations + boundary;
            /* Commit linearizes before the waiting STOP; return OK is NOT a
             * promise that STOP has not superseded it by the time caller runs. */
            assert(drv_motor_apply(&motor, 10U, 300, 400) == MOTION_OK);
        }
        fake_assert_safe(&fake);
        assert(drv_motor_read(&motor, &state) == MOTION_OK);
        assert(state.inhibited && !state.armed && state.safe_command_applied);
        after_stop = fake.trace_count;
        /* Simulate an arbitrarily full ordinary queue by replaying its entries.
         * STOP never consumes such a queue and none of the writes can resume. */
        assert(drv_motor_apply(&motor, 10U, 1000, 1000) == MOTION_INHIBITED);
        assert(drv_motor_apply(&motor, 9U, 100, -100) == MOTION_INHIBITED);
        assert(fake.trace_count == after_stop);
        assert(drv_motor_arm(&motor, state.stop_sequence, 11U) == MOTION_INHIBITED);
        assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
        fake_assert_safe(&fake);
        assert(drv_motor_arm(&motor, state.stop_sequence, 10U) == MOTION_STALE);
        assert(drv_motor_arm(&motor, state.stop_sequence, 11U) == MOTION_OK);
        fake_assert_safe(&fake);
    }
}

static void test_clear_arm_and_shutdown_order(void)
{
    fake_motion_hal_t fake;
    drv_motor_t motor = {0};
    motion_motor_state_t state;
    motion_hal_t hal;
    size_t start;
    size_t i;
    fake_motion_init(&fake);
    hal = fake_motion_hal(&fake);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    fake.stop_motor = &motor;
    fake.stop_before_enter = true;
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_STALE);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    fake.stop_before_enter = true;
    assert(drv_motor_arm(&motor, state.stop_sequence, 1U) == MOTION_INHIBITED);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    /* STOP linearizes immediately AFTER a clear that reported success. */
    fake.pending_stop = true;
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(state.inhibited);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    /* Likewise, an arm acknowledgment cannot override a later STOP. */
    fake.pending_stop = true;
    assert(drv_motor_arm(&motor, state.stop_sequence, 1U) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(state.inhibited && !state.armed);
    fake_arm(&motor, 2U);
    assert(drv_motor_apply(&motor, 2U, 500, 500) == MOTION_OK);
    /* Simulate already HIGH output frozen if a slice is simply disabled. */
    fake.frozen_high[8] = true;
    fake.frozen_high[10] = true;
    start = fake.trace_count;
    assert(drv_motor_stop(&motor) == MOTION_OK);
    assert(fake.trace_count == start + 14U);
    for (i = 0U; i < 4U; ++i)
    {
        assert(fake.trace[start + i].operation == 'c');
        assert(fake.trace[start + 4U + i].operation == 'w');
        assert(fake.trace[start + 8U + i].operation == 'o');
    }
    assert(fake.trace[start + 12U].operation == 'e');
    assert(fake.trace[start + 13U].operation == 'e');
    fake_assert_safe(&fake);
    assert(drv_motor_stop(&motor) == MOTION_OK);
    fake_assert_safe(&fake);
}

int main(void)
{
    test_stop_at_each_commit_boundary();
    test_clear_arm_and_shutdown_order();
    (void)puts("B2-01 STOP ordering/interleavings passed (modeled lock only)");
    return 0;
}
