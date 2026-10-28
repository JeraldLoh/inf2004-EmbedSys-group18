#include "fake_motion_hal.h"

#include <assert.h>
#include <stdio.h>

static void test_boot_tokens_and_mapping(void)
{
    fake_motion_hal_t fake;
    drv_motor_t motor = {0};
    motion_hal_t hal;
    motion_motor_state_t state;
    uint32_t old_sequence;
    fake_motion_init(&fake);
    hal = fake_motion_hal(&fake);
    assert(!motion_hal_is_valid(NULL));
    assert(drv_motor_init(&motor, NULL, 6249U) == MOTION_INVALID);
    assert(drv_motor_init(&motor, &hal, UINT16_MAX) == MOTION_INVALID);
    assert(fake.operations == 0U);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    assert(fake.operations == 16U);
    fake_assert_safe(&fake);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    old_sequence = state.stop_sequence;
    assert(state.inhibited && !state.armed && state.safe_command_applied);
    assert(drv_motor_apply(&motor, 1U, 100, 100) == MOTION_INHIBITED);
    assert(drv_motor_arm(&motor, old_sequence, 1U) == MOTION_INHIBITED);
    assert(drv_motor_clear_inhibit(&motor, old_sequence - 1U) == MOTION_STALE);
    assert(drv_motor_clear_inhibit(&motor, old_sequence) == MOTION_OK);
    assert(drv_motor_apply(&motor, 1U, 100, 100) == MOTION_STATE);
    assert(drv_motor_arm(&motor, old_sequence, 0U) == MOTION_STALE);
    assert(drv_motor_arm(&motor, old_sequence, 1U) == MOTION_OK);
    fake_assert_safe(&fake);
    assert(drv_motor_apply(&motor, 2U, 100, 100) == MOTION_STALE);
    assert(drv_motor_apply(&motor, 1U, 100, -200) == MOTION_OK);
    assert(fake.compare[8] == 625U && fake.compare[9] == 0U);
    assert(fake.compare[10] == 0U && fake.compare[11] == 1250U);
    assert(fake_pin_high(&fake, 8U) && fake_pin_high(&fake, 11U));
    assert(drv_motor_apply(&motor, 1U, -100, -200) == MOTION_REVERSAL_REQUIRED);
    fake_assert_safe(&fake);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(state.inhibited && !state.armed);
    assert(drv_motor_clear_inhibit(&motor, old_sequence) == MOTION_STALE);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    assert(drv_motor_arm(&motor, state.stop_sequence, 1U) == MOTION_STALE);
    assert(drv_motor_arm(&motor, state.stop_sequence, 2U) == MOTION_OK);
    assert(drv_motor_apply(&motor, 2U, -1000, 0) == MOTION_OK);
    assert(fake.compare[9] == 6250U && fake.compare[8] == 0U);
    assert(fake.compare[10] == 0U && fake.compare[11] == 0U);
    assert(drv_motor_apply(&motor, 2U, 0, 0) == MOTION_OK);
    fake_assert_safe(&fake);
    assert(drv_motor_apply(&motor, 2U, 1001, 0) == MOTION_INVALID);
    fake_assert_safe(&fake);
    fake_arm(&motor, 3U);
    assert(drv_motor_apply(&motor, 3U, 0, 1000) == MOTION_OK);
    assert(fake.compare[10] == 6250U && fake.compare[11] == 0U);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_STATE);
    fake_assert_safe(&fake);
    assert(drv_motor_deinit(&motor) == MOTION_OK);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    assert(drv_motor_arm(&motor, state.stop_sequence, 3U) == MOTION_STALE);
    assert(drv_motor_arm(&motor, state.stop_sequence, 4U) == MOTION_OK);
    fake_assert_safe(&fake);
}

static void test_failures(void)
{
    unsigned failure;
    for (failure = 1U; failure <= 16U; ++failure)
    {
        fake_motion_hal_t fake;
        drv_motor_t motor = {0};
        motion_motor_state_t state;
        motion_hal_t hal;
        fake_motion_init(&fake);
        hal = fake_motion_hal(&fake);
        fake.fail_at = failure;
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_IO_ERROR);
        fake_assert_safe(&fake);
        assert(drv_motor_read(&motor, &state) == MOTION_OK);
        assert(!state.ready && state.io_fault && state.inhibited);
        assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_STATE);
        fake.fail_at = 0U;
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
        fake_assert_safe(&fake);
    }
    for (failure = 1U; failure <= 10U; ++failure)
    {
        fake_motion_hal_t fake;
        drv_motor_t motor = {0};
        motion_motor_state_t state;
        motion_hal_t hal;
        fake_motion_init(&fake);
        hal = fake_motion_hal(&fake);
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
        fake_arm(&motor, 1U);
        fake.fail_at = fake.operations + failure;
        assert(drv_motor_apply(&motor, 1U, 200, 300) == MOTION_IO_ERROR);
        fake_assert_safe(&fake);
        assert(drv_motor_read(&motor, &state) == MOTION_OK);
        assert(state.io_fault && state.inhibited && !state.ready);
    }
    for (failure = 1U; failure <= 14U; ++failure)
    {
        fake_motion_hal_t fake;
        drv_motor_t motor = {0};
        motion_motor_state_t state;
        motion_hal_t hal;
        unsigned before;
        fake_motion_init(&fake);
        hal = fake_motion_hal(&fake);
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
        fake_arm(&motor, 1U);
        assert(drv_motor_apply(&motor, 1U, 200, 300) == MOTION_OK);
        before = fake.operations;
        fake.fail_at = before + failure;
        assert(drv_motor_stop(&motor) == MOTION_IO_ERROR);
        assert(fake.operations == before + 14U); /* Never short-circuit cleanup. */
        assert(drv_motor_read(&motor, &state) == MOTION_OK);
        assert(state.inhibited && state.io_fault && !state.safe_command_applied);
        assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_STATE);
        fake.fail_at = 0U;
        assert(drv_motor_stop(&motor) == MOTION_OK);
        fake_assert_safe(&fake);
    }
    /* Persistent electrical/HAL failure must never be called confirmed-safe. */
    {
        fake_motion_hal_t fake;
        drv_motor_t motor = {0};
        motion_motor_state_t state;
        motion_hal_t hal;
        unsigned before;
        fake_motion_init(&fake);
        hal = fake_motion_hal(&fake);
        assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
        fake_arm(&motor, 1U);
        assert(drv_motor_apply(&motor, 1U, 200, 200) == MOTION_OK);
        before = fake.operations;
        fake.fail_from = before + 1U;
        assert(drv_motor_stop(&motor) == MOTION_IO_ERROR);
        assert(fake.operations == before + 14U);
        assert(drv_motor_read(&motor, &state) == MOTION_OK);
        assert(state.inhibited && state.io_fault && !state.safe_command_applied);
        assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_STATE);
        fake.fail_from = 0U;
        assert(drv_motor_stop(&motor) == MOTION_OK);
        fake_assert_safe(&fake);
    }
}

static void test_invalid_reinitialization_and_direction_epoch(void)
{
    fake_motion_hal_t fake;
    fake_motion_hal_t other_fake;
    motion_hal_t hal;
    motion_hal_t foreign_hal;
    drv_motor_t motor = {0};
    motion_motor_state_t state;
    fake_motion_init(&fake);
    fake_motion_init(&other_fake);
    hal = fake_motion_hal(&fake);
    foreign_hal = fake_motion_hal(&other_fake);
    assert(drv_motor_stop(&motor) == MOTION_STATE);
    assert(drv_motor_apply(&motor, 1U, 100, 100) == MOTION_STATE);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    fake_arm(&motor, 1U);
    assert(drv_motor_apply(&motor, 1U, 100, 100) == MOTION_OK);
    assert(drv_motor_init(&motor, &hal, 0U) == MOTION_INVALID);
    fake_assert_safe(&fake);
    fake_arm(&motor, 2U);
    assert(drv_motor_apply(&motor, 2U, 100, 100) == MOTION_OK);
    assert(drv_motor_init(&motor, &foreign_hal, 6249U) == MOTION_STATE);
    assert(other_fake.operations == 0U);
    fake_assert_safe(&fake);
    fake_arm(&motor, 3U);
    assert(drv_motor_apply(&motor, 3U, 100, 100) == MOTION_OK);
    motor.state.direction_epoch = UINT32_MAX;
    assert(drv_motor_apply(&motor, 3U, 0, 0) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(state.sequence_exhausted && state.inhibited && !state.armed);
    assert(!state.ready);
    fake_assert_safe(&fake);
    assert(drv_motor_apply(&motor, 3U, 100, 100) == MOTION_INHIBITED);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_STATE);
}

static void test_generation_wrap_and_exhaustion(void)
{
    fake_motion_hal_t fake;
    drv_motor_t motor = {0};
    motion_motor_state_t state;
    motion_hal_t hal;
    fake_motion_init(&fake);
    hal = fake_motion_hal(&fake);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    fake_arm(&motor, UINT32_MAX);
    assert(drv_motor_stop(&motor) == MOTION_OK);
    fake_arm(&motor, 1U);
    assert(drv_motor_stop(&motor) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(drv_motor_clear_inhibit(&motor, state.stop_sequence) == MOTION_OK);
    assert(drv_motor_arm(&motor, state.stop_sequence, UINT32_MAX) == MOTION_STALE);
    /* Fixture sets inaccessible-in-normal-use boundary under no active task. */
    motor.state.stop_sequence = UINT32_MAX;
    assert(drv_motor_stop(&motor) == MOTION_OK);
    assert(drv_motor_read(&motor, &state) == MOTION_OK);
    assert(state.sequence_exhausted && state.inhibited);
    assert(drv_motor_clear_inhibit(&motor, UINT32_MAX) == MOTION_STATE);
    assert(drv_motor_deinit(&motor) == MOTION_OK);
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_STATE);
    fake_assert_safe(&fake);
}

int main(void)
{
    test_boot_tokens_and_mapping();
    test_failures();
    test_invalid_reinitialization_and_direction_epoch();
    test_generation_wrap_and_exhaustion();
    (void)puts("B2-01 lifecycle/mapping/failure/token checks passed");
    return 0;
}
