#include "platform_stubs.h"
#include "subsystem2_internal.h"
#include "b2_platform_rp2040.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned sonar_events;
static void sonar(void *user, uint32_t rising, uint32_t falling, uint32_t when)
{
    motion_motor_state_t state;
    (void)user; (void)when;
    assert(rising == ((uint32_t)1U << 17) && falling == 0U);
    assert(subsystem2_read_motor(&state) == MOTION_OK);
    assert(state.inhibited); /* Urgent STOP already ran before normal client. */
    ++sonar_events;
}

int main(int argc, char **argv)
{
    fake_motion_hal_t fake;
    motion_hal_t hal;
    b2_trial_io_t io;
    b2_trial_t trial;
    vehicle_controller_t controller;
    b2_platform_events_t events = {0};
    b2_trial_result_t result;
    motion_encoder_snapshot_t sample;
    motion_motor_state_t motor;
    fake_motion_init(&fake); bsp_fake = &fake;
    mock_core = 1U;
    assert(!b2_platform_init(&hal, &io));
    assert(fake.operations == 0U);
    mock_core = 0U;
    if (argc == 2)
    {
        if (strcmp(argv[1], "timer_owner") == 0) { mock_timer_enabled = 1U; }
        else if (strcmp(argv[1], "gpio_owner") == 0) { mock_gpio_owned = true; }
        else if (strcmp(argv[1], "irq_failure") == 0) { mock_fail_irq = 0U; }
        else if (strcmp(argv[1], "output_failure") == 0) { fake.fail_at = 1U; }
        else if (strcmp(argv[1], "clock_failure") == 0) { mock_clock_step = false; }
        else { assert(false); }
        assert(!b2_platform_init(&hal, &io));
        fake_assert_safe(&fake);
        (void)puts("Platform startup refusal leaves motors disabled, no trial");
        return 0;
    }
    assert(b2_platform_init(&hal, &io));
    mock_clock_step = false;
    assert(subsystem2_bind_hal(&hal) == E_OK);
    assert(subsystem2_init() == E_OK && subsystem2_start() == E_OK);
    fake_assert_safe(&fake);
    vehicle_controller_init(&controller);
    assert(b2_trial_init(&trial, subsystem2_bench_motor(), subsystem2_bench_encoder(),
        &controller, &io));
    mock_alarm_setup_delay = 10U;
    assert(b2_trial_begin(&trial, 150, 0, 1U) == MOTION_INHIBITED);
    fake_assert_safe(&fake);
    assert(mock_timer_enabled == 0U);
    mock_alarm_setup_delay = 0U;
    fake.now_us = 100U;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_OK);
    assert(mock_timer_enabled == 1U && mock_timer_alarm == 250100U);
    fake.now_us = mock_timer_alarm;
    mock_fire_irq(0U); /* No task service: actual timer handler must stop PWM. */
    fake_assert_safe(&fake);
    b2_platform_take_events(&events);
    assert(events.cutoff_fired && events.reasons == STOP_REASON_NONE);
    assert(mock_timer_enabled == 0U);
    b2_trial_service(&trial, events.reasons, events.cutoff_fired);
    assert(b2_trial_take_result(&trial, &result));
    assert(result.state == B2_TRIAL_TIME_ENDED);
    assert(gpio_dispatcher_attach(b2_platform_dispatcher(), (uint32_t)1U << 17,
        sonar, NULL, false));
    assert(b2_trial_begin(&trial, 150, 150, 250000U) == MOTION_OK);
    fake.inputs[21] = false;
    mock_gpio_edges(((uint32_t)1U << 27) | ((uint32_t)1U << 17),
        ((uint32_t)1U << 5) | ((uint32_t)1U << 21));
    mock_fire_irq(13U);
    fake_assert_safe(&fake);
    assert(sonar_events == 1U);
    assert(subsystem2_read_encoders(&sample) == MOTION_OK);
    assert(sample.left_count == 1U && sample.right_count == 1U);
    b2_platform_take_events(&events);
    assert(events.reasons & STOP_REASON_PHYSICAL);
    b2_trial_service(&trial, events.reasons, events.cutoff_fired);
    assert(b2_trial_take_result(&trial, &result) && result.state == B2_TRIAL_STOPPED);
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    fake.inputs[21] = true;
    mock_stop_during_arm = true;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    fake_assert_safe(&fake);
    assert(subsystem2_read_motor(&motor) == MOTION_OK && motor.inhibited);
    mock_core = 1U;
    assert(!gpio_dispatcher_detach(b2_platform_dispatcher(), (uint32_t)1U << 17));
    mock_core = 0U;
    assert(gpio_dispatcher_detach(b2_platform_dispatcher(), (uint32_t)1U << 17));
    assert(subsystem2_shutdown() == E_OK);
    assert(mock_lock_entries > 20U);
    (void)puts("Actual RP2040 binding with MMIO/IRQ/kernel-lock mocks passed; target latency unmeasured");
    return 0;
}
