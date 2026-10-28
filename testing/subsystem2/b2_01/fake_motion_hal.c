#include "fake_motion_hal.h"

#include <assert.h>
#include <string.h>

#include "../../../app_program/project_config/pinmap.h"

static bool step(fake_motion_hal_t *fake, char operation, uint8_t pin,
    uint32_t value)
{
    bool success;
    assert((operation == 'a') || (operation == 'd') || fake->locked);
    assert(pin < 32U);
    assert(fake->trace_count < FAKE_TRACE_CAPACITY);
    ++fake->operations;
    success = (fake->operations != fake->fail_at) &&
        ((fake->fail_from == 0U) || (fake->operations < fake->fail_from));
    fake->trace[fake->trace_count++] = (fake_trace_t)
        {operation, pin, value, success};
    if (fake->operations == fake->inject_at)
    {
        if (fake->stop_motor != NULL)
        {
            fake->pending_stop = true;
        }
        else
        {
            fake->pending_edges = true;
        }
    }
    return success;
}

static motion_lock_key_t enter(void *context)
{
    fake_motion_hal_t *fake = context;
    if (fake->stop_before_enter)
    {
        fake->stop_before_enter = false;
        assert(drv_motor_stop(fake->stop_motor) == MOTION_OK);
    }
    assert(!fake->locked);
    fake->locked = true;
    return (motion_lock_key_t)0xB201U;
}

static void leave(void *context, motion_lock_key_t key)
{
    fake_motion_hal_t *fake = context;
    assert(fake->locked && (key == (motion_lock_key_t)0xB201U));
    fake->locked = false;
    /* Models either masked same-core IRQ delivery or another core acquiring
     * after this owner. Never recursively execute STOP while the lock is held. */
    if (fake->pending_stop)
    {
        fake->pending_stop = false;
        assert(drv_motor_stop(fake->stop_motor) == MOTION_OK);
    }
    if (fake->pending_edges)
    {
        fake->pending_edges = false;
        fake_dispatch(fake, fake->rising, fake->falling, fake->edge_us);
    }
}

bool fake_pin_high(const fake_motion_hal_t *fake, uint8_t pin)
{
    if (!fake->output[pin])
    {
        return false;
    }
    if (!fake->pin_pwm[pin])
    {
        return fake->latch[pin];
    }
    return fake->enabled[pin / 2U] ? (fake->compare[pin] != 0U)
        : fake->frozen_high[pin];
}

static bool gpio_write(void *context, uint8_t pin, bool high)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'w', pin, high ? 1U : 0U)) { return false; }
    fake->latch[pin] = high;
    return true;
}

static bool gpio_output(void *context, uint8_t pin)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'o', pin, 0U)) { return false; }
    fake->output[pin] = true;
    fake->pin_pwm[pin] = false;
    return true;
}

static bool gpio_input(void *context, uint8_t pin)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'i', pin, 0U)) { return false; }
    fake->output[pin] = false;
    fake->pin_pwm[pin] = false;
    return true;
}

static bool pwm_function(void *context, uint8_t pin)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'p', pin, 0U)) { return false; }
    fake->output[pin] = true;
    fake->pin_pwm[pin] = true;
    return true;
}

static bool pwm_compare(void *context, uint8_t pin, uint16_t value)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'c', pin, value)) { return false; }
    fake->compare[pin] = value;
    return true;
}

static bool pwm_wrap(void *context, uint8_t pin, uint16_t value)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 't', pin, value)) { return false; }
    fake->wrap[pin / 2U] = value;
    return true;
}

static bool pwm_enable(void *context, uint8_t pin, bool enabled)
{
    fake_motion_hal_t *fake = context;
    bool high_a = fake_pin_high(fake, pin);
    bool high_b = fake_pin_high(fake, (uint8_t)(pin + 1U));
    if (!step(fake, 'e', pin, enabled ? 1U : 0U)) { return false; }
    fake->enabled[pin / 2U] = enabled;
    if (!enabled)
    {
        fake->frozen_high[pin] = high_a;
        fake->frozen_high[pin + 1U] = high_b;
    }
    return true;
}

static bool clock_us(void *context, uint32_t *now)
{
    fake_motion_hal_t *fake = context;
    if (!step(fake, 'k', 0U, fake->now_us) || !fake->clock_valid)
    {
        return false;
    }
    *now = fake->now_us;
    return true;
}

static bool attach(void *context, motion_edge_handler_t handler, void *user)
{
    fake_motion_hal_t *fake = context;
    assert(!fake->locked && !fake->subscribed);
    if (!step(fake, 'a', 0U, 0U)) { return false; }
    fake->handler = handler;
    fake->handler_user = user;
    fake->subscribed = true;
    return true;
}

static bool detach(void *context)
{
    fake_motion_hal_t *fake = context;
    assert(!fake->locked);
    ++fake->detach_calls;
    if (!step(fake, 'd', 0U, 0U)) { return false; }
    fake->subscribed = false;
    fake->handler = NULL;
    fake->handler_user = NULL;
    return true;
}

void fake_motion_init(fake_motion_hal_t *fake)
{
    (void)memset(fake, 0, sizeof(*fake));
    fake->clock_valid = true;
    fake->inputs[20] = true;
    fake->inputs[21] = true;
}

motion_hal_t fake_motion_hal(fake_motion_hal_t *fake)
{
    motion_hal_t hal = {fake, enter, leave, gpio_write, gpio_output, gpio_input,
        pwm_function, pwm_compare, pwm_wrap, pwm_enable, clock_us, attach, detach};
    return hal;
}

void fake_assert_safe(const fake_motion_hal_t *fake)
{
    uint8_t pin;
    assert(!fake->locked);
    for (pin = PIN_MOTOR_LEFT_A; pin <= PIN_MOTOR_RIGHT_B; ++pin)
    {
        assert(!fake_pin_high(fake, pin));
        assert(!fake->pin_pwm[pin]);
        assert(fake->output[pin]);
        assert(!fake->enabled[pin / 2U]);
    }
}

void fake_arm(drv_motor_t *motor, uint32_t generation)
{
    motion_motor_state_t state;
    assert(drv_motor_read(motor, &state) == MOTION_OK);
    assert(drv_motor_clear_inhibit(motor, state.stop_sequence) == MOTION_OK);
    assert(drv_motor_arm(motor, state.stop_sequence, generation) == MOTION_OK);
}

void fake_dispatch(fake_motion_hal_t *fake, uint32_t rising,
    uint32_t falling, uint32_t when_us)
{
    if (((rising | falling) & ((uint32_t)1U << PIN_SONAR_ECHO)) != 0U)
    {
        ++fake->unrelated_sonar_events;
    }
    if (fake->subscribed)
    {
        fake->handler(fake->handler_user, rising, falling, when_us);
    }
}
