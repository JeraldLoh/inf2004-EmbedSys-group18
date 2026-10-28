#include "fake_motion_hal.h"
#include <bsp/libbsp.h>

fake_motion_hal_t *bsp_fake;

UINT gpio_get_val(UINT pin) { return bsp_fake->inputs[pin] ? 1U : 0U; }

ER gpio_set_pin(UINT pin, UINT mode)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    bool ok = (mode == GPIO_MODE_OUT)
        ? hal.gpio_output(hal.context, (uint8_t)pin)
        : hal.gpio_input(hal.context, (uint8_t)pin);
    return ok ? E_OK : E_SYS;
}

ER gpio_set_val(UINT pin, UINT value)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    return hal.gpio_write(hal.context, (uint8_t)pin, value != 0U) ? E_OK : E_SYS;
}

ER pwm_set_pin(UINT pin)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    return hal.pwm_function(hal.context, (uint8_t)pin) ? E_OK : E_SYS;
}

ER pwm_set_wrap(UINT pin, UW wrap)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    return hal.pwm_wrap(hal.context, (uint8_t)pin, (uint16_t)wrap) ? E_OK : E_SYS;
}

ER pwm_set_cc(UINT pin, UW compare)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    return hal.pwm_compare(hal.context, (uint8_t)pin, (uint16_t)compare) ? E_OK : E_SYS;
}

ER pwm_set_enabled(UINT pin, BOOL enabled)
{
    motion_hal_t hal = fake_motion_hal(bsp_fake);
    return hal.pwm_enable(hal.context, (uint8_t)pin, enabled != FALSE) ? E_OK : E_SYS;
}
