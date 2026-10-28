#include "motion_hal_rp2040.h"

/* Compile this adapter against the real course BSP, not a second SDK main. */
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

static motion_lock_key_t port_enter(void *context)
{
    motion_rp2040_port_t *port = context;
    return port->enter(port->context);
}

static void port_leave(void *context, motion_lock_key_t key)
{
    motion_rp2040_port_t *port = context;
    port->leave(port->context, key);
}

static bool port_clock(void *context, uint32_t *now)
{
    motion_rp2040_port_t *port = context;
    return port->clock_us(port->context, now);
}

static bool port_attach(void *context, motion_edge_handler_t handler, void *user)
{
    motion_rp2040_port_t *port = context;
    return port->attach_encoders(port->context, handler, user);
}

static bool port_detach(void *context)
{
    motion_rp2040_port_t *port = context;
    return port->detach_encoders(port->context);
}

static bool pin_write(void *context, uint8_t pin, bool high)
{
    (void)context;
    return gpio_set_val(pin, high ? 1U : 0U) == E_OK;
}

static bool pin_output(void *context, uint8_t pin)
{
    (void)context;
    return gpio_set_pin(pin, GPIO_MODE_OUT) == E_OK;
}

static bool pin_input(void *context, uint8_t pin)
{
    (void)context;
    return gpio_set_pin(pin, GPIO_MODE_IN) == E_OK;
}

static bool pin_pwm(void *context, uint8_t pin)
{
    (void)context;
    return pwm_set_pin(pin) == E_OK;
}

static bool pin_compare(void *context, uint8_t pin, uint16_t compare)
{
    (void)context;
    return pwm_set_cc(pin, compare) == E_OK;
}

static bool slice_wrap(void *context, uint8_t pin, uint16_t wrap)
{
    (void)context;
    return pwm_set_wrap(pin, wrap) == E_OK;
}

static bool slice_enable(void *context, uint8_t pin, bool enabled)
{
    (void)context;
    return pwm_set_enabled(pin, enabled ? TRUE : FALSE) == E_OK;
}

bool motion_hal_rp2040_create(motion_hal_t *hal, motion_rp2040_port_t *port)
{
    motion_hal_t candidate;
    if ((hal == NULL) || (port == NULL) || (port->enter == NULL) ||
        (port->leave == NULL) || (port->clock_us == NULL) ||
        (port->attach_encoders == NULL) || (port->detach_encoders == NULL))
    {
        return false;
    }
    candidate.context = port;
    candidate.enter = port_enter;
    candidate.leave = port_leave;
    candidate.gpio_write = pin_write;
    candidate.gpio_output = pin_output;
    candidate.gpio_input = pin_input;
    candidate.pwm_function = pin_pwm;
    candidate.pwm_compare = pin_compare;
    candidate.pwm_wrap = slice_wrap;
    candidate.pwm_enable = slice_enable;
    candidate.clock_us = port_clock;
    candidate.attach_encoders = port_attach;
    candidate.detach_encoders = port_detach;
    *hal = candidate;
    return true;
}
