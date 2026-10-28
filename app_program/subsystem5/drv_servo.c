#include "drv_servo.h"

#include <bsp/libbsp.h>
#include <sys/sysdef.h>

#define SERVO_SYS_CLOCK_MHZ 125U /* fixed by cpu_clock.c */
#define SERVO_PERIOD_US 20000U   /* 50 Hz */
#define SERVO_PWM_SLICE(pin) (((pin) >> 1) & 0x07U)
#define SERVO_PWM_DIV(slice) (PWM_BASE + PWM_CHx_DIV + (0x14U * (slice)))

static uint32_t pulse_us(const drv_servo_t *servo, int angle_deg)
{
    uint32_t span = (uint32_t)(servo->max_us - servo->min_us);
    return servo->min_us + ((span * (uint32_t)angle_deg) + 90U) / 180U;
}

static int clamp_angle(int angle_deg)
{
    if (angle_deg < 0) { return 0; }
    if (angle_deg > 180) { return 180; }
    return angle_deg;
}

ER drv_servo_init(drv_servo_t *servo, UINT pin, uint16_t min_us,
    uint16_t max_us, int start_deg)
{
    if ((servo == NULL) || (pin >= 30U) || (min_us >= max_us) ||
        (max_us >= SERVO_PERIOD_US))
    {
        return E_PAR;
    }
    servo->pin = pin;
    servo->min_us = min_us;
    servo->max_us = max_us;
    servo->angle_deg = clamp_angle(start_deg);

    /* Integer divider in DIV[11:4], no fraction: 125 MHz / 125 = 1 MHz. */
    out_w(SERVO_PWM_DIV(SERVO_PWM_SLICE(pin)), SERVO_SYS_CLOCK_MHZ << 4);
    if ((pwm_set_wrap(pin, SERVO_PERIOD_US - 1U) != E_OK) ||
        (pwm_set_cc(pin, pulse_us(servo, servo->angle_deg)) != E_OK) ||
        (pwm_set_pin(pin) != E_OK) ||
        (pwm_set_enabled(pin, TRUE) != E_OK))
    {
        return E_IO;
    }
    servo->ready = true;
    return E_OK;
}

void drv_servo_set_angle(drv_servo_t *servo, int angle_deg)
{
    if ((servo == NULL) || !servo->ready)
    {
        return;
    }
    servo->angle_deg = clamp_angle(angle_deg);
    (void)pwm_set_cc(servo->pin, pulse_us(servo, servo->angle_deg));
}
