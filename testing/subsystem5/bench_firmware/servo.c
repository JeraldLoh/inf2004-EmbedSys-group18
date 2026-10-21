#include "servo.h"

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

#define SERVO_PERIOD_US 20000u

void servo_init(servo_t *s, unsigned pin, uint16_t min_us, uint16_t max_us) {
    s->pin = pin;
    s->min_us = min_us;
    s->max_us = max_us;
    s->angle_deg = -1;

    gpio_set_function(pin, GPIO_FUNC_PWM);
    s->slice = pwm_gpio_to_slice_num(pin);
    s->channel = pwm_gpio_to_channel(pin);

    // 1 counter tick = 1 us, so the compare level equals the pulse width in us.
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)clock_get_hz(clk_sys) / 1000000.0f);
    pwm_config_set_wrap(&cfg, SERVO_PERIOD_US - 1);
    pwm_init(s->slice, &cfg, false);
    pwm_set_chan_level(s->slice, s->channel, 0); // no pulse until first command
    pwm_set_enabled(s->slice, true);
}

void servo_set_angle(servo_t *s, int angle_deg) {
    if (angle_deg < 0) angle_deg = 0;
    if (angle_deg > 180) angle_deg = 180;
    uint32_t span = (uint32_t)(s->max_us - s->min_us);
    uint32_t pulse = s->min_us + (span * (uint32_t)angle_deg + 90u) / 180u;
    pwm_set_chan_level(s->slice, s->channel, (uint16_t)pulse);
    s->angle_deg = angle_deg;
}
