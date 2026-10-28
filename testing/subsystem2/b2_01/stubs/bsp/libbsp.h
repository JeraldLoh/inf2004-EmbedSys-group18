#ifndef B2_TEST_LIBBSP_H
#define B2_TEST_LIBBSP_H
#include <tk/tkernel.h>
#define GPIO_MODE_OUT 1U
#define GPIO_MODE_IN 0U
ER gpio_set_pin(UINT pin, UINT mode);
ER gpio_set_val(UINT pin, UINT val);
UINT gpio_get_val(UINT pin);
ER pwm_set_pin(UINT pin);
ER pwm_set_wrap(UINT pin, UW wrap);
ER pwm_set_cc(UINT pin, UW compare);
ER pwm_set_enabled(UINT pin, BOOL enabled);
#endif
