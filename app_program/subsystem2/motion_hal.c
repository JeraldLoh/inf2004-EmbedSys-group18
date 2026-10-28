#include "motion_hal.h"

#include <stddef.h>

bool motion_hal_is_valid(const motion_hal_t *hal)
{
    return (hal != NULL) && (hal->enter != NULL) && (hal->leave != NULL) &&
        (hal->gpio_write != NULL) && (hal->gpio_output != NULL) &&
        (hal->gpio_input != NULL) && (hal->pwm_function != NULL) &&
        (hal->pwm_compare != NULL) && (hal->pwm_wrap != NULL) &&
        (hal->pwm_enable != NULL) && (hal->clock_us != NULL) &&
        (hal->attach_encoders != NULL) && (hal->detach_encoders != NULL);
}
