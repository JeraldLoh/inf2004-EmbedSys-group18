#ifndef GROUP18_DRV_SERVO_H
#define GROUP18_DRV_SERVO_H

/* Buddy 5: scan servo on the kernel image (target only).
 * 50 Hz PWM with 1 us per count: divider 125 from the fixed 125 MHz system
 * clock (kernel/sysdepend/pico_rp2040/cpu_clock.c), wrap 19999. GP12 is
 * PWM slice 6, which no other subsystem uses (motors are slices 4 and 5).
 */
#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

typedef struct
{
    UINT pin;
    uint16_t min_us; /* pulse at 0 deg */
    uint16_t max_us; /* pulse at 180 deg */
    int angle_deg;
    bool ready;
} drv_servo_t;

ER drv_servo_init(drv_servo_t *servo, UINT pin, uint16_t min_us,
    uint16_t max_us, int start_deg);
/* Clamped to 0..180. */
void drv_servo_set_angle(drv_servo_t *servo, int angle_deg);

#endif
