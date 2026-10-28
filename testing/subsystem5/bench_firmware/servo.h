#ifndef BUDDY5_SERVO_H
#define BUDDY5_SERVO_H

#include <stdint.h>

typedef struct {
    unsigned pin;
    unsigned slice;
    unsigned channel;
    uint16_t min_us; // pulse at 0 deg
    uint16_t max_us; // pulse at 180 deg
    int angle_deg;
} servo_t;

// 50 Hz hobby servo on any PWM-capable GPIO. Derives the PWM divider from the
// actual system clock, so it stays correct if the clock is changed.
void servo_init(servo_t *s, unsigned pin, uint16_t min_us, uint16_t max_us);
void servo_set_angle(servo_t *s, int angle_deg); // clamped to 0..180

#endif
