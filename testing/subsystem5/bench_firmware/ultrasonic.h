#ifndef BUDDY5_ULTRASONIC_H
#define BUDDY5_ULTRASONIC_H

// HC-SR04 driver. Echo edges are timestamped in a GPIO interrupt, so the
// measurement is not skewed by other interrupts (e.g. USB) during polling.
// Only one instance is supported.
//
// WARNING: a standard HC-SR04 drives ECHO at 5 V. The RP2040 is not 5 V
// tolerant: use a divider (e.g. 1k/2k) on ECHO or a 3.3 V HC-SR04P.

#include <stddef.h>
#include <stdint.h>

#include "range.h"

typedef struct {
    unsigned trig_pin;
    unsigned echo_pin;
    float min_cm;
    float max_cm;
    float cm_per_us; // one-way: half the speed of sound
} ultrasonic_t;

void ultrasonic_init(ultrasonic_t *u, unsigned trig_pin, unsigned echo_pin);
// Speed of sound varies ~0.6 m/s per degree C.
void ultrasonic_set_temperature_c(ultrasonic_t *u, float celsius);
range_reading_t ultrasonic_read_once(const ultrasonic_t *u);
// Median-filtered reading over `samples` pings (<= RANGE_MAX_SAMPLES).
range_reading_t ultrasonic_read(const ultrasonic_t *u, size_t samples, uint32_t spacing_ms);

#endif
