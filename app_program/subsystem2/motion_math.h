#ifndef GROUP18_MOTION_MATH_H
#define GROUP18_MOTION_MATH_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Hardware-independent motion conversions retained from Buddy 2's standalone
 * prototype.  The integrated driver/task supplies measured calibration; this
 * module never configures GPIO, PWM, interrupts, or motors.
 */
typedef struct
{
    uint32_t wheel_diameter_mm;
    uint32_t wheelbase_mm;
    uint32_t left_counts_per_revolution;
    uint32_t right_counts_per_revolution;
} motion_calibration_t;

bool motion_calibration_is_valid(const motion_calibration_t *calibration);

uint32_t motion_counts_to_mm(
    uint32_t counts,
    uint32_t counts_per_revolution,
    const motion_calibration_t *calibration);

uint32_t motion_mm_to_counts(
    uint32_t distance_mm,
    uint32_t counts_per_revolution,
    const motion_calibration_t *calibration);

uint32_t motion_pivot_wheel_distance_mm(
    uint32_t angle_mdeg,
    const motion_calibration_t *calibration);

#endif
