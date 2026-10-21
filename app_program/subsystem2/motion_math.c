#include "motion_math.h"

#include <stddef.h>

/* pi scaled by 1,000,000; avoids floating point in the control path. */
#define MOTION_PI_U6 3141593ULL
#define MOTION_U6 1000000ULL

static uint32_t saturate_u32(uint64_t value)
{
    return (value > UINT32_MAX) ? UINT32_MAX : (uint32_t)value;
}

static bool multiply_overflows_u64(uint64_t left, uint64_t right)
{
    return (right != 0ULL) && (left > (UINT64_MAX / right));
}

bool motion_calibration_is_valid(const motion_calibration_t *calibration)
{
    return (calibration != NULL) &&
        (calibration->wheel_diameter_mm > 0U) &&
        (calibration->wheelbase_mm > 0U) &&
        (calibration->left_counts_per_revolution > 0U) &&
        (calibration->right_counts_per_revolution > 0U);
}

uint32_t motion_counts_to_mm(
    uint32_t counts,
    uint32_t counts_per_revolution,
    const motion_calibration_t *calibration)
{
    uint64_t numerator;
    uint64_t denominator;

    if (!motion_calibration_is_valid(calibration) ||
        (counts_per_revolution == 0U))
    {
        return 0U;
    }

    numerator = (uint64_t)counts * MOTION_PI_U6;
    if (multiply_overflows_u64(
        numerator,
        calibration->wheel_diameter_mm))
    {
        return UINT32_MAX;
    }
    numerator *= calibration->wheel_diameter_mm;
    denominator = MOTION_U6 * counts_per_revolution;
    return saturate_u32(numerator / denominator);
}

uint32_t motion_mm_to_counts(
    uint32_t distance_mm,
    uint32_t counts_per_revolution,
    const motion_calibration_t *calibration)
{
    uint64_t numerator;
    uint64_t denominator;

    if (!motion_calibration_is_valid(calibration) ||
        (counts_per_revolution == 0U))
    {
        return 0U;
    }

    numerator = (uint64_t)distance_mm * counts_per_revolution;
    if (multiply_overflows_u64(numerator, MOTION_U6))
    {
        return UINT32_MAX;
    }
    numerator *= MOTION_U6;
    denominator = MOTION_PI_U6 * calibration->wheel_diameter_mm;

    /* Round up so a finite goal is never shortened by integer truncation. */
    return saturate_u32((numerator + denominator - 1ULL) / denominator);
}

uint32_t motion_pivot_wheel_distance_mm(
    uint32_t angle_mdeg,
    const motion_calibration_t *calibration)
{
    uint64_t numerator;
    uint64_t denominator;

    if (!motion_calibration_is_valid(calibration))
    {
        return 0U;
    }

    /* Each wheel follows radius wheelbase/2 during an in-place pivot. */
    numerator = (uint64_t)calibration->wheelbase_mm * MOTION_PI_U6;
    if (multiply_overflows_u64(numerator, angle_mdeg))
    {
        return UINT32_MAX;
    }
    numerator *= angle_mdeg;
    denominator = 360000ULL * MOTION_U6;
    return saturate_u32((numerator + (denominator / 2ULL)) / denominator);
}
