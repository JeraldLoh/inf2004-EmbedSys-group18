#include <assert.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem2/motion_math.h"

static const motion_calibration_t calibration =
{
    65U,
    140U,
    20U,
    20U
};

static void test_invalid_calibration_fails_closed(void)
{
    motion_calibration_t invalid = calibration;

    invalid.left_counts_per_revolution = 0U;
    assert(!motion_calibration_is_valid(&invalid));
    assert(motion_counts_to_mm(10U, 0U, &invalid) == 0U);
    assert(motion_mm_to_counts(100U, 0U, &invalid) == 0U);
    assert(motion_pivot_wheel_distance_mm(90000U, &invalid) == 0U);
}

static void test_count_distance_round_trip_is_conservative(void)
{
    uint32_t requested_mm = 200U;
    uint32_t counts = motion_mm_to_counts(
        requested_mm,
        calibration.left_counts_per_revolution,
        &calibration);
    uint32_t represented_mm = motion_counts_to_mm(
        counts,
        calibration.left_counts_per_revolution,
        &calibration);

    assert(counts > 0U);
    assert(represented_mm >= requested_mm);
    assert((represented_mm - requested_mm) <= 11U);
}

static void test_quarter_turn_uses_millidegrees(void)
{
    uint32_t distance = motion_pivot_wheel_distance_mm(90000U, &calibration);

    /* pi * 140 / 4 = about 110 mm for each wheel. */
    assert(distance >= 109U);
    assert(distance <= 111U);
}

static void test_extreme_inputs_saturate(void)
{
    motion_calibration_t extreme = calibration;

    extreme.wheel_diameter_mm = UINT32_MAX;
    extreme.wheelbase_mm = UINT32_MAX;
    extreme.left_counts_per_revolution = UINT32_MAX;
    assert(motion_counts_to_mm(
        UINT32_MAX,
        extreme.left_counts_per_revolution,
        &extreme) == UINT32_MAX);
    assert(motion_mm_to_counts(
        UINT32_MAX,
        extreme.left_counts_per_revolution,
        &extreme) == UINT32_MAX);
    assert(motion_pivot_wheel_distance_mm(
        UINT32_MAX,
        &extreme) == UINT32_MAX);
}

int main(void)
{
    assert(motion_calibration_is_valid(&calibration));
    test_invalid_calibration_fails_closed();
    test_count_distance_round_trip_is_conservative();
    test_quarter_turn_uses_millidegrees();
    test_extreme_inputs_saturate();
    (void)puts("motion math host tests passed");
    return 0;
}
