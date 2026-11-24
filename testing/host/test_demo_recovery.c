#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/wheel_path.h"
#include "../../app_program/vehicle_controller/pickup_guard.h"

static void test_s_bend_reverses_in_order(void)
{
    wheel_path_t path = {0};
    uint64_t totals[2];
    unsigned i;
    for (i = 0U; i < 10U; ++i) { wheel_path_record(&path, 3000U, 1000U); }
    for (i = 0U; i < 10U; ++i) { wheel_path_record(&path, 1000U, 3000U); }
    assert(wheel_path_plan(&path, 40U, totals));
    assert(totals[0] == totals[1]); /* Aggregate reverse would be straight. */
    wheel_path_next(&path, 20000U);
    assert(path.segment_um[0] == 10000U && path.segment_um[1] == 30000U);
    wheel_path_next(&path, 20000U);
    assert(path.segment_um[0] == 30000U && path.segment_um[1] == 10000U);
    assert(path.remaining == 0U && path.waypoint_um[0] == totals[0]);
    assert(path.waypoint_um[1] == totals[1]);
    assert(!wheel_path_plan(&path, 41U, totals));
}

static void test_ring_wrap_and_invalid_gap(void)
{
    wheel_path_t path = {0};
    uint64_t totals[2];
    unsigned i;
    for (i = 0U; i < B2_RETRACE_SAMPLES + 10U; ++i)
    { wheel_path_record(&path, 1000U, 1000U); }
    assert(wheel_path_plan(&path, B2_RETRACE_SAMPLES, totals));
    assert(!wheel_path_plan(&path, B2_RETRACE_SAMPLES + 1U, totals));
    wheel_path_record(&path, 65536U, 1U);
    assert(path.count == 0U);
}

static bool sample(pickup_guard_t *guard, uint32_t now, uint16_t sequence,
    bool candidate, bool floor, bool eligible, bool imu)
{
    return pickup_guard_update(guard, now, sequence, true, candidate, floor,
        eligible, imu, 300U, 1000U, 1000U, 200U);
}

static void test_pickup_and_rearm(void)
{
    pickup_guard_t guard = {0};
    unsigned i;
    /* A junction pause cannot accumulate pickup time. */
    for (i = 0U; i <= 20U; ++i)
    { assert(!sample(&guard, i * 100U, (uint16_t)i, true, false, false, true)); }
    assert(!sample(&guard, 2100U, 21U, true, false, true, true));
    assert(!sample(&guard, 2200U, 22U, true, false, true, true));
    assert(!sample(&guard, 2300U, 23U, true, false, true, true));
    assert(sample(&guard, 2400U, 24U, true, false, true, true));
    /* Stale/missing data cannot unlock a stopped car. */
    assert(sample(&guard, 4000U, 24U, false, true, false, false));
    for (i = 0U; i < 10U; ++i)
    { assert(sample(&guard, 4100U + i * 100U, (uint16_t)(30U+i), false, true, false, false)); }
    assert(!sample(&guard, 5100U, 40U, false, true, false, false));
}

static void test_slow_lift_stale_and_clock_wrap(void)
{
    pickup_guard_t guard = {0};
    uint32_t start = UINT32_MAX - 500U;
    unsigned i;
    for (i = 0U; i < 10U; ++i)
    { assert(!sample(&guard, start+i*100U, (uint16_t)i, true, false, true, false)); }
    assert(sample(&guard, start+1000U, 10U, true, false, true, false));
    guard = (pickup_guard_t){0};
    assert(!sample(&guard, 0U, 1U, true, false, true, true));
    assert(!sample(&guard, 200U, 1U, true, false, true, true));
    assert(!sample(&guard, 1000U, 1U, true, false, true, true));
    assert(!guard.latched); /* A frozen sample never establishes pickup. */
    guard = (pickup_guard_t){0};
    assert(!sample(&guard, 0U, 1U, true, false, true, true));
    assert(!sample(&guard, 300U, 2U, true, false, true, true));
    assert(!guard.latched); /* A fresh word after a gap starts a new interval. */
}

int main(void)
{
    test_s_bend_reverses_in_order();
    test_ring_wrap_and_invalid_gap();
    test_pickup_and_rearm();
    test_slow_lift_stale_and_clock_wrap();
    puts("demo recovery regressions passed");
    return 0;
}
