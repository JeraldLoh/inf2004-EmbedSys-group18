/* Host tests for the draft Buddy 2 motion executor core. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/subsystem2/motion_executor.h"

static motion_executor_t mx;
static motion_executor_config_t cfg;

static void setup(void)
{
    motion_executor_default_config(&cfg);
    assert(motion_executor_init(&mx, &cfg));
}

static motion_continuous_request_t line_request(authority_generation_t gen,
    int32_t speed, int32_t curvature, uint32_t expires_ms)
{
    motion_continuous_request_t r;
    memset(&r, 0, sizeof(r));
    r.authority_generation = gen;
    r.source = MOTION_SOURCE_LINE;
    r.speed_mm_s = speed;
    r.curvature_u1_m = curvature;
    r.expires_ms = expires_ms;
    return r;
}

static motion_goal_request_t goal(goal_id_t id, authority_generation_t gen,
    motion_goal_type_t type, int32_t mm, int32_t mdeg)
{
    motion_goal_request_t g;
    memset(&g, 0, sizeof(g));
    g.goal_id = id;
    g.authority_generation = gen;
    g.source = MOTION_SOURCE_OBSTACLE;
    g.type = type;
    g.target_mm = mm;
    g.target_mdeg = mdeg;
    return g;
}

/* Counts for mm on the left wheel with the default calibration. */
static uint32_t left_counts(uint32_t mm)
{
    return motion_mm_to_counts(mm, cfg.calibration.left_counts_per_revolution,
        &cfg.calibration);
}

static uint32_t right_counts(uint32_t mm)
{
    return motion_mm_to_counts(mm, cfg.calibration.right_counts_per_revolution,
        &cfg.calibration);
}

static void test_init_rejects_bad_config(void)
{
    motion_executor_config_t bad;
    motion_executor_default_config(&bad);
    bad.calibration.wheelbase_mm = 0U;
    assert(!motion_executor_init(&mx, &bad));
    motion_executor_default_config(&bad);
    bad.max_effort = 1001;
    assert(!motion_executor_init(&mx, &bad));
    motion_executor_default_config(&bad);
    bad.goal_min_effort = 400;
    assert(!motion_executor_init(&mx, &bad));
    assert(!motion_executor_init(NULL, &cfg));
    assert(!motion_executor_init(&mx, NULL));
}

static void test_idle_outputs_zero(void)
{
    motion_executor_output_t out;
    setup();
    out = motion_executor_step(&mx, 0U, 0U, 0U);
    assert(!out.active && out.left == 0 && out.right == 0);
}

static void test_straight_line_request_matches_b3_bench(void)
{
    motion_continuous_request_t r = line_request(1U, 150, 0, 100U);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.active && out.generation == 1U);
    assert(out.left == 199 && out.right == 199); /* 150 * 133 / 100 */
}

static void test_curvature_steers_and_never_reverses(void)
{
    /* Positive curvature = turn left: right wheel faster. */
    motion_continuous_request_t r = line_request(1U, 150, 5000000, 100U);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.right > out.left && out.left > 0);
    /* Very tight: inner wheel clamps to 0, outer to max, never negative. */
    r = line_request(1U, 150, 40000000, 100U);
    assert(motion_executor_continuous(&mx, &r, 5U));
    out = motion_executor_step(&mx, 10U, 0U, 0U);
    assert(out.left == 0 && out.right == cfg.max_effort);
    out = motion_executor_step(&mx, 12U, 0U, 0U);   /* after the deadtime too */
    assert(out.left == 0 && out.right == cfg.max_effort);
    r = line_request(1U, 150, -40000000, 100U);
    assert(motion_executor_continuous(&mx, &r, 10U));
    out = motion_executor_step(&mx, 15U, 0U, 0U);
    assert(out.right == 0 && out.left == cfg.max_effort);
    /* Still clamped once the deadtime period is over. */
    out = motion_executor_step(&mx, 20U, 0U, 0U);
    assert(out.right == 0 && out.left == cfg.max_effort);
}

static void test_speed_cap_applies(void)
{
    motion_continuous_request_t r = line_request(1U, 200, 0, 100U);
    motion_executor_output_t out;
    setup();
    r.speed_cap_mm_s = 100U;
    assert(motion_executor_continuous(&mx, &r, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.left == 133 && out.right == 133);
}

static void test_continuous_expires(void)
{
    motion_continuous_request_t r = line_request(1U, 150, 0, 100U);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    out = motion_executor_step(&mx, 99U, 0U, 0U);
    assert(out.active);
    out = motion_executor_step(&mx, 100U, 0U, 0U);
    assert(!out.active && out.left == 0 && out.right == 0);
    /* Already expired on arrival: rejected. */
    assert(!motion_executor_continuous(&mx, &r, 100U));
}

static void test_rejects_invalid_requests(void)
{
    motion_continuous_request_t r = line_request(0U, 150, 0, 100U);
    motion_goal_request_t g;
    setup();
    assert(!motion_executor_continuous(&mx, &r, 0U));          /* gen 0 */
    r = line_request(1U, 251, 0, 100U);
    assert(!motion_executor_continuous(&mx, &r, 0U));          /* too fast */
    g = goal(1U, 1U, MOTION_GOAL_FORWARD, 0, 0);
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));        /* 0 mm */
    g = goal(1U, 1U, MOTION_GOAL_FORWARD, 5001, 0);
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    g = goal(1U, 1U, MOTION_GOAL_PIVOT_LEFT, 0, 0);
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));        /* 0 deg */
    g = goal(0U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));        /* id 0 */
    g = goal(1U, 1U, (motion_goal_type_t)9, 100, 0);
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    assert(mx.rejected == 7U && mx.accepted == 0U);
}

static void test_older_generation_is_stale(void)
{
    motion_continuous_request_t r = line_request(5U, 150, 0, 100U);
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    r.authority_generation = 4U;
    assert(!motion_executor_continuous(&mx, &r, 0U));
    r.authority_generation = 5U;
    assert(motion_executor_continuous(&mx, &r, 0U));   /* refresh */
    r.authority_generation = 6U;
    assert(motion_executor_continuous(&mx, &r, 0U));   /* newer */
}

static void test_forward_goal_completes_on_counts(void)
{
    motion_goal_request_t g = goal(7U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    motion_goal_result_t res;
    motion_executor_output_t out;
    uint32_t l = left_counts(100U), r = right_counts(100U);
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 1000U, 2000U));
    out = motion_executor_step(&mx, 5U, 1000U, 2000U);
    assert(out.active && out.left > 0 && out.right > 0);
    assert(out.left >= cfg.goal_min_effort);
    assert(motion_executor_goal_result(&mx, 7U, &res));
    assert(res.code == MOTION_RESULT_PROGRESS);
    /* Left finished first: left stops, right keeps going. */
    out = motion_executor_step(&mx, 10U, 1000U + l, 2000U + r - 5U);
    assert(out.left == 0 && out.right > 0);
    out = motion_executor_step(&mx, 15U, 1000U + l, 2000U + r);
    assert(!out.active && out.left == 0 && out.right == 0);
    assert(motion_executor_goal_result(&mx, 7U, &res));
    assert(res.code == MOTION_RESULT_COMPLETED && res.odometry_valid);
    assert(res.travelled_mm >= 99 && res.travelled_mm <= 101);
    assert(!motion_executor_goal_result(&mx, 8U, &res));
}

static void test_reverse_and_pivot_signs(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_REVERSE, 100, 0);
    motion_goal_result_t res;
    motion_executor_output_t out;
    uint32_t arc;
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.left < 0 && out.right < 0);
    out = motion_executor_step(&mx, 10U, left_counts(100U), right_counts(100U));
    assert(motion_executor_goal_result(&mx, 1U, &res));
    assert(res.code == MOTION_RESULT_COMPLETED && res.travelled_mm < 0);

    setup();
    g = goal(2U, 1U, MOTION_GOAL_PIVOT_LEFT, 0, 90000);
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.left < 0 && out.right > 0);
    arc = motion_pivot_wheel_distance_mm(90000U, &cfg.calibration);
    out = motion_executor_step(&mx, 10U, left_counts(arc), right_counts(arc));
    assert(motion_executor_goal_result(&mx, 2U, &res));
    assert(res.code == MOTION_RESULT_COMPLETED);
    assert(res.turned_mdeg > 85000 && res.turned_mdeg < 95000);

    setup();
    g = goal(3U, 1U, MOTION_GOAL_PIVOT_RIGHT, 0, 30000);
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.left > 0 && out.right < 0);
}

static void test_goal_slows_near_the_end(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_FORWARD, 1000, 0);
    motion_executor_output_t fast, slow;
    setup();
    cfg.goal_min_effort = 0;
    assert(motion_executor_init(&mx, &cfg));
    g.max_speed_mm_s = 240U;
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    fast = motion_executor_step(&mx, 5U, 0U, 0U);
    slow = motion_executor_step(&mx, 10U, left_counts(800U), right_counts(800U));
    assert(fast.left == 319 && slow.left == 159);
}

static void test_goal_keeps_wheels_level(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_FORWARD, 1000, 0);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    /* Left 10 mm ahead: left slowed, right unchanged. */
    out = motion_executor_step(&mx, 5U, left_counts(110U), right_counts(100U));
    assert(out.left < out.right);
    assert(out.right == 159 || out.right == cfg.goal_min_effort);
}

static void test_goal_timeout(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    motion_goal_result_t res;
    motion_executor_output_t out;
    setup();
    g.timeout_ms = 500U;
    assert(motion_executor_goal(&mx, &g, 1000U, 0U, 0U));
    out = motion_executor_step(&mx, 1499U, 0U, 0U);
    assert(out.active);
    out = motion_executor_step(&mx, 1500U, 0U, 0U);
    assert(!out.active && out.left == 0);
    assert(motion_executor_goal_result(&mx, 1U, &res));
    assert(res.code == MOTION_RESULT_TIMEOUT);
}

static void test_new_command_cancels_goal(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    motion_goal_request_t g2 = goal(2U, 1U, MOTION_GOAL_PIVOT_LEFT, 0, 10000);
    motion_continuous_request_t r = line_request(2U, 150, 0, 100U);
    motion_goal_result_t res;
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    assert(motion_executor_goal(&mx, &g2, 0U, 0U, 0U));
    assert(motion_executor_goal_result(&mx, 1U, &res));
    assert(res.code == MOTION_RESULT_CANCELLED);
    assert(motion_executor_continuous(&mx, &r, 0U));
    assert(motion_executor_goal_result(&mx, 2U, &res));
    assert(res.code == MOTION_RESULT_CANCELLED);
}

static void test_halt_blocks_inflight_line_requests_only(void)
{
    motion_continuous_request_t r = line_request(3U, 150, 0, 100U);
    motion_goal_request_t g = goal(1U, 3U, MOTION_GOAL_PIVOT_LEFT, 0, 10000);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    motion_executor_halt(&mx);
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(!out.active && out.left == 0);
    /* The same-generation line request already in flight is refused... */
    assert(!motion_executor_continuous(&mx, &r, 5U));
    /* ...but an obstacle goal with the same authority is fine... */
    assert(motion_executor_goal(&mx, &g, 5U, 0U, 0U));
    /* ...and line tracking resumes with fresh authority. */
    r.authority_generation = 4U;
    assert(motion_executor_continuous(&mx, &r, 5U));
}

static void test_halt_cancels_goal_without_odometry(void)
{
    motion_goal_request_t g = goal(1U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    motion_goal_result_t res;
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    motion_executor_halt(&mx);
    assert(motion_executor_goal_result(&mx, 1U, &res));
    assert(res.code == MOTION_RESULT_CANCELLED && !res.odometry_valid);
    /* Another goal with the same authority is still accepted. */
    g.goal_id = 2U;
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
}

static void test_abort_needs_fresh_authority(void)
{
    motion_goal_request_t g = goal(1U, 6U, MOTION_GOAL_FORWARD, 100, 0);
    motion_continuous_request_t r = line_request(6U, 150, 0, 100U);
    motion_goal_result_t res;
    setup();
    assert(motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    motion_executor_abort(&mx, MOTION_RESULT_ABORTED);
    assert(motion_executor_goal_result(&mx, 1U, &res));
    assert(res.code == MOTION_RESULT_ABORTED);
    g.goal_id = 2U;
    assert(!motion_executor_goal(&mx, &g, 0U, 0U, 0U));
    assert(!motion_executor_continuous(&mx, &r, 0U));
    r.authority_generation = 7U;
    assert(motion_executor_continuous(&mx, &r, 0U));
}

static void test_direction_change_gets_a_zero_period(void)
{
    motion_goal_request_t fwd = goal(1U, 1U, MOTION_GOAL_FORWARD, 100, 0);
    motion_goal_request_t rev = goal(2U, 1U, MOTION_GOAL_REVERSE, 100, 0);
    motion_executor_output_t out;
    setup();
    assert(motion_executor_goal(&mx, &fwd, 0U, 0U, 0U));
    out = motion_executor_step(&mx, 5U, 0U, 0U);
    assert(out.left > 0);
    assert(motion_executor_goal(&mx, &rev, 5U, 0U, 0U));
    out = motion_executor_step(&mx, 10U, 0U, 0U);
    assert(out.active && out.left == 0 && out.right == 0);
    out = motion_executor_step(&mx, 15U, 0U, 0U);
    assert(out.left < 0 && out.right < 0);
}

static void test_measured_speed(void)
{
    motion_continuous_request_t r = line_request(1U, 150, 0, 1000U);
    setup();
    assert(motion_executor_continuous(&mx, &r, 0U));
    (void)motion_executor_step(&mx, 0U, 0U, 0U);
    (void)motion_executor_step(&mx, 20U, left_counts(2U), right_counts(2U));
    assert(mx.left_mm_s == 0);   /* window not over yet */
    (void)motion_executor_step(&mx, 100U, left_counts(15U), right_counts(15U));
    assert(mx.left_mm_s >= 145 && mx.left_mm_s <= 155);
    assert(mx.right_mm_s >= 145 && mx.right_mm_s <= 155);
}

int main(void)
{
    test_init_rejects_bad_config();
    test_idle_outputs_zero();
    test_straight_line_request_matches_b3_bench();
    test_curvature_steers_and_never_reverses();
    test_speed_cap_applies();
    test_continuous_expires();
    test_rejects_invalid_requests();
    test_older_generation_is_stale();
    test_forward_goal_completes_on_counts();
    test_reverse_and_pivot_signs();
    test_goal_slows_near_the_end();
    test_goal_keeps_wheels_level();
    test_goal_timeout();
    test_new_command_cancels_goal();
    test_halt_blocks_inflight_line_requests_only();
    test_halt_cancels_goal_without_odometry();
    test_abort_needs_fresh_authority();
    test_direction_change_gets_a_zero_period();
    test_measured_speed();
    puts("motion executor host tests: PASS");
    return 0;
}
