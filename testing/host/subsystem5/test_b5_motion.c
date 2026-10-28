/* Host tests: Buddy 5 moves as authority-stamped Buddy 2 goals. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "b5_motion.h"

/* ---- Fakes for the controller gate, Buddy 2 and Buddy 3 ---------------- */

static struct
{
    bool allowed;
    authority_generation_t generation;
    bool accept_goal;
    int goals;
    motion_goal_request_t last_goal;
    int continuous;
    motion_continuous_request_t last_request;
    bool have_result;
    motion_result_code_t result;
    int halts;
    bool line;
    int reacquires;
    bool reacquire_left;
} f;

static bool fake_authority(motion_source_t source, authority_generation_t *generation)
{
    assert(source == MOTION_SOURCE_OBSTACLE);
    if (!f.allowed) { return false; }
    *generation = f.generation;
    return true;
}

static bool fake_goal(const motion_goal_request_t *goal)
{
    f.goals++;
    f.last_goal = *goal;
    return f.accept_goal;
}

static bool fake_continuous(const motion_continuous_request_t *request)
{
    f.continuous++;
    f.last_request = *request;
    return true;
}

static bool fake_goal_result(goal_id_t goal_id, motion_goal_result_t *result)
{
    if (!f.have_result || (goal_id != f.last_goal.goal_id)) { return false; }
    memset(result, 0, sizeof(*result));
    result->goal_id = goal_id;
    result->code = f.result;
    return true;
}

static void fake_halt(void) { f.halts++; }
static bool fake_line_seen(void) { return f.line; }
static void fake_reacquire(bool toward_left)
{
    f.reacquires++;
    f.reacquire_left = toward_left;
}

static const b5_motion_ops_t ops = {
    fake_authority, fake_goal, fake_continuous, fake_goal_result,
    fake_halt, fake_line_seen, fake_reacquire,
};

static b5_motion_t m;

static void setup(void)
{
    memset(&f, 0, sizeof(f));
    f.allowed = true;
    f.generation = 7U;
    f.accept_goal = true;
    f.have_result = true;
    f.result = MOTION_RESULT_PROGRESS;
    b5_motion_init(&m, &ops, NULL);
}

/* ---- Tests ------------------------------------------------------------- */

static void test_ops_all_or_nothing(void)
{
    b5_motion_ops_t none;
    b5_motion_ops_t half = ops;
    memset(&none, 0, sizeof(none));
    half.halt = NULL;
    assert(b5_motion_ops_valid(&ops) && b5_motion_ops_present(&ops));
    assert(b5_motion_ops_valid(&none) && !b5_motion_ops_present(&none));
    assert(!b5_motion_ops_valid(&half) && !b5_motion_ops_present(&half));
    assert(!b5_motion_ops_valid(NULL));
}

static void test_without_ops_service_is_a_no_op(void)
{
    b5_motion_ops_t none;
    memset(&none, 0, sizeof(none));
    b5_motion_init(&m, &none, NULL);
    b5_motion_service(&m, 0U);
    assert(b5_motion_done(&m) && !b5_motion_failed(&m));
    assert(!b5_motion_line_seen(&m));
}

static void test_turn_becomes_authority_stamped_pivot(void)
{
    setup();
    b5_motion_turn(&m, 30000, 100U);
    assert(m.state == B5_MOVE_PENDING && !b5_motion_done(&m));
    b5_motion_service(&m, 110U);
    assert(f.goals == 1 && m.state == B5_MOVE_GOAL);
    assert(f.last_goal.type == MOTION_GOAL_PIVOT_LEFT);
    assert(f.last_goal.target_mdeg == 30000);
    assert(f.last_goal.authority_generation == 7U);
    assert(f.last_goal.source == MOTION_SOURCE_OBSTACLE);
    assert(f.last_goal.goal_id != 0U);
    assert(f.last_goal.max_speed_mm_s == m.config.goal_speed_mm_s);
    assert(f.last_goal.timeout_ms == m.config.goal_timeout_ms);

    b5_motion_turn(&m, -45000, 200U);    /* right */
    b5_motion_service(&m, 200U);
    assert(f.last_goal.type == MOTION_GOAL_PIVOT_RIGHT);
    assert(f.last_goal.target_mdeg == 45000);
}

static void test_straight_moves(void)
{
    setup();
    b5_motion_straight(&m, 300, false, 0U);
    b5_motion_service(&m, 0U);
    assert(f.last_goal.type == MOTION_GOAL_FORWARD && f.last_goal.target_mm == 300);
    b5_motion_straight(&m, 150, true, 0U);
    b5_motion_service(&m, 0U);
    assert(f.last_goal.type == MOTION_GOAL_REVERSE && f.last_goal.target_mm == 150);
    assert(f.goals == 2);
}

static void test_zero_moves_complete_at_once(void)
{
    setup();
    b5_motion_turn(&m, 0, 0U);
    assert(b5_motion_done(&m));
    b5_motion_straight(&m, 0, false, 0U);
    assert(b5_motion_done(&m));
    b5_motion_service(&m, 0U);
    assert(f.goals == 0);
}

static void test_waits_for_authority_then_submits(void)
{
    setup();
    f.allowed = false;   /* controller has not entered the obstacle mission */
    b5_motion_turn(&m, 10000, 1000U);
    b5_motion_service(&m, 1100U);
    assert(m.state == B5_MOVE_PENDING && f.goals == 0);
    f.allowed = true;
    f.generation = 8U;
    b5_motion_service(&m, 1200U);
    assert(m.state == B5_MOVE_GOAL && f.last_goal.authority_generation == 8U);
}

static void test_rejected_submission_is_retried(void)
{
    setup();
    f.accept_goal = false;   /* stale authority still in flight */
    b5_motion_turn(&m, 10000, 0U);
    b5_motion_service(&m, 10U);
    assert(m.state == B5_MOVE_PENDING && f.goals == 1);
    f.accept_goal = true;
    b5_motion_service(&m, 20U);
    assert(m.state == B5_MOVE_GOAL && f.goals == 2);
}

static void test_no_authority_fails_after_wait(void)
{
    setup();
    f.allowed = false;
    b5_motion_turn(&m, 10000, 1000U);
    b5_motion_service(&m, 1000U + m.config.wait_ms - 1U);
    assert(!b5_motion_failed(&m));
    b5_motion_service(&m, 1000U + m.config.wait_ms);
    assert(b5_motion_failed(&m) && !b5_motion_done(&m));
    assert(m.last_result == MOTION_RESULT_REJECTED);

    /* Authority arriving exactly at the deadline still counts. */
    setup();
    f.allowed = false;
    b5_motion_turn(&m, 10000, 1000U);
    b5_motion_service(&m, 1100U);
    f.allowed = true;
    b5_motion_service(&m, 1000U + m.config.wait_ms);
    assert(m.state == B5_MOVE_GOAL && !b5_motion_failed(&m));
}

static void test_goal_results(void)
{
    setup();
    b5_motion_turn(&m, 10000, 0U);
    b5_motion_service(&m, 0U);
    b5_motion_service(&m, 10U);
    assert(m.state == B5_MOVE_GOAL);       /* PROGRESS */
    f.result = MOTION_RESULT_COMPLETED;
    b5_motion_service(&m, 20U);
    assert(b5_motion_done(&m) && !b5_motion_failed(&m));
    assert(m.last_result == MOTION_RESULT_COMPLETED);

    setup();
    b5_motion_straight(&m, 100, false, 0U);
    b5_motion_service(&m, 0U);
    f.result = MOTION_RESULT_TIMEOUT;
    b5_motion_service(&m, 10U);
    assert(b5_motion_failed(&m) && m.last_result == MOTION_RESULT_TIMEOUT);

    setup();
    b5_motion_straight(&m, 100, false, 0U);
    b5_motion_service(&m, 0U);
    f.have_result = false;                 /* executor lost the goal */
    b5_motion_service(&m, 10U);
    assert(b5_motion_failed(&m) && m.last_result == MOTION_RESULT_FAULT);
}

static void test_stop_halts_and_clears(void)
{
    setup();
    b5_motion_turn(&m, 10000, 0U);
    b5_motion_service(&m, 0U);
    b5_motion_stop(&m);
    assert(f.halts == 1 && b5_motion_done(&m) && !b5_motion_failed(&m));
    b5_motion_service(&m, 10U);
    assert(f.goals == 1);                  /* nothing resubmitted */
}

static void test_search_arcs_toward_the_line(void)
{
    setup();
    b5_motion_search(&m, true);
    assert(f.reacquires == 1 && f.reacquire_left);
    b5_motion_service(&m, 500U);
    assert(f.continuous == 1);
    assert(f.last_request.source == MOTION_SOURCE_OBSTACLE);
    assert(f.last_request.authority_generation == 7U);
    assert(f.last_request.speed_mm_s == m.config.search_speed_mm_s);
    assert(f.last_request.curvature_u1_m > 0);   /* left */
    assert(f.last_request.expires_ms == 500U + m.config.search_life_ms);
    b5_motion_service(&m, 510U);
    assert(f.continuous == 2);                   /* refreshed every tick */

    b5_motion_search(&m, false);
    b5_motion_service(&m, 520U);
    assert(!f.reacquire_left && f.last_request.curvature_u1_m < 0);

    f.allowed = false;                           /* not allowed: nothing sent */
    b5_motion_service(&m, 530U);
    assert(f.continuous == 3);

    b5_motion_resume(&m);
    b5_motion_service(&m, 540U);
    assert(f.continuous == 3 && b5_motion_done(&m));
}

static void test_line_seen(void)
{
    setup();
    assert(!b5_motion_line_seen(&m));
    f.line = true;
    assert(b5_motion_line_seen(&m));
}

int main(void)
{
    test_ops_all_or_nothing();
    test_without_ops_service_is_a_no_op();
    test_turn_becomes_authority_stamped_pivot();
    test_straight_moves();
    test_zero_moves_complete_at_once();
    test_waits_for_authority_then_submits();
    test_rejected_submission_is_retried();
    test_no_authority_fails_after_wait();
    test_goal_results();
    test_stop_halts_and_clears();
    test_search_arcs_toward_the_line();
    test_line_seen();
    puts("b5 motion host tests: PASS");
    return 0;
}
