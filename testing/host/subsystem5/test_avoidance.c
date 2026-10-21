#include <stdint.h>
#include <string.h>

#include "avoidance.h"
#include "test.h"

// ---------------------------------------------------------------------------
// Simulated robot + world
// ---------------------------------------------------------------------------

#define MAX_EVENTS 1024
#define MAX_SEARCHES 16
#define MEASURE_COST_MS 90 // 3 pings with spacing, as on the real board
#define TICK_MS 10

typedef struct {
    int lo, hi; // servo angle range covered (inclusive)
    float cm;
    bool present;
} blob_t;

typedef struct {
    av_config_t cfg; // to check settle times against
    uint32_t t;

    int servo;
    bool servo_known;
    uint32_t servo_ready_at;
    bool settle_violation;
    int min_angle, max_angle;

    blob_t obstacle;
    blob_t wall;
    blob_t late_obstacle; // appears when bypass starts
    float bg_cm;          // <= 0 means no echo
    bool force;
    range_status_t forced_status;
    int spikes; // next N readings are a 5 cm spike

    bool motion_active;
    uint32_t motion_end;
    uint32_t bypass_ms, reverse_ms;
    bool bypass_never_done;
    bool vanish_on_stop;       // obstacle disappears on the first stop
    bool open_on_reverse;      // boxed-in walls vanish when reversing
    bool vanish_late_on_estop; // late obstacle vanishes on an emergency stop

    int stops, reverses, bypasses, searches, resumes;
    side_t last_bypass;
    side_t search_sides[MAX_SEARCHES];
    bool searching;
    uint32_t search_start;
    uint32_t line_after_ms;
    int line_on_search; // line only found during this search number (1-based); 0 = any
    bool line_never;

    av_event_type_t events[MAX_EVENTS];
    int n_events;
    const char *last_reason;
} sim_t;

static bool in_blob(const blob_t *b, int a) { return b->present && a >= b->lo && a <= b->hi; }

static uint32_t s_now(void *ctx) { return ((sim_t *)ctx)->t; }

static void s_servo(void *ctx, int deg) {
    sim_t *s = ctx;
    int delta = s->servo_known ? abs(deg - s->servo) : 180;
    uint32_t need = delta == 0 ? 0 : s->cfg.servo_settle_base_ms + (uint32_t)(s->cfg.servo_ms_per_deg * (float)delta + 0.5f);
    s->servo_ready_at = s->t + need;
    s->servo = deg;
    s->servo_known = true;
    if (deg < s->min_angle) s->min_angle = deg;
    if (deg > s->max_angle) s->max_angle = deg;
}

static range_reading_t s_measure(void *ctx) {
    sim_t *s = ctx;
    if ((int32_t)(s->t - s->servo_ready_at) < 0) s->settle_violation = true;
    s->t += MEASURE_COST_MS;

    if (s->force) return (range_reading_t){s->forced_status, 0.0f};
    if (s->spikes > 0) {
        s->spikes--;
        return (range_reading_t){RANGE_OK, 5.0f};
    }
    int a = s->servo;
    if (in_blob(&s->late_obstacle, a)) return (range_reading_t){RANGE_OK, s->late_obstacle.cm};
    if (in_blob(&s->obstacle, a)) return (range_reading_t){RANGE_OK, s->obstacle.cm};
    if (in_blob(&s->wall, a)) return (range_reading_t){RANGE_OK, s->wall.cm};
    if (s->bg_cm <= 0.0f) return (range_reading_t){RANGE_NO_ECHO, 0.0f};
    return (range_reading_t){RANGE_OK, s->bg_cm};
}

static void s_stop(void *ctx) {
    sim_t *s = ctx;
    s->stops++;
    s->motion_active = false;
    s->searching = false;
    if (s->vanish_on_stop && s->stops == 1) s->obstacle.present = false;
    if (s->vanish_late_on_estop && s->late_obstacle.present) s->late_obstacle.present = false;
}

static void s_reverse(void *ctx, float cm) {
    sim_t *s = ctx;
    (void)cm;
    s->reverses++;
    s->motion_active = true;
    s->motion_end = s->t + s->reverse_ms;
    if (s->open_on_reverse) {
        s->bg_cm = 150.0f;
        s->obstacle = (blob_t){80, 100, 12.0f, true};
    }
}

static void s_bypass(void *ctx, side_t side, const obstacle_profile_t *p) {
    sim_t *s = ctx;
    (void)p;
    s->bypasses++;
    s->last_bypass = side;
    s->motion_active = true;
    s->motion_end = s->t + s->bypass_ms;
    if (s->late_obstacle.cm > 0.0f) s->late_obstacle.present = true;
}

static void s_search(void *ctx, side_t toward) {
    sim_t *s = ctx;
    if (s->searches < MAX_SEARCHES) s->search_sides[s->searches] = toward;
    s->searches++;
    s->searching = true;
    s->search_start = s->t;
}

static void s_resume(void *ctx) {
    sim_t *s = ctx;
    s->resumes++;
    s->searching = false;
}

static bool s_done(void *ctx) {
    sim_t *s = ctx;
    if (!s->motion_active) return true;
    if (s->bypass_never_done && s->bypasses > 0) return false;
    return (int32_t)(s->t - s->motion_end) >= 0;
}

static bool s_line(void *ctx) {
    sim_t *s = ctx;
    if (s->line_never || !s->searching) return false;
    if (s->line_on_search != 0 && s->searches != s->line_on_search) return false;
    return s->t - s->search_start >= s->line_after_ms;
}

static void s_event(void *ctx, const av_event_t *e) {
    sim_t *s = ctx;
    if (s->n_events < MAX_EVENTS) s->events[s->n_events++] = e->type;
    if (e->reason) s->last_reason = e->reason;
}

static av_platform_t platform_for(sim_t *s) {
    av_platform_t p = {
        .ctx = s,
        .now_ms = s_now,
        .servo_set = s_servo,
        .measure = s_measure,
        .motion_stop = s_stop,
        .motion_reverse = s_reverse,
        .motion_bypass = s_bypass,
        .motion_search_line = s_search,
        .motion_resume_line_follow = s_resume,
        .motion_done = s_done,
        .line_detected = s_line,
        .on_event = s_event,
    };
    return p;
}

static void sim_setup(sim_t *s, av_t *av, uint32_t start_ms) {
    memset(s, 0, sizeof *s);
    av_config_default(&s->cfg);
    s->t = start_ms;
    s->min_angle = 999;
    s->max_angle = -999;
    s->bg_cm = 150.0f;
    s->bypass_ms = 1500;
    s->reverse_ms = 600;
    s->line_after_ms = 300;
    av_platform_t p = platform_for(s);
    bool ok = av_init(av, &s->cfg, &p);
    CHECK(ok);
}

static void run_ms(sim_t *s, av_t *av, uint32_t ms) {
    uint32_t end = s->t + ms;
    while ((int32_t)(s->t - end) < 0) {
        av_tick(av);
        s->t += TICK_MS;
    }
}

static bool run_until(sim_t *s, av_t *av, av_state_t target, uint32_t max_ms) {
    uint32_t end = s->t + max_ms;
    while ((int32_t)(s->t - end) < 0) {
        av_tick(av);
        if (av_state(av) == target) return true;
        s->t += TICK_MS;
    }
    return false;
}

static int count_events(const sim_t *s, av_event_type_t type) {
    int n = 0;
    for (int i = 0; i < s->n_events; i++) n += s->events[i] == type;
    return n;
}

static int first_event(const sim_t *s, av_event_type_t type) {
    for (int i = 0; i < s->n_events; i++)
        if (s->events[i] == type) return i;
    return -1;
}

static void check_hygiene(const sim_t *s) {
    CHECK(!s->settle_violation);
    CHECK(s->min_angle >= 0);
    CHECK(s->max_angle <= 180);
}

// Runs a full encounter until back in MONITOR; returns false on timeout.
static bool run_encounter(sim_t *s, av_t *av) {
    if (!run_until(s, av, AV_STATE_COARSE_SCAN, 5000)) return false;
    return run_until(s, av, AV_STATE_MONITOR, 30000);
}

// ---------------------------------------------------------------------------
// Configuration / init
// ---------------------------------------------------------------------------

static void default_config_is_valid(void) {
    av_config_t c;
    av_config_default(&c);
    CHECK(av_config_check(&c) == NULL);
}

static void invalid_configs_rejected(void) {
    av_config_t c;
#define EXPECT_BAD(mutation)                        \
    do {                                            \
        av_config_default(&c);                      \
        mutation;                                   \
        CHECK(av_config_check(&c) != NULL);         \
    } while (0)
    EXPECT_BAD(c.coarse_points = 4);
    EXPECT_BAD(c.coarse_points = 1);
    EXPECT_BAD(c.coarse_points = AV_MAX_COARSE_POINTS + 2);
    EXPECT_BAD(c.center_deg = 91);
    EXPECT_BAD(c.left_deg = 190);
    EXPECT_BAD((c.left_deg = 90, c.right_deg = 90));
    EXPECT_BAD((c.left_deg = 140, c.right_deg = 40, c.coarse_points = 7)); // 100 % 6 != 0
    EXPECT_BAD(c.fine_step_deg = 0);
    EXPECT_BAD(c.fine_half_range_deg = 2);
    EXPECT_BAD(c.fine_half_range_deg = 95);
    EXPECT_BAD(c.detect_cm = 0.0f);
    EXPECT_BAD(c.detect_cm = NAN);
    EXPECT_BAD(c.max_range_cm = 10.0f);
    EXPECT_BAD(c.detect_confirm_count = 0);
    EXPECT_BAD(c.min_clearance_cm = 0.0f);
    EXPECT_BAD(c.emergency_stop_cm = -1.0f);
    EXPECT_BAD(c.max_rescans = -1);
    EXPECT_BAD(c.reverse_cm = 0.0f);
    EXPECT_BAD(c.motion_timeout_ms = 0);
    EXPECT_BAD(c.max_reacquire_attempts = 0);
    EXPECT_BAD(c.max_consecutive_faults = 0);
#undef EXPECT_BAD
    CHECK(av_config_check(NULL) != NULL);
}

static void init_rejects_incomplete_platform(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    av_platform_t p = platform_for(&s);
    p.line_detected = NULL;
    CHECK(!av_init(&av, &s.cfg, &p));
    CHECK_INT(av_state(&av), AV_STATE_FAILED);
    av_tick(&av);  // must not crash
    av_reset(&av); // must not crash or revive
    CHECK_INT(av_state(&av), AV_STATE_FAILED);

    p = platform_for(&s);
    p.on_event = NULL; // optional
    CHECK(av_init(&av, &s.cfg, &p));
}

// ---------------------------------------------------------------------------
// Detection
// ---------------------------------------------------------------------------

static void no_obstacle_stays_passive(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    run_ms(&s, &av, 10000);
    CHECK_INT(av_state(&av), AV_STATE_MONITOR);
    CHECK(!av_is_active(&av));
    CHECK_INT(s.stops, 0);
    CHECK_INT(av_metrics(&av)->detections, 0);
    CHECK_INT(s.servo, 90);
    range_reading_t r;
    CHECK(av_last_reading(&av, &r));
    CHECK_NEAR(r.cm, 150.0f, 1e-6);
    check_hygiene(&s);
}

static void single_spike_is_debounced(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    run_ms(&s, &av, 500);
    s.spikes = 1;
    run_ms(&s, &av, 3000);
    CHECK_INT(av_metrics(&av)->detections, 0);
    CHECK_INT(s.stops, 0);

    s.spikes = 2; // two in a row is a real detection
    run_ms(&s, &av, 500);
    CHECK_INT(av_metrics(&av)->detections, 1);
}

static void too_close_counts_as_obstacle(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.force = true;
    s.forced_status = RANGE_TOO_CLOSE;
    CHECK(run_until(&s, &av, AV_STATE_COARSE_SCAN, 3000));
    CHECK_INT(s.stops, 1);
}

// ---------------------------------------------------------------------------
// Full encounters
// ---------------------------------------------------------------------------

static void centered_obstacle_full_encounter(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    CHECK(run_encounter(&s, &av));

    const av_metrics_t *m = av_metrics(&av);
    const obstacle_profile_t *p = av_profile(&av);
    CHECK(p != NULL);
    if (p) {
        CHECK(p->left.found && p->right.found);
        CHECK_NEAR(p->left.angle_deg, 102.5f, 1e-4);
        CHECK_NEAR(p->right.angle_deg, 77.5f, 1e-4);
        CHECK_NEAR(p->distance_cm, 12.0f, 1e-4);
        // Sim obstacle spans 80..100 deg at 12 cm => ~4.2 cm; step quantisation adds ~1 cm.
        CHECK(p->width_cm > 3.5f && p->width_cm < 6.5f);
        CHECK_NEAR(p->center.x_cm, 0.0f, 1e-3);
    }
    CHECK_INT(m->detections, 1);
    CHECK_INT(m->profiles_complete, 1);
    CHECK_INT(s.bypasses, 1);
    CHECK_INT(s.last_bypass, SIDE_LEFT); // exact tie -> LEFT
    CHECK_INT(s.searches, 1);
    CHECK_INT(s.search_sides[0], SIDE_RIGHT); // line is back where we came from
    CHECK_INT(s.resumes, 1);
    CHECK_INT(m->reacquire_ok, 1);
    CHECK(m->last_encounter_ms > 0);
    CHECK_INT(m->total_encounter_ms, m->last_encounter_ms);
    CHECK(m->last_scan_ms > 0 && m->last_scan_ms < m->last_encounter_ms);
    // Rubric: "time to bypass" < 3 s for an A. Scanning is the part we own.
    CHECK(m->last_scan_ms < 2500);
    printf("    (sim timing: scan %lu ms, full encounter %lu ms)\n",
           (unsigned long)m->last_scan_ms, (unsigned long)m->last_encounter_ms);
    CHECK_INT(m->failures, 0);
    CHECK(!av_is_active(&av));

    // Events arrive in pipeline order.
    int d = first_event(&s, AV_EVT_OBSTACLE_DETECTED), sp = first_event(&s, AV_EVT_SCAN_POINT),
        pr = first_event(&s, AV_EVT_PROFILE_READY), de = first_event(&s, AV_EVT_DECISION),
        by = first_event(&s, AV_EVT_BYPASS_START), rs = first_event(&s, AV_EVT_REACQUIRE_START),
        ok = first_event(&s, AV_EVT_LINE_REACQUIRED);
    CHECK(d == 0 && d < sp && sp < pr && pr < de && de < by && by < rs && rs < ok);
    CHECK_INT(count_events(&s, AV_EVT_SCAN_POINT), 5 + 3 + 3); // coarse + 3 fine each side
    check_hygiene(&s);
}

static void offset_obstacle_goes_short_way(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){55, 95, 12.0f, true}; // mostly on the right
    CHECK(run_encounter(&s, &av));
    CHECK_INT(s.last_bypass, SIDE_LEFT);
    CHECK(av_profile(&av) && av_profile(&av)->center.x_cm > 0.0f);

    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){85, 125, 12.0f, true}; // mostly on the left
    CHECK(run_encounter(&s, &av));
    CHECK_INT(s.last_bypass, SIDE_RIGHT);
    CHECK_INT(s.search_sides[0], SIDE_LEFT);
    check_hygiene(&s);
}

static void wall_on_one_side_forces_other(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.wall = (blob_t){115, 180, 15.0f, true}; // left side walled off
    CHECK(run_encounter(&s, &av));
    CHECK_INT(s.last_bypass, SIDE_RIGHT);
    check_hygiene(&s);
}

static void wide_obstacle_gives_partial_profile(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){40, 140, 12.0f, true}; // wider than the +/-45 deg fine scan
    CHECK(run_encounter(&s, &av));
    const obstacle_profile_t *p = av_profile(&av);
    CHECK(p != NULL);
    if (p) {
        CHECK(!p->left.found && !p->right.found);
        CHECK(p->width_cm >= 16.9f); // lower bound: 2 * 12 * sin(45)
    }
    CHECK_INT(av_metrics(&av)->profiles_partial, 1);
    CHECK_INT(s.bypasses, 1);
    check_hygiene(&s);
}

static void flipped_servo_mounting(void) {
    sim_t s;
    av_t av;
    memset(&s, 0, sizeof s);
    av_config_default(&s.cfg);
    s.cfg.left_deg = 30; // servo mounted the other way round
    s.cfg.right_deg = 150;
    s.min_angle = 999;
    s.max_angle = -999;
    s.bg_cm = 150.0f;
    s.bypass_ms = 1500;
    s.line_after_ms = 300;
    av_platform_t p = platform_for(&s);
    CHECK(av_init(&av, &s.cfg, &p));
    s.obstacle = (blob_t){85, 125, 12.0f, true}; // now mostly on the RIGHT
    CHECK(run_encounter(&s, &av));
    CHECK_INT(s.last_bypass, SIDE_LEFT);
    CHECK_INT(s.search_sides[0], SIDE_RIGHT);
    check_hygiene(&s);
}

static void back_to_back_encounters_accumulate_metrics(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    CHECK(run_encounter(&s, &av));
    uint32_t first = av_metrics(&av)->last_encounter_ms;
    CHECK(run_encounter(&s, &av)); // obstacle still there: second encounter
    const av_metrics_t *m = av_metrics(&av);
    CHECK_INT(m->detections, 2);
    CHECK_INT(m->reacquire_ok, 2);
    CHECK_INT(m->total_encounter_ms, (uint64_t)first + m->last_encounter_ms);
    CHECK(m->max_encounter_ms >= first && m->max_encounter_ms >= m->last_encounter_ms);
}

// ---------------------------------------------------------------------------
// Recovery paths
// ---------------------------------------------------------------------------

static void obstacle_gone_on_rescan_resumes(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.vanish_on_stop = true;
    CHECK(run_encounter(&s, &av));
    CHECK_INT(av_metrics(&av)->obstacle_cleared, 1);
    CHECK_INT(s.bypasses, 0);
    CHECK_INT(s.searches, 0);
    CHECK_INT(s.resumes, 1);
    CHECK_INT(count_events(&s, AV_EVT_OBSTACLE_CLEARED), 1);
}

static void boxed_in_rescans_reverses_then_fails(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.bg_cm = 10.0f; // walls everywhere
    CHECK(run_until(&s, &av, AV_STATE_FAILED, 120000));
    const av_metrics_t *m = av_metrics(&av);
    CHECK_INT(m->rescans, 6); // 2 per cycle x (1 + 2 reverses)
    CHECK_INT(m->reverses, 2);
    CHECK_INT(s.reverses, 2);
    CHECK_INT(m->failures, 1);
    CHECK_INT(s.bypasses, 0);
    CHECK(av_fail_reason(&av) != NULL && strstr(av_fail_reason(&av), "no clear path"));
    CHECK(!s.motion_active); // stopped on failure

    // Stays failed and passive.
    int stops = s.stops;
    run_ms(&s, &av, 5000);
    CHECK_INT(av_state(&av), AV_STATE_FAILED);
    CHECK_INT(s.stops, stops);
    check_hygiene(&s);
}

static void reverse_opens_a_path(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.bg_cm = 10.0f;
    s.open_on_reverse = true;
    CHECK(run_encounter(&s, &av));
    CHECK_INT(av_metrics(&av)->reverses, 1);
    CHECK_INT(s.bypasses, 1);
    CHECK_INT(av_metrics(&av)->reacquire_ok, 1);
}

static void bypass_timeout_fails_safe(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.bypass_never_done = true;
    CHECK(run_until(&s, &av, AV_STATE_FAILED, 60000));
    CHECK_INT(av_metrics(&av)->motion_timeouts, 1);
    CHECK(strstr(av_fail_reason(&av), "bypass timed out") != NULL);
    CHECK(!s.motion_active);
}

static void reacquire_retries_alternate_direction(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.line_on_search = 2;
    CHECK(run_encounter(&s, &av));
    CHECK_INT(s.searches, 2);
    CHECK_INT(s.search_sides[0], SIDE_RIGHT); // toward the line first
    CHECK_INT(s.search_sides[1], SIDE_LEFT);  // then the other way
    CHECK_INT(av_metrics(&av)->reacquire_retries, 1);
    CHECK_INT(av_metrics(&av)->reacquire_ok, 1);
}

static void reacquire_gives_up(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.line_never = true;
    CHECK(run_until(&s, &av, AV_STATE_FAILED, 60000));
    CHECK_INT(s.searches, s.cfg.max_reacquire_attempts);
    CHECK_INT(av_metrics(&av)->reacquire_fail, 1);
    CHECK(strstr(av_fail_reason(&av), "line not reacquired") != NULL);
}

static void emergency_stop_replans_and_rejoins(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    s.late_obstacle = (blob_t){85, 95, 4.0f, false}; // appears once bypass starts
    s.vanish_late_on_estop = true;
    CHECK(run_until(&s, &av, AV_STATE_BYPASS, 10000));
    s.obstacle.present = false; // we've moved past the original obstacle
    CHECK(run_until(&s, &av, AV_STATE_MONITOR, 30000));

    const av_metrics_t *m = av_metrics(&av);
    CHECK_INT(m->emergency_stops, 1);
    CHECK_INT(count_events(&s, AV_EVT_EMERGENCY_STOP), 1);
    // Off the line when the path cleared: must search, not blindly resume.
    CHECK_INT(m->obstacle_cleared, 1);
    CHECK_INT(s.searches, 1);
    CHECK_INT(s.resumes, 1);
    CHECK_INT(m->reacquire_ok, 1);
    check_hygiene(&s);
}

static void persistent_sensor_fault_fails(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.force = true;
    s.forced_status = RANGE_FAULT;
    CHECK(run_until(&s, &av, AV_STATE_FAILED, 10000));
    CHECK_INT(av_metrics(&av)->sensor_faults, s.cfg.max_consecutive_faults);
    CHECK(strstr(av_fail_reason(&av), "sensor fault") != NULL);
}

static void intermittent_fault_does_not_fail(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    for (int i = 0; i < 20; i++) {
        s.force = (i % 2 == 0);
        s.forced_status = RANGE_FAULT;
        run_ms(&s, &av, 200);
    }
    CHECK(av_state(&av) != AV_STATE_FAILED);
}

static void reset_recovers_from_failure(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, 0);
    s.force = true;
    s.forced_status = RANGE_FAULT;
    CHECK(run_until(&s, &av, AV_STATE_FAILED, 10000));
    s.force = false;
    int stops = s.stops;
    av_reset(&av);
    CHECK_INT(av_state(&av), AV_STATE_MONITOR);
    CHECK_INT(s.stops, stops + 1);
    CHECK(av_fail_reason(&av) == NULL);
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    CHECK(run_encounter(&s, &av));
    CHECK_INT(av_metrics(&av)->reacquire_ok, 1);
    CHECK_INT(av_metrics(&av)->failures, 1); // metrics survive reset
}

static void survives_clock_wraparound(void) {
    sim_t s;
    av_t av;
    sim_setup(&s, &av, UINT32_MAX - 1500); // wraps mid-encounter
    s.obstacle = (blob_t){80, 100, 12.0f, true};
    CHECK(run_encounter(&s, &av));
    CHECK_INT(av_metrics(&av)->reacquire_ok, 1);
    CHECK(av_metrics(&av)->last_encounter_ms < 20000);
    check_hygiene(&s);
}

void run_avoidance_tests(void) {
    RUN(default_config_is_valid);
    RUN(invalid_configs_rejected);
    RUN(init_rejects_incomplete_platform);
    RUN(no_obstacle_stays_passive);
    RUN(single_spike_is_debounced);
    RUN(too_close_counts_as_obstacle);
    RUN(centered_obstacle_full_encounter);
    RUN(offset_obstacle_goes_short_way);
    RUN(wall_on_one_side_forces_other);
    RUN(wide_obstacle_gives_partial_profile);
    RUN(flipped_servo_mounting);
    RUN(back_to_back_encounters_accumulate_metrics);
    RUN(obstacle_gone_on_rescan_resumes);
    RUN(boxed_in_rescans_reverses_then_fails);
    RUN(reverse_opens_a_path);
    RUN(bypass_timeout_fails_safe);
    RUN(reacquire_retries_alternate_direction);
    RUN(reacquire_gives_up);
    RUN(emergency_stop_replans_and_rejoins);
    RUN(persistent_sensor_fault_fails);
    RUN(intermittent_fault_does_not_fail);
    RUN(reset_recovers_from_failure);
    RUN(survives_clock_wraparound);
}
