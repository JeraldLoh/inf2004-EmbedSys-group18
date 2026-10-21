#include "avoidance.h"

#include <stdlib.h>
#include <string.h>

// ---------------------------------------------------------------------------
// Configuration
// ---------------------------------------------------------------------------

void av_config_default(av_config_t *cfg) {
    memset(cfg, 0, sizeof *cfg);
    cfg->left_deg = 150;
    cfg->center_deg = 90;
    cfg->right_deg = 30;
    cfg->coarse_points = 5;
    cfg->fine_step_deg = 5;
    cfg->fine_half_range_deg = 45;

    cfg->detect_cm = 15.0f;
    cfg->clear_hysteresis_cm = 5.0f;
    cfg->detect_confirm_count = 2;
    cfg->edge_margin_cm = 8.0f;
    cfg->min_clearance_cm = 25.0f;
    cfg->max_range_cm = 400.0f;
    cfg->emergency_stop_cm = 6.0f;

    cfg->servo_settle_base_ms = 40;
    cfg->servo_ms_per_deg = 2.0f; // SG90: ~0.1 s / 60 deg, plus margin

    cfg->max_rescans = 2;
    cfg->rescan_delay_ms = 500;
    cfg->max_reverses = 2;
    cfg->reverse_cm = 10.0f;

    cfg->motion_timeout_ms = 8000;
    cfg->reacquire_timeout_ms = 3000;
    cfg->max_reacquire_attempts = 3;
    cfg->max_consecutive_faults = 5;
}

static bool angle_ok(int deg) { return deg >= 0 && deg <= 180; }

const char *av_config_check(const av_config_t *c) {
    if (c == NULL) return "config is NULL";
    if (!angle_ok(c->left_deg) || !angle_ok(c->right_deg) || !angle_ok(c->center_deg))
        return "servo angles must be within 0..180";
    if (c->left_deg == c->right_deg) return "left_deg and right_deg must differ";
    if (c->left_deg + c->right_deg != 2 * c->center_deg)
        return "center_deg must be exactly halfway between left_deg and right_deg";
    if (c->coarse_points < 3 || c->coarse_points > AV_MAX_COARSE_POINTS || c->coarse_points % 2 == 0)
        return "coarse_points must be odd and within 3..AV_MAX_COARSE_POINTS";
    if (abs(c->left_deg - c->right_deg) % (c->coarse_points - 1) != 0)
        return "coarse_points must evenly divide the left..right span";
    if (c->fine_step_deg <= 0) return "fine_step_deg must be > 0";
    if (c->fine_half_range_deg < c->fine_step_deg)
        return "fine_half_range_deg must be >= fine_step_deg";
    if (!angle_ok(c->center_deg - c->fine_half_range_deg) ||
        !angle_ok(c->center_deg + c->fine_half_range_deg))
        return "fine scan range exceeds servo limits";
    if (!(c->detect_cm > 0.0f)) return "detect_cm must be > 0";
    if (!(c->max_range_cm > c->detect_cm)) return "max_range_cm must exceed detect_cm";
    if (!(c->clear_hysteresis_cm >= 0.0f)) return "clear_hysteresis_cm must be >= 0";
    if (c->detect_confirm_count < 1) return "detect_confirm_count must be >= 1";
    if (!(c->edge_margin_cm >= 0.0f)) return "edge_margin_cm must be >= 0";
    if (!(c->min_clearance_cm > 0.0f)) return "min_clearance_cm must be > 0";
    if (!(c->emergency_stop_cm >= 0.0f)) return "emergency_stop_cm must be >= 0";
    if (!(c->servo_ms_per_deg >= 0.0f)) return "servo_ms_per_deg must be >= 0";
    if (c->max_rescans < 0 || c->max_reverses < 0) return "retry limits must be >= 0";
    if (c->max_reverses > 0 && !(c->reverse_cm > 0.0f)) return "reverse_cm must be > 0";
    if (c->motion_timeout_ms == 0 || c->reacquire_timeout_ms == 0) return "timeouts must be > 0";
    if (c->max_reacquire_attempts < 1) return "max_reacquire_attempts must be >= 1";
    if (c->max_consecutive_faults < 1) return "max_consecutive_faults must be >= 1";
    return NULL;
}

static bool platform_ok(const av_platform_t *p) {
    return p != NULL && p->now_ms && p->servo_set && p->measure && p->motion_stop &&
           p->motion_reverse && p->motion_bypass && p->motion_search_line &&
           p->motion_resume_line_follow && p->motion_done && p->line_detected;
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Wrap-safe: valid while deadlines are < ~24 days ahead.
static bool time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

static side_t opposite(side_t s) {
    return s == SIDE_LEFT ? SIDE_RIGHT : (s == SIDE_RIGHT ? SIDE_LEFT : SIDE_NONE);
}

static void emit(av_t *a, uint32_t now, av_event_t *e) {
    if (a->hw.on_event == NULL) return;
    e->time_ms = now;
    a->hw.on_event(a->hw.ctx, e);
}

static av_event_t event_of(av_event_type_t type) {
    av_event_t e;
    memset(&e, 0, sizeof e);
    e.type = type;
    return e;
}

static void enter(av_t *a, av_state_t s) { a->state = s; }

static void fail(av_t *a, uint32_t now, const char *reason) {
    a->hw.motion_stop(a->hw.ctx);
    a->metrics.failures++;
    a->fail_reason = reason;
    enter(a, AV_STATE_FAILED);
    av_event_t e = event_of(AV_EVT_FAILED);
    e.reason = reason;
    emit(a, now, &e);
}

static uint32_t settle_ms(const av_t *a, int delta_deg) {
    if (delta_deg == 0) return 0;
    return a->cfg.servo_settle_base_ms + (uint32_t)(a->cfg.servo_ms_per_deg * (float)delta_deg + 0.5f);
}

static bool is_blocked(range_reading_t r, float threshold_cm) {
    return r.status == RANGE_TOO_CLOSE || (r.status == RANGE_OK && r.cm < threshold_cm);
}

typedef enum { TAKE_PENDING, TAKE_READY, TAKE_ABORTED } take_result_t;

// Points the servo at angle and, once it has had time to settle, takes one
// reading. Never blocks on the servo: returns TAKE_PENDING until settled.
static take_result_t take(av_t *a, uint32_t now, int angle, range_reading_t *out) {
    if (!a->step_pending || a->pending_angle != angle) {
        int delta = a->servo_known ? abs(angle - a->servo_deg) : 180;
        if (delta != 0) {
            a->hw.servo_set(a->hw.ctx, angle);
            a->servo_deg = angle;
            a->servo_known = true;
            a->step_pending = true;
            a->pending_angle = angle;
            a->settle_until = now + settle_ms(a, delta);
            return TAKE_PENDING;
        }
    } else if (!time_reached(now, a->settle_until)) {
        return TAKE_PENDING;
    }

    a->step_pending = false;
    *out = a->hw.measure(a->hw.ctx);
    a->last_reading = *out;
    a->have_reading = true;

    if (out->status == RANGE_FAULT) {
        a->metrics.sensor_faults++;
        if (++a->fault_streak >= a->cfg.max_consecutive_faults) {
            fail(a, now, "ultrasonic sensor fault");
            return TAKE_ABORTED;
        }
    } else {
        a->fault_streak = 0;
    }
    return TAKE_READY;
}

static int coarse_angle(const av_t *a, int i) {
    return a->cfg.right_deg + i * (a->cfg.left_deg - a->cfg.right_deg) / (a->cfg.coarse_points - 1);
}

static int fine_angle(const av_t *a, side_t side, int offset) {
    int toward_left = a->geom.left_is_higher ? 1 : -1;
    return a->cfg.center_deg + offset * (side == SIDE_LEFT ? toward_left : -toward_left);
}

// Distance to assume for a point that hit the obstacle but has no usable cm.
static float hit_cm(const av_t *a, const scan_point_t *p) {
    return p->reading.status == RANGE_OK ? p->reading.cm : a->ahead_cm;
}

static obstacle_edge_t make_edge(const av_t *a, float angle_deg, float cm, bool found) {
    obstacle_edge_t e;
    e.angle_deg = angle_deg;
    e.pt = scan_polar_to_xy(&a->geom, angle_deg, cm);
    e.found = found;
    return e;
}

static void start_coarse(av_t *a) {
    a->coarse_idx = 0;
    enter(a, AV_STATE_COARSE_SCAN);
}

static void start_fine(av_t *a, av_state_t which) {
    a->fine_offset = a->cfg.fine_step_deg;
    a->last_hit = a->coarse[a->cfg.coarse_points / 2];
    enter(a, which);
}

static void start_reacquire(av_t *a, uint32_t now) {
    a->reacquire_attempts = 1;
    // The line lies back toward the side we left it from.
    side_t toward = opposite(a->choice.side);
    a->hw.motion_search_line(a->hw.ctx, toward);
    a->deadline = now + a->cfg.reacquire_timeout_ms;
    enter(a, AV_STATE_REACQUIRE);
    av_event_t e = event_of(AV_EVT_REACQUIRE_START);
    e.side = toward;
    e.attempt = a->reacquire_attempts;
    emit(a, now, &e);
}

static void begin_encounter(av_t *a, uint32_t now, range_reading_t r) {
    a->hw.motion_stop(a->hw.ctx);
    a->metrics.detections++;
    a->t_detect = now;
    a->rescans = 0;
    a->reverses = 0;
    a->reacquire_attempts = 0;
    a->off_line = false;
    a->choice.side = SIDE_NONE;
    av_event_t e = event_of(AV_EVT_OBSTACLE_DETECTED);
    e.point.angle_deg = a->cfg.center_deg;
    e.point.reading = r;
    emit(a, now, &e);
    start_coarse(a);
}

// Collision guard while manoeuvring. Returns true if it stopped the robot.
static bool guard_tripped(av_t *a, uint32_t now) {
    if (a->cfg.emergency_stop_cm <= 0.0f) return false;
    range_reading_t r;
    take_result_t t = take(a, now, a->cfg.center_deg, &r);
    if (t == TAKE_ABORTED) return true;
    if (t != TAKE_READY || !is_blocked(r, a->cfg.emergency_stop_cm)) return false;

    a->hw.motion_stop(a->hw.ctx);
    a->metrics.emergency_stops++;
    av_event_t e = event_of(AV_EVT_EMERGENCY_STOP);
    e.point.angle_deg = a->cfg.center_deg;
    e.point.reading = r;
    emit(a, now, &e);
    start_coarse(a); // re-plan; off_line is kept so a clear path leads to reacquire
    return true;
}

// ---------------------------------------------------------------------------
// State handlers
// ---------------------------------------------------------------------------

static void on_monitor(av_t *a, uint32_t now) {
    range_reading_t r;
    if (take(a, now, a->cfg.center_deg, &r) != TAKE_READY) return;
    if (!is_blocked(r, a->cfg.detect_cm)) {
        a->confirm_count = 0;
        return;
    }
    if (++a->confirm_count >= a->cfg.detect_confirm_count) {
        a->confirm_count = 0;
        begin_encounter(a, now, r);
    }
}

static void on_coarse(av_t *a, uint32_t now) {
    int angle = coarse_angle(a, a->coarse_idx);
    range_reading_t r;
    if (take(a, now, angle, &r) != TAKE_READY) return;

    scan_point_t p = {angle, r};
    a->coarse[a->coarse_idx++] = p;
    av_event_t e = event_of(AV_EVT_SCAN_POINT);
    e.point = p;
    emit(a, now, &e);
    if (a->coarse_idx < a->cfg.coarse_points) return;

    range_reading_t center = a->coarse[a->cfg.coarse_points / 2].reading;
    a->ahead_cm = range_effective_cm(center, a->cfg.max_range_cm);

    bool gone = center.status != RANGE_TOO_CLOSE && center.status != RANGE_FAULT &&
                a->ahead_cm >= a->cfg.detect_cm + a->cfg.clear_hysteresis_cm;
    if (!gone) {
        start_fine(a, AV_STATE_FINE_SCAN_RIGHT);
        return;
    }

    a->metrics.obstacle_cleared++;
    av_event_t c = event_of(AV_EVT_OBSTACLE_CLEARED);
    c.reason = a->off_line ? "path clear, rejoining line" : "obstacle no longer ahead";
    emit(a, now, &c);
    if (a->off_line) {
        start_reacquire(a, now);
    } else {
        a->hw.motion_resume_line_follow(a->hw.ctx);
        enter(a, AV_STATE_MONITOR);
    }
}

static void on_fine(av_t *a, uint32_t now) {
    side_t side = a->state == AV_STATE_FINE_SCAN_RIGHT ? SIDE_RIGHT : SIDE_LEFT;
    obstacle_edge_t edge = {{0.0f, 0.0f}, 0.0f, false};
    bool done = false;

    if (a->fine_offset > a->cfg.fine_half_range_deg) {
        edge = make_edge(a, (float)a->last_hit.angle_deg, hit_cm(a, &a->last_hit), false);
        done = true;
    } else {
        int angle = fine_angle(a, side, a->fine_offset);
        range_reading_t r;
        if (take(a, now, angle, &r) != TAKE_READY) return;

        scan_point_t p = {angle, r};
        av_event_t e = event_of(AV_EVT_SCAN_POINT);
        e.point = p;
        emit(a, now, &e);

        if (obstacle_is_clear(r, a->ahead_cm, a->cfg.edge_margin_cm, a->cfg.max_range_cm)) {
            // True edge lies somewhere between the last hit and this miss.
            float edge_deg = 0.5f * (float)(a->last_hit.angle_deg + angle);
            edge = make_edge(a, edge_deg, hit_cm(a, &a->last_hit), true);
            done = true;
        } else {
            a->last_hit = p;
            a->fine_offset += a->cfg.fine_step_deg;
        }
    }

    if (!done) return;
    if (side == SIDE_RIGHT) {
        a->right_edge = edge;
        start_fine(a, AV_STATE_FINE_SCAN_LEFT);
        return;
    }

    a->profile = obstacle_build_profile(a->ahead_cm, edge, a->right_edge);
    a->have_profile = true;
    enter(a, AV_STATE_DECIDE);
}

static void on_decide(av_t *a, uint32_t now) {
    if (a->profile.left.found && a->profile.right.found) {
        a->metrics.profiles_complete++;
    } else {
        a->metrics.profiles_partial++;
    }
    a->metrics.last_scan_ms = now - a->t_detect;

    av_event_t pe = event_of(AV_EVT_PROFILE_READY);
    pe.profile = &a->profile;
    emit(a, now, &pe);

    a->choice = obstacle_choose_side(&a->geom, a->coarse, (size_t)a->cfg.coarse_points,
                                     &a->profile, a->cfg.min_clearance_cm, a->cfg.max_range_cm);
    av_event_t de = event_of(AV_EVT_DECISION);
    de.side = a->choice.side;
    de.profile = &a->profile;
    de.choice = &a->choice;
    emit(a, now, &de);

    if (a->choice.side != SIDE_NONE) {
        if (a->choice.side == SIDE_LEFT) {
            a->metrics.bypass_left++;
        } else {
            a->metrics.bypass_right++;
        }
        a->off_line = true;
        a->hw.motion_bypass(a->hw.ctx, a->choice.side, &a->profile);
        a->deadline = now + a->cfg.motion_timeout_ms;
        enter(a, AV_STATE_BYPASS);
        av_event_t be = event_of(AV_EVT_BYPASS_START);
        be.side = a->choice.side;
        emit(a, now, &be);
        return;
    }

    if (a->rescans < a->cfg.max_rescans) {
        a->rescans++;
        a->metrics.rescans++;
        a->deadline = now + a->cfg.rescan_delay_ms;
        enter(a, AV_STATE_WAIT_RESCAN);
        av_event_t e = event_of(AV_EVT_RESCAN);
        e.attempt = a->rescans;
        emit(a, now, &e);
    } else if (a->reverses < a->cfg.max_reverses) {
        a->reverses++;
        a->rescans = 0;
        a->metrics.reverses++;
        a->hw.motion_reverse(a->hw.ctx, a->cfg.reverse_cm);
        a->deadline = now + a->cfg.motion_timeout_ms;
        enter(a, AV_STATE_REVERSE);
        av_event_t e = event_of(AV_EVT_REVERSING);
        e.attempt = a->reverses;
        emit(a, now, &e);
    } else {
        fail(a, now, "no clear path after reversing");
    }
}

static void on_wait_rescan(av_t *a, uint32_t now) {
    if (time_reached(now, a->deadline)) start_coarse(a);
}

static void on_reverse(av_t *a, uint32_t now) {
    if (a->hw.motion_done(a->hw.ctx)) {
        start_coarse(a);
    } else if (time_reached(now, a->deadline)) {
        a->metrics.motion_timeouts++;
        fail(a, now, "reverse timed out");
    }
}

static void on_bypass(av_t *a, uint32_t now) {
    if (a->hw.motion_done(a->hw.ctx)) {
        start_reacquire(a, now);
        return;
    }
    if (time_reached(now, a->deadline)) {
        a->metrics.motion_timeouts++;
        fail(a, now, "bypass timed out");
        return;
    }
    guard_tripped(a, now);
}

static void on_reacquire(av_t *a, uint32_t now) {
    if (a->hw.line_detected(a->hw.ctx)) {
        a->hw.motion_resume_line_follow(a->hw.ctx);
        a->off_line = false;
        uint32_t dur = now - a->t_detect;
        a->metrics.reacquire_ok++;
        a->metrics.last_encounter_ms = dur;
        if (dur > a->metrics.max_encounter_ms) a->metrics.max_encounter_ms = dur;
        a->metrics.total_encounter_ms += dur;
        a->confirm_count = 0;
        enter(a, AV_STATE_MONITOR);
        av_event_t e = event_of(AV_EVT_LINE_REACQUIRED);
        e.duration_ms = dur;
        emit(a, now, &e);
        return;
    }

    if (time_reached(now, a->deadline)) {
        if (a->reacquire_attempts >= a->cfg.max_reacquire_attempts) {
            a->metrics.reacquire_fail++;
            fail(a, now, "line not reacquired");
            return;
        }
        // Alternate search direction in case the first search overshot.
        a->reacquire_attempts++;
        a->metrics.reacquire_retries++;
        side_t toward = (a->reacquire_attempts % 2 == 1) ? opposite(a->choice.side) : a->choice.side;
        a->hw.motion_search_line(a->hw.ctx, toward);
        a->deadline = now + a->cfg.reacquire_timeout_ms;
        av_event_t e = event_of(AV_EVT_REACQUIRE_START);
        e.side = toward;
        e.attempt = a->reacquire_attempts;
        emit(a, now, &e);
        return;
    }

    guard_tripped(a, now);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool av_init(av_t *a, const av_config_t *cfg, const av_platform_t *hw) {
    if (a == NULL) return false;
    memset(a, 0, sizeof *a);
    a->state = AV_STATE_FAILED;
    if (av_config_check(cfg) != NULL || !platform_ok(hw)) {
        a->fail_reason = "invalid configuration";
        return false;
    }
    a->cfg = *cfg;
    a->hw = *hw;
    a->geom.center_deg = cfg->center_deg;
    a->geom.left_is_higher = cfg->left_deg > cfg->right_deg;
    a->state = AV_STATE_MONITOR;
    return true;
}

void av_tick(av_t *a) {
    if (a->state == AV_STATE_FAILED) return; // also covers a failed av_init
    uint32_t now = a->hw.now_ms(a->hw.ctx);
    switch (a->state) {
        case AV_STATE_MONITOR: on_monitor(a, now); break;
        case AV_STATE_COARSE_SCAN: on_coarse(a, now); break;
        case AV_STATE_FINE_SCAN_RIGHT:
        case AV_STATE_FINE_SCAN_LEFT: on_fine(a, now); break;
        case AV_STATE_DECIDE: on_decide(a, now); break;
        case AV_STATE_WAIT_RESCAN: on_wait_rescan(a, now); break;
        case AV_STATE_REVERSE: on_reverse(a, now); break;
        case AV_STATE_BYPASS: on_bypass(a, now); break;
        case AV_STATE_REACQUIRE: on_reacquire(a, now); break;
        case AV_STATE_FAILED:
        default: break;
    }
}

void av_reset(av_t *a) {
    if (a->hw.motion_stop == NULL) return; // never initialised
    a->hw.motion_stop(a->hw.ctx);
    a->state = AV_STATE_MONITOR;
    a->step_pending = false;
    a->confirm_count = 0;
    a->fault_streak = 0;
    a->off_line = false;
    a->rescans = 0;
    a->reverses = 0;
    a->reacquire_attempts = 0;
    a->fail_reason = NULL;
}

av_state_t av_state(const av_t *a) { return a->state; }

const char *av_state_name(av_state_t s) {
    static const char *const names[AV_STATE_COUNT] = {
        "MONITOR", "COARSE_SCAN", "FINE_SCAN_RIGHT", "FINE_SCAN_LEFT", "DECIDE",
        "WAIT_RESCAN", "REVERSE", "BYPASS", "REACQUIRE", "FAILED",
    };
    return ((unsigned)s < AV_STATE_COUNT) ? names[s] : "INVALID";
}

bool av_is_active(const av_t *a) { return a->state != AV_STATE_MONITOR; }

bool av_last_reading(const av_t *a, range_reading_t *out) {
    if (!a->have_reading) return false;
    *out = a->last_reading;
    return true;
}

const obstacle_profile_t *av_profile(const av_t *a) { return a->have_profile ? &a->profile : NULL; }
const av_metrics_t *av_metrics(const av_t *a) { return &a->metrics; }
const char *av_fail_reason(const av_t *a) { return a->fail_reason; }
