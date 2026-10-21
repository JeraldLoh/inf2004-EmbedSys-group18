// Buddy 5: adaptive ultrasonic scanning, obstacle profiling and avoidance.
//
// Bench-test firmware (Pico SDK). The avoidance logic is complete;
// Buddy 2 (motion) and Buddy 3 (line sensors) are not integrated yet, so the
// motion_* / line_* callbacks below simulate them with fixed durations.
// Integration = replace those callback bodies; nothing else changes.
// The avoidance logic itself is app_program/subsystem5 (shared with the RTOS build).

#include <stdio.h>
#include <stdlib.h>

#include "hardware/watchdog.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"

#include "avoidance.h"
#include "servo.h"
#include "ultrasonic.h"

#define TRIG_PIN 6
#define ECHO_PIN 7
#define SERVO_PIN 12

#define SERVO_MIN_US 500
#define SERVO_MAX_US 2500
#define RANGE_SAMPLES 3
#define RANGE_SAMPLE_SPACING_MS 30

#define WATCHDOG_TIMEOUT_MS 3000
#define USB_WAIT_MS 5000
#define HEARTBEAT_MS 1000

// Simulated Buddy 2/3 behaviour -- remove when integrating.
#define SIM_REVERSE_MS 600
#define SIM_BYPASS_MS 1500
#define SIM_LINE_FOUND_MS 400

typedef struct {
    ultrasonic_t sonar;
    servo_t servo;
    uint32_t motion_until;
    uint32_t search_started;
    bool searching;
} board_t;

static board_t g_board;
static av_t g_av;

static uint32_t now_ms(void) { return to_ms_since_boot(get_absolute_time()); }

// ---------------------------------------------------------------------------
// Platform callbacks
// ---------------------------------------------------------------------------

static uint32_t cb_now_ms(void *ctx) {
    (void)ctx;
    return now_ms();
}

static void cb_servo_set(void *ctx, int angle_deg) {
    servo_set_angle(&((board_t *)ctx)->servo, angle_deg);
}

static range_reading_t cb_measure(void *ctx) {
    return ultrasonic_read(&((board_t *)ctx)->sonar, RANGE_SAMPLES, RANGE_SAMPLE_SPACING_MS);
}

// --- Buddy 2 integration points (simulated) --------------------------------

static void cb_motion_stop(void *ctx) {
    board_t *b = ctx;
    b->motion_until = 0;
    b->searching = false;
    printf("[motion] STOP\n");
}

static void cb_motion_reverse(void *ctx, float cm) {
    board_t *b = ctx;
    b->motion_until = now_ms() + SIM_REVERSE_MS;
    printf("[motion] REVERSE %.1f cm\n", (double)cm);
}

static void cb_motion_bypass(void *ctx, side_t side, const obstacle_profile_t *p) {
    board_t *b = ctx;
    b->motion_until = now_ms() + SIM_BYPASS_MS;
    printf("[motion] BYPASS %s around %.1f cm wide obstacle at %.1f cm\n", side_name(side),
           (double)p->width_cm, (double)p->distance_cm);
}

static void cb_motion_search_line(void *ctx, side_t toward) {
    board_t *b = ctx;
    b->searching = true;
    b->search_started = now_ms();
    printf("[motion] SEARCH for line toward %s\n", side_name(toward));
}

static void cb_motion_resume(void *ctx) {
    board_t *b = ctx;
    b->searching = false;
    printf("[motion] RESUME line following\n");
}

static bool cb_motion_done(void *ctx) {
    board_t *b = ctx;
    return b->motion_until == 0 || (int32_t)(now_ms() - b->motion_until) >= 0;
}

// --- Buddy 3 integration point (simulated) ---------------------------------

static bool cb_line_detected(void *ctx) {
    board_t *b = ctx;
    return b->searching && now_ms() - b->search_started >= SIM_LINE_FOUND_MS;
}

// --- Logging / telemetry (Buddy 1 would publish these) ---------------------

static void print_reading(range_reading_t r) {
    if (r.status == RANGE_OK) {
        printf("%.1f cm", (double)r.cm);
    } else {
        printf("%s", range_status_name(r.status));
    }
}

static void print_metrics(const av_metrics_t *m) {
    uint32_t done = m->reacquire_ok;
    printf("[metrics] detections=%lu cleared=%lu bypass L/R=%lu/%lu rescans=%lu reverses=%lu "
           "estops=%lu reacq ok/retry/fail=%lu/%lu/%lu timeouts=%lu faults=%lu failures=%lu\n",
           (unsigned long)m->detections, (unsigned long)m->obstacle_cleared,
           (unsigned long)m->bypass_left, (unsigned long)m->bypass_right,
           (unsigned long)m->rescans, (unsigned long)m->reverses,
           (unsigned long)m->emergency_stops, (unsigned long)m->reacquire_ok,
           (unsigned long)m->reacquire_retries, (unsigned long)m->reacquire_fail,
           (unsigned long)m->motion_timeouts, (unsigned long)m->sensor_faults,
           (unsigned long)m->failures);
    printf("[metrics] last scan=%lu ms, last encounter=%lu ms, max=%lu ms, avg=%lu ms\n",
           (unsigned long)m->last_scan_ms, (unsigned long)m->last_encounter_ms,
           (unsigned long)m->max_encounter_ms,
           (unsigned long)(done ? m->total_encounter_ms / done : 0));
}

static void cb_on_event(void *ctx, const av_event_t *e) {
    (void)ctx;
    printf("%8lu ", (unsigned long)e->time_ms);
    switch (e->type) {
        case AV_EVT_OBSTACLE_DETECTED:
            printf("[detect] obstacle ahead: ");
            print_reading(e->point.reading);
            printf("\n");
            break;
        case AV_EVT_SCAN_POINT:
            printf("[scan] %3d deg: ", e->point.angle_deg);
            print_reading(e->point.reading);
            printf("\n");
            break;
        case AV_EVT_OBSTACLE_CLEARED:
            printf("[scan] %s\n", e->reason);
            break;
        case AV_EVT_PROFILE_READY: {
            const obstacle_profile_t *p = e->profile;
            printf("[profile] dist=%.1f cm width=%s%.1f cm center x=%+.1f y=%.1f cm "
                   "edges L=%.1f%s R=%.1f%s deg\n",
                   (double)p->distance_cm, (p->left.found && p->right.found) ? "" : ">=",
                   (double)p->width_cm, (double)p->center.x_cm, (double)p->center.y_cm,
                   (double)p->left.angle_deg, p->left.found ? "" : "(open)",
                   (double)p->right.angle_deg, p->right.found ? "" : "(open)");
            break;
        }
        case AV_EVT_DECISION:
            printf("[decide] clearance L=%.1f R=%.1f cm -> %s\n",
                   (double)e->choice->left_clear_cm, (double)e->choice->right_clear_cm,
                   side_name(e->side));
            break;
        case AV_EVT_RESCAN:
            printf("[decide] no clear side, rescan #%d\n", e->attempt);
            break;
        case AV_EVT_REVERSING:
            printf("[decide] still blocked, reversing (#%d)\n", e->attempt);
            break;
        case AV_EVT_BYPASS_START:
            printf("[bypass] going %s\n", side_name(e->side));
            break;
        case AV_EVT_EMERGENCY_STOP:
            printf("[guard] EMERGENCY STOP, object at ");
            print_reading(e->point.reading);
            printf("\n");
            break;
        case AV_EVT_REACQUIRE_START:
            printf("[recover] searching for line toward %s (attempt %d)\n", side_name(e->side),
                   e->attempt);
            break;
        case AV_EVT_LINE_REACQUIRED:
            printf("[recover] line reacquired, encounter took %lu ms\n",
                   (unsigned long)e->duration_ms);
            print_metrics(av_metrics(&g_av));
            break;
        case AV_EVT_FAILED:
            printf("[FAILED] %s -- robot stopped, needs reset\n", e->reason);
            print_metrics(av_metrics(&g_av));
            break;
    }
}

// ---------------------------------------------------------------------------
// Servo calibration mode: step the servo by hand to find the safe range.
// ---------------------------------------------------------------------------

// Set to 1 by the separate calibration build target (see CMakeLists.txt).
#ifndef BUDDY5_SERVO_CALIBRATION
#define BUDDY5_SERVO_CALIBRATION 0
#endif

#define CAL_SAFETY_MARGIN_DEG 5

#if BUDDY5_SERVO_CALIBRATION
static void cal_help(void) {
    printf("\n--- Servo calibration ---\n"
           "  a / d : step -5 / +5 deg      A / D : step -1 / +1 deg\n"
           "  c     : back to 90 deg        m     : measure distance here\n"
           "  l     : mark current angle as LEFT limit (sensor looking left)\n"
           "  r     : mark current angle as RIGHT limit (sensor looking right)\n"
           "  p     : print suggested config  h : help   q : quit to normal run\n"
           "Step slowly. Stop as soon as the servo touches anything, buzzes or\n"
           "strains, step back until it moves freely, then mark that angle.\n\n");
}

static void cal_move(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;
    servo_set_angle(&g_board.servo, angle);
    printf("servo -> %3d deg\n", angle);
}

// Builds a symmetric config inside the marked limits and checks it is valid.
static void cal_suggest(int left, int right, bool have_left, bool have_right) {
    if (!have_left || !have_right) {
        printf("mark both limits first (l and r)\n");
        return;
    }
    if (left == right) {
        printf("left and right limits are the same angle; re-mark them\n");
        return;
    }
    const int center = 90;
    int left_half = abs(left - center);
    int right_half = abs(right - center);
    bool left_is_higher = left > right;
    if ((left > center) != left_is_higher || (right > center) == left_is_higher) {
        printf("both limits must be on opposite sides of 90 deg; re-mark them\n");
        return;
    }

    // Use the smaller side (minus a margin) so both sides are equal, as the scan assumes.
    int half = (left_half < right_half ? left_half : right_half) - CAL_SAFETY_MARGIN_DEG;
    half -= half % 2; // span must split into 4 equal coarse steps
    if (half < 10) {
        printf("usable range is only +/-%d deg: too narrow to scan. Clear the obstruction.\n",
               half < 0 ? 0 : half);
        return;
    }

    av_config_t cfg;
    av_config_default(&cfg);
    cfg.center_deg = center;
    cfg.left_deg = left_is_higher ? center + half : center - half;
    cfg.right_deg = left_is_higher ? center - half : center + half;
    cfg.fine_half_range_deg = half - half % cfg.fine_step_deg;
    if (cfg.fine_half_range_deg < cfg.fine_step_deg) cfg.fine_half_range_deg = cfg.fine_step_deg;

    const char *err = av_config_check(&cfg);
    printf("\nMeasured safe limits: LEFT %d deg, RIGHT %d deg\n", left, right);
    printf("Usable symmetric range: +/-%d deg (includes %d deg safety margin)\n", half,
           CAL_SAFETY_MARGIN_DEG);
    if (err != NULL) {
        printf("Could not build a valid config: %s\n", err);
        return;
    }
    printf("Paste into av_config_default() in buddy5/src/avoidance.c:\n"
           "    cfg->left_deg = %d;\n"
           "    cfg->center_deg = %d;\n"
           "    cfg->right_deg = %d;\n"
           "    cfg->fine_half_range_deg = %d;\n\n",
           cfg.left_deg, cfg.center_deg, cfg.right_deg, cfg.fine_half_range_deg);
}

static void run_servo_calibration(void) {
    int angle = 90;
    int left = 0, right = 0;
    bool have_left = false, have_right = false;

    cal_help();
    cal_move(angle);

    while (true) {
        int ch = getchar_timeout_us(100000);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        switch (ch) {
            case 'a': angle -= 5; cal_move(angle); break;
            case 'd': angle += 5; cal_move(angle); break;
            case 'A': angle -= 1; cal_move(angle); break;
            case 'D': angle += 1; cal_move(angle); break;
            case 'c': angle = 90; cal_move(angle); break;
            case 'm': {
                range_reading_t r = ultrasonic_read(&g_board.sonar, RANGE_SAMPLES,
                                                    RANGE_SAMPLE_SPACING_MS);
                printf("at %3d deg: ", angle);
                print_reading(r);
                printf("\n");
                break;
            }
            case 'l':
                left = angle;
                have_left = true;
                printf("LEFT limit marked at %d deg\n", left);
                break;
            case 'r':
                right = angle;
                have_right = true;
                printf("RIGHT limit marked at %d deg\n", right);
                break;
            case 'p': cal_suggest(left, right, have_left, have_right); break;
            case 'h': cal_help(); break;
            case 'q':
                cal_move(90);
                printf("servo parked at 90 deg. Flash the normal build to run the car.\n");
                return;
            default: break;
        }
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
    }
}
#endif

// ---------------------------------------------------------------------------

int main(void) {
    stdio_init_all();
    for (uint32_t waited = 0; !stdio_usb_connected() && waited < USB_WAIT_MS; waited += 100) {
        sleep_ms(100);
    }
    printf("\nBuddy 5: adaptive ultrasonic scanning + obstacle profiling\n");
    if (watchdog_caused_reboot()) {
        printf("WARNING: previous run was reset by the watchdog\n");
    }

    ultrasonic_init(&g_board.sonar, TRIG_PIN, ECHO_PIN);
    servo_init(&g_board.servo, SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

#if BUDDY5_SERVO_CALIBRATION
    // Calibration build: no watchdog and no avoidance run, only manual servo stepping.
    run_servo_calibration();
    while (true) tight_loop_contents();
#endif

    av_config_t cfg;
    av_config_default(&cfg);
    const char *err = av_config_check(&cfg);
    if (err != NULL) {
        printf("FATAL: bad config: %s\n", err);
        while (true) tight_loop_contents();
    }

    av_platform_t hw = {
        .ctx = &g_board,
        .now_ms = cb_now_ms,
        .servo_set = cb_servo_set,
        .measure = cb_measure,
        .motion_stop = cb_motion_stop,
        .motion_reverse = cb_motion_reverse,
        .motion_bypass = cb_motion_bypass,
        .motion_search_line = cb_motion_search_line,
        .motion_resume_line_follow = cb_motion_resume,
        .motion_done = cb_motion_done,
        .line_detected = cb_line_detected,
        .on_event = cb_on_event,
    };
    if (!av_init(&g_av, &cfg, &hw)) {
        printf("FATAL: avoidance init failed\n");
        while (true) tight_loop_contents();
    }
    printf("ready: detect < %.0f cm, servo L/C/R = %d/%d/%d deg\n", (double)cfg.detect_cm,
           cfg.left_deg, cfg.center_deg, cfg.right_deg);

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    uint32_t next_heartbeat = now_ms();

    while (true) {
        watchdog_update();
        av_tick(&g_av);

        uint32_t now = now_ms();
        if (av_state(&g_av) == AV_STATE_MONITOR && (int32_t)(now - next_heartbeat) >= 0) {
            next_heartbeat = now + HEARTBEAT_MS;
            range_reading_t r;
            if (av_last_reading(&g_av, &r)) {
                printf("%8lu [monitor] ahead: ", (unsigned long)now);
                print_reading(r);
                printf("\n");
            }
        }
    }
}
