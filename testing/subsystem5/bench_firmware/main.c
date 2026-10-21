// Buddy 5: adaptive ultrasonic scanning, obstacle profiling and avoidance.
//
// Bench-test firmware (Pico SDK). The avoidance logic is complete;
// Buddy 2 (motion) and Buddy 3 (line sensors) are not integrated yet, so the
// motion_* / line_* callbacks below simulate them with fixed durations.
// Integration = replace those callback bodies; nothing else changes.
// The avoidance logic itself is app_program/subsystem5 (shared with the RTOS build).
//
// Boot sequence:
//   1. Calibration menu: recalibrate the servo range, or keep the saved one.
//      The calibration is stored in the last flash sector, so it survives
//      power-off and reflashing. With no key press (or no PC attached) the
//      saved calibration is used automatically.
//   2. Self-check: sweeps the servo across the scan angles and prints a
//      reading at each one, so wiring problems show up before the run.
//   3. Avoidance run: detection, scanning, decision and recovery, with every
//      step and state change printed.

#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/flash.h"
#include "hardware/watchdog.h"
#include "pico/flash.h"
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
#define FAILED_REMINDER_MS 5000
#define MENU_TIMEOUT_S 10
#define SELFTEST_SETTLE_MS 300

// Servo range measured on this car with the calibration menu. Used when no
// valid calibration is saved in flash.
#define CAR_LEFT_DEG 102
#define CAR_CENTER_DEG 90
#define CAR_RIGHT_DEG 78
#define CAR_FINE_HALF_RANGE_DEG 10

#define CAL_SAFETY_MARGIN_DEG 5

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
    printf("         [motion] STOP\n");
}

static void cb_motion_reverse(void *ctx, float cm) {
    board_t *b = ctx;
    b->motion_until = now_ms() + SIM_REVERSE_MS;
    printf("         [motion] REVERSE %.1f cm (simulated, %d ms)\n", (double)cm, SIM_REVERSE_MS);
}

static void cb_motion_bypass(void *ctx, side_t side, const obstacle_profile_t *p) {
    board_t *b = ctx;
    b->motion_until = now_ms() + SIM_BYPASS_MS;
    printf("         [motion] BYPASS %s around %.1f cm wide obstacle at %.1f cm (simulated, %d ms)\n",
           side_name(side), (double)p->width_cm, (double)p->distance_cm, SIM_BYPASS_MS);
}

static void cb_motion_search_line(void *ctx, side_t toward) {
    board_t *b = ctx;
    b->searching = true;
    b->search_started = now_ms();
    printf("         [motion] SEARCH for line toward %s (simulated)\n", side_name(toward));
}

static void cb_motion_resume(void *ctx) {
    board_t *b = ctx;
    b->searching = false;
    printf("         [motion] RESUME line following\n");
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
            printf(" -> stopping and scanning\n");
            break;
        case AV_EVT_SCAN_POINT:
            printf("[scan] %3d deg (%-6s): ", e->point.angle_deg,
                   side_name(scan_side_of(&g_av.geom, (float)e->point.angle_deg)));
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
            if (!p->left.found || !p->right.found) {
                printf("         [profile] an edge was beyond the scan range, width is a minimum\n");
            }
            break;
        }
        case AV_EVT_DECISION:
            printf("[decide] clearance L=%.1f R=%.1f cm (need >= %.0f) -> %s\n",
                   (double)e->choice->left_clear_cm, (double)e->choice->right_clear_cm,
                   (double)g_av.cfg.min_clearance_cm, side_name(e->side));
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
            printf("[FAILED] %s -- robot stopped. Press r to restart.\n", e->reason);
            print_metrics(av_metrics(&g_av));
            break;
    }
}

// ---------------------------------------------------------------------------
// Calibration storage: one record in the last 4 KB flash sector.
// ---------------------------------------------------------------------------

#define CAL_MAGIC 0xB5CA1B01u
#define CAL_VERSION 1u
#define CAL_FLASH_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)

typedef struct {
    uint32_t magic;
    uint32_t version;
    int32_t left_deg;
    int32_t center_deg;
    int32_t right_deg;
    int32_t fine_half_range_deg;
    uint32_t checksum; // FNV-1a over every field above
} cal_record_t;

static uint32_t cal_checksum(const cal_record_t *r) {
    const uint8_t *p = (const uint8_t *)r;
    uint32_t h = 2166136261u;
    for (size_t i = 0; i < offsetof(cal_record_t, checksum); i++) {
        h ^= p[i];
        h *= 16777619u;
    }
    return h;
}

static void cal_apply(av_config_t *cfg, const cal_record_t *r) {
    cfg->left_deg = r->left_deg;
    cfg->center_deg = r->center_deg;
    cfg->right_deg = r->right_deg;
    cfg->fine_half_range_deg = r->fine_half_range_deg;
}

static cal_record_t cal_make(int left, int center, int right, int fine_half) {
    cal_record_t r = {CAL_MAGIC, CAL_VERSION, left, center, right, fine_half, 0};
    r.checksum = cal_checksum(&r);
    return r;
}

// True only if a record is present, intact and gives a valid config.
static bool cal_load(cal_record_t *out) {
    const cal_record_t *r = (const cal_record_t *)(XIP_BASE + CAL_FLASH_OFFSET);
    if (r->magic != CAL_MAGIC || r->version != CAL_VERSION || r->checksum != cal_checksum(r)) {
        return false;
    }
    av_config_t cfg;
    av_config_default(&cfg);
    cal_apply(&cfg, r);
    if (av_config_check(&cfg) != NULL) return false;
    *out = *r;
    return true;
}

static void flash_write_page(void *param) {
    flash_range_erase(CAL_FLASH_OFFSET, FLASH_SECTOR_SIZE);
    if (param != NULL) flash_range_program(CAL_FLASH_OFFSET, param, FLASH_PAGE_SIZE);
}

// rec == NULL erases the saved calibration.
static bool cal_store(const cal_record_t *rec) {
    static uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof page);
    if (rec != NULL) memcpy(page, rec, sizeof *rec);
    if (flash_safe_execute(flash_write_page, rec != NULL ? page : NULL, 1000) != PICO_OK) {
        return false;
    }
    cal_record_t check;
    if (rec == NULL) return !cal_load(&check);
    return cal_load(&check) && memcmp(&check, rec, sizeof check) == 0;
}

static void print_cal(const char *label, const cal_record_t *r) {
    printf("%s LEFT %ld deg, CENTER %ld deg, RIGHT %ld deg, fine scan +/-%ld deg\n", label,
           (long)r->left_deg, (long)r->center_deg, (long)r->right_deg,
           (long)r->fine_half_range_deg);
}

// ---------------------------------------------------------------------------
// Servo calibration: step the servo by hand to find the safe range.
// ---------------------------------------------------------------------------

static void cal_help(void) {
    printf("\n--- Servo calibration ---\n"
           "  a / d : step -5 / +5 deg      A / D : step -1 / +1 deg\n"
           "  c     : back to 90 deg        m     : measure distance here\n"
           "  l     : mark current angle as LEFT limit (sensor looking left)\n"
           "  r     : mark current angle as RIGHT limit (sensor looking right)\n"
           "  p     : preview the calibration from the marked limits\n"
           "  s     : SAVE the calibration and start the car\n"
           "  q     : quit without saving (keeps the previous calibration)\n"
           "  h     : help\n"
           "Step slowly. Stop as soon as the servo touches anything, buzzes or\n"
           "strains, step back until it moves freely, then mark that angle.\n\n");
}

static void cal_move(int angle) {
    servo_set_angle(&g_board.servo, angle);
    printf("servo -> %3d deg\n", angle);
}

// Builds a symmetric range inside the marked limits and checks it is valid.
static bool cal_suggest(int left, int right, bool have_left, bool have_right, cal_record_t *out) {
    if (!have_left || !have_right) {
        printf("mark both limits first (l and r)\n");
        return false;
    }
    const int center = 90;
    bool left_is_higher = left > right;
    if (left == right || (left > center) != left_is_higher || (right > center) == left_is_higher) {
        printf("the two limits must be on opposite sides of 90 deg; re-mark them\n");
        return false;
    }

    // Use the smaller side (minus a margin) so both sides are equal, as the scan assumes.
    int left_half = abs(left - center);
    int right_half = abs(right - center);
    int half = (left_half < right_half ? left_half : right_half) - CAL_SAFETY_MARGIN_DEG;
    half -= half % 2; // span must split into 4 equal coarse steps
    if (half < 10) {
        printf("usable range is only +/-%d deg: too narrow to scan. Clear the obstruction.\n",
               half < 0 ? 0 : half);
        return false;
    }

    av_config_t cfg;
    av_config_default(&cfg);
    int fine_half = half - half % cfg.fine_step_deg;
    if (fine_half < cfg.fine_step_deg) fine_half = cfg.fine_step_deg;
    *out = cal_make(left_is_higher ? center + half : center - half, center,
                    left_is_higher ? center - half : center + half, fine_half);
    cal_apply(&cfg, out);

    const char *err = av_config_check(&cfg);
    printf("\nMarked limits: LEFT %d deg, RIGHT %d deg\n", left, right);
    printf("Usable symmetric range: +/-%d deg (after a %d deg safety margin)\n", half,
           CAL_SAFETY_MARGIN_DEG);
    if (err != NULL) {
        printf("Could not build a valid calibration: %s\n", err);
        return false;
    }
    print_cal("Calibration:", out);
    return true;
}

// Returns true if a new calibration was saved into *cal.
static bool run_servo_calibration(cal_record_t *cal) {
    int angle = 90;
    int left = 0, right = 0;
    bool have_left = false, have_right = false;
    cal_record_t next;

    cal_help();
    cal_move(angle);

    while (true) {
        int ch = getchar_timeout_us(100000);
        if (ch == PICO_ERROR_TIMEOUT) continue;
        int prev = angle;
        switch (ch) {
            case 'a': angle -= 5; break;
            case 'd': angle += 5; break;
            case 'A': angle -= 1; break;
            case 'D': angle += 1; break;
            case 'c': angle = 90; break;
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
            case 'p': cal_suggest(left, right, have_left, have_right, &next); break;
            case 's':
                if (!cal_suggest(left, right, have_left, have_right, &next)) break;
                if (!cal_store(&next)) {
                    printf("ERROR: could not write flash, calibration not saved\n");
                    break;
                }
                printf("Saved to flash. It will be offered at every boot.\n");
                *cal = next;
                return true;
            case 'q':
                printf("calibration not changed\n");
                return false;
            case 'h': cal_help(); break;
            default: break;
        }
        if (angle < 0) angle = 0;
        if (angle > 180) angle = 180;
        if (angle != prev) cal_move(angle);
    }
}

// ---------------------------------------------------------------------------
// Boot menu and self-check
// ---------------------------------------------------------------------------

// Lets the tester recalibrate or keep the current calibration. Waits
// MENU_TIMEOUT_S for a key, then keeps the current calibration.
static void boot_menu(cal_record_t *cal, bool *from_flash) {
    if (!stdio_usb_connected()) {
        printf("no serial terminal: skipping the menu\n");
        return;
    }
    printf("\n=== Calibration ===\n");
    print_cal(*from_flash ? "Saved in flash:" : "No saved calibration, built-in:", cal);
    printf("  Enter / u : use this calibration and start\n"
           "  c         : calibrate the servo now\n"
           "  x         : erase the saved calibration (use built-in values)\n");

    for (int left_s = MENU_TIMEOUT_S; left_s > 0; left_s--) {
        printf("starting with this calibration in %d s...\r", left_s);
        absolute_time_t until = make_timeout_time_ms(1000);
        while (!time_reached(until)) {
            int ch = getchar_timeout_us(50000);
            if (ch == PICO_ERROR_TIMEOUT) continue;
            if (ch == '\r' || ch == '\n' || ch == 'u') {
                printf("\n");
                return;
            }
            if (ch == 'c') {
                printf("\n");
                if (run_servo_calibration(cal)) *from_flash = true;
                return;
            }
            if (ch == 'x') {
                printf("\n");
                if (!cal_store(NULL)) {
                    printf("ERROR: could not erase flash\n");
                    return;
                }
                *cal = cal_make(CAR_LEFT_DEG, CAR_CENTER_DEG, CAR_RIGHT_DEG,
                                CAR_FINE_HALF_RANGE_DEG);
                *from_flash = false;
                print_cal("Saved calibration erased, using built-in:", cal);
                return;
            }
        }
    }
    printf("\nno key pressed\n");
}

static void print_config(const av_config_t *c, bool from_flash) {
    int step = (c->left_deg - c->right_deg) / (c->coarse_points - 1);
    printf("\n=== Configuration (%s) ===\n", from_flash ? "saved calibration" : "built-in values");
    printf("servo   LEFT %d / CENTER %d / RIGHT %d deg\n", c->left_deg, c->center_deg,
           c->right_deg);
    printf("coarse  %d angles:", c->coarse_points);
    for (int i = 0; i < c->coarse_points; i++) printf(" %d", c->right_deg + i * step);
    printf("\nfine    %d deg steps out to +/-%d deg\n", c->fine_step_deg, c->fine_half_range_deg);
    printf("detect  < %.0f cm (clears at >= %.0f cm, %d readings in a row)\n",
           (double)c->detect_cm, (double)(c->detect_cm + c->clear_hysteresis_cm),
           c->detect_confirm_count);
    printf("side    clear if >= %.0f cm; emergency stop < %.0f cm\n",
           (double)c->min_clearance_cm, (double)c->emergency_stop_cm);
    printf("retry   %d rescans, %d reverses of %.0f cm\n", c->max_rescans, c->max_reverses,
           (double)c->reverse_cm);
}

// Points at each coarse angle and prints a reading, then returns to center.
static void self_check(const av_config_t *c) {
    printf("\n=== Self-check: servo sweep + sensor ===\n");
    int step = (c->left_deg - c->right_deg) / (c->coarse_points - 1);
    scan_geometry_t g = {c->center_deg, c->left_deg > c->right_deg};
    int ok = 0, faults = 0;
    for (int i = 0; i < c->coarse_points; i++) {
        int angle = c->right_deg + i * step;
        servo_set_angle(&g_board.servo, angle);
        sleep_ms(SELFTEST_SETTLE_MS);
        range_reading_t r = ultrasonic_read(&g_board.sonar, RANGE_SAMPLES, RANGE_SAMPLE_SPACING_MS);
        printf("[check] %3d deg (%-6s): ", angle, side_name(scan_side_of(&g, (float)angle)));
        print_reading(r);
        printf("\n");
        if (r.status == RANGE_OK) ok++;
        if (r.status == RANGE_FAULT) faults++;
    }
    servo_set_angle(&g_board.servo, c->center_deg);
    sleep_ms(SELFTEST_SETTLE_MS);
    if (faults == c->coarse_points) {
        printf("[check] WARNING: sensor fault at every angle. Check TRIG GP%d / ECHO GP%d / 5 V.\n",
               TRIG_PIN, ECHO_PIN);
    } else if (ok == 0) {
        printf("[check] WARNING: no distance read at any angle. Nothing in range, or check the sensor.\n");
    } else {
        printf("[check] sensor OK (%d/%d angles gave a distance)\n", ok, c->coarse_points);
    }
}

static void run_help(void) {
    printf("keys while running:  m = metrics   r = restart after FAILED   h = help\n");
}

// ---------------------------------------------------------------------------

int main(void) {
    stdio_init_all();
    for (uint32_t waited = 0; !stdio_usb_connected() && waited < USB_WAIT_MS; waited += 100) {
        sleep_ms(100);
    }
    printf("\nBuddy 5: adaptive ultrasonic scanning + obstacle profiling\n");
    printf("pins: TRIG GP%d, ECHO GP%d, servo GP%d. Wheels are simulated (no motor output).\n",
           TRIG_PIN, ECHO_PIN, SERVO_PIN);
    if (watchdog_caused_reboot()) {
        printf("WARNING: previous run was reset by the watchdog\n");
    }

    ultrasonic_init(&g_board.sonar, TRIG_PIN, ECHO_PIN);
    servo_init(&g_board.servo, SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
    servo_set_angle(&g_board.servo, CAR_CENTER_DEG);

    cal_record_t cal;
    bool from_flash = cal_load(&cal);
    if (!from_flash) {
        cal = cal_make(CAR_LEFT_DEG, CAR_CENTER_DEG, CAR_RIGHT_DEG, CAR_FINE_HALF_RANGE_DEG);
    }
    boot_menu(&cal, &from_flash);

    av_config_t cfg;
    av_config_default(&cfg);
    cal_apply(&cfg, &cal);
    const char *err = av_config_check(&cfg);
    if (err != NULL) {
        printf("FATAL: bad config: %s\n", err);
        while (true) tight_loop_contents();
    }
    print_config(&cfg, from_flash);
    self_check(&cfg);

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
    printf("\n=== Running: watching ahead, obstacle if < %.0f cm ===\n", (double)cfg.detect_cm);
    run_help();

    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);
    uint32_t next_heartbeat = now_ms();
    uint32_t next_fail_note = 0;
    av_state_t last_state = av_state(&g_av);

    while (true) {
        watchdog_update();
        av_tick(&g_av);

        av_state_t state = av_state(&g_av);
        uint32_t now = now_ms();
        if (state != last_state) {
            printf("%8lu [state] %s -> %s\n", (unsigned long)now, av_state_name(last_state),
                   av_state_name(state));
            last_state = state;
            next_fail_note = now + FAILED_REMINDER_MS;
        }

        if (state == AV_STATE_MONITOR && (int32_t)(now - next_heartbeat) >= 0) {
            next_heartbeat = now + HEARTBEAT_MS;
            range_reading_t r;
            if (av_last_reading(&g_av, &r)) {
                bool blocked = r.status == RANGE_TOO_CLOSE ||
                               (r.status == RANGE_OK && r.cm < cfg.detect_cm);
                printf("%8lu [monitor] ahead: ", (unsigned long)now);
                print_reading(r);
                printf(blocked ? "  <- obstacle, confirming\n" : "  (clear)\n");
            }
        }

        if (state == AV_STATE_FAILED && (int32_t)(now - next_fail_note) >= 0) {
            next_fail_note = now + FAILED_REMINDER_MS;
            printf("%8lu [FAILED] %s. Press r to restart.\n", (unsigned long)now,
                   av_fail_reason(&g_av));
        }

        int ch = getchar_timeout_us(0);
        if (ch == 'm') {
            print_metrics(av_metrics(&g_av));
        } else if (ch == 'h') {
            run_help();
        } else if (ch == 'r' && state == AV_STATE_FAILED) {
            av_reset(&g_av);
            printf("%8lu [state] restarted, back to MONITOR\n", (unsigned long)now);
            last_state = av_state(&g_av);
        }
    }
}
