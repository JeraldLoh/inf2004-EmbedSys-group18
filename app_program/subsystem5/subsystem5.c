/* Buddy 5 kernel task: avoidance state machine + controller mission events. */
/* Use the C library's size_t (avoidance.h), not the kernel's (tk/syslib.h). */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <string.h>

#include "subsystem5.h"
#include "avoidance.h"
#include "b5_mission_bridge.h"
#include "drv_servo.h"
#include "drv_sonar.h"
#include "../project_config/pinmap.h"
#if APP_CAR_MQTT
#include "../subsystem1/buddy1_bridge.h"
#include <math.h>
static b1_s5_telemetry b5_wifi = {.front_status=4U,.last_status=4U};
static uint32_t b5_wifi_at;
#endif

#define B5_TASK_PRIORITY 12   /* provisional: below the controller (10) */
#define B5_TASK_STACK 2048
#define B5_TICK_MS 10U
#define B5_SAMPLES 3U
#define B5_SAMPLE_SPACING_MS 30U
#define B5_SERVO_MIN_US 500U
#define B5_SERVO_MAX_US 2500U
#define B5_PENDING_MAX 8U
#define B5_STATUS_PERIOD_MS 1000U
/* Crossings shallower than this hand over at once (avoidance.c). The return
 * leg crosses at the bypass turn, 60..75 deg; the pass leg runs along it. */
#define B5_LINE_ALIGN_MIN_DEG 20
/* Pass leg side watch (avoidance.c). The car centre passes ~15 cm outside
 * the edge (bypass_side_margin_cm), so a face square to the car under 30 cm
 * is the obstacle. Rear clearance ~ half the car length plus a margin. One
 * forward collision ping after every 3 side pings (~1.2 s at 50 mm/s). */
#define B5_PASS_BESIDE_CM 30.0f
#define B5_PASS_MAX_EXTRA_CM 40.0f
#define B5_PASS_CLEAR_MARGIN_CM 8.0f
#define B5_PASS_GUARD_EVERY 3
static subsystem5_bindings_t bindings;
static bool bound;
static bool initialized;
static bool started;
static ID task;

static drv_sonar_t sonar;
static drv_servo_t servo;
static av_t av;
static b5_bridge_t bridge;
static bool motion_refused;
static uint32_t refused_count;
static b5_motion_t motion;

/* Controller events not yet accepted by the (full) controller queue. */
static controller_event_t pending[B5_PENDING_MAX];
static size_t pending_count;
static uint32_t dropped_events;

/* ---------------------------------------------------------------------------
 * Avoidance platform callbacks (all run in the Buddy 5 task)
 * ------------------------------------------------------------------------- */

static uint32_t cb_now_ms(void *ctx)
{
    SYSTIM now;
    (void)ctx;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

/* Car trial 2026-11-03: the servo jumped straight to each new angle (up to
 * 140 deg at once), which looked violent and erratic. It now sweeps to the
 * target at B5_SERVO_MDEG_PER_MS; avoidance's servo_ms_per_deg waits for
 * the sweep before reading. */
#define B5_SERVO_MDEG_PER_MS 300 /* 300 deg/s */
static int32_t servo_target_mdeg;
static int32_t servo_pos_mdeg;
static uint32_t servo_step_ms;

static void servo_service(uint32_t now)
{
    uint32_t dt = now - servo_step_ms;
    int32_t limit = (int32_t)(dt > 100U ? 100U : dt) * B5_SERVO_MDEG_PER_MS;
    int32_t diff = servo_target_mdeg - servo_pos_mdeg;
    servo_step_ms = now;
    if (diff == 0) { return; }
    if (diff > limit) { diff = limit; }
    if (diff < -limit) { diff = -limit; }
    servo_pos_mdeg += diff;
    drv_servo_set_angle(&servo, (int)((servo_pos_mdeg + 500) / 1000));
}

static void cb_servo_set(void *ctx, int angle_deg)
{
    (void)ctx;
    servo_target_mdeg = (int32_t)angle_deg * 1000;
    servo_service(cb_now_ms(NULL));
}

static range_reading_t cb_measure(void *ctx)
{
    (void)ctx;
    range_reading_t reading=drv_sonar_read(&sonar, B5_SAMPLES, B5_SAMPLE_SPACING_MS);
#if APP_CAR_MQTT
    {
        uint32_t at=cb_now_ms(NULL), mm=0;
        range_status_t status=reading.status;
        if ((unsigned)status>=RANGE_STATUS_COUNT ||
            (status==RANGE_OK && (!isfinite(reading.cm) || reading.cm<2.0f || reading.cm>400.0f))) status=RANGE_FAULT;
        if(status==RANGE_OK) mm=(uint32_t)(reading.cm*10.0f+0.5f);
        b5_wifi.last_status=status;b5_wifi.last_mm=mm;b5_wifi.last_at_ms=at;
        b5_wifi.direction_halfdeg=2*(av.heading_deg-av.origin_deg)+
            2*(av.servo_deg-av.cfg.center_deg)*(av.cfg.left_deg>av.cfg.right_deg?1:-1);
        if(av.servo_deg==av.cfg.center_deg && av.heading_deg==av.origin_deg){
            b5_wifi.front_status=status;b5_wifi.front_mm=mm;b5_wifi.front_at_ms=at;
        }
    }
#endif
    return reading;
}

/* ---- Moves: Buddy 2 goals (CAR_RUN, b5_motion.c) or refused (B5_RUN) ---- */

static bool has_motion(void)
{
    return b5_motion_ops_present(&bindings.motion);
}

static void refuse(const char *what, int amount)
{
    motion_refused = true;
    refused_count++;
    tm_printf((UB*)"[B5] %s %d refused: no motion executor in this build\n",
        what, amount);
}

static void cb_motion_stop(void *ctx)
{
    (void)ctx;
    motion_refused = false;
    if (has_motion())
    {
        b5_motion_stop(&motion);
        return;
    }
    /* No executor: stop directly instead of waiting for the controller queue
     * (emergency stop mid-manoeuvre). */
    if (!bindings.urgent_stop())
    {
        tm_printf((UB*)"[B5] urgent stop NOT applied\n");
    }
}

static void cb_motion_turn(void *ctx, float deg)
{
    (void)ctx;
    if (!has_motion())
    {
        refuse("turn (deg)", (int)deg);
        return;
    }
    b5_motion_turn(&motion, (int32_t)(deg * 1000.0f), cb_now_ms(NULL));
}

static void cb_motion_forward(void *ctx, float cm)
{
    (void)ctx;
    if (!has_motion())
    {
        refuse("forward (cm)", (int)cm);
        return;
    }
    b5_motion_straight(&motion, (int32_t)(cm * 10.0f), false, cb_now_ms(NULL));
}

static void cb_motion_reverse(void *ctx, float cm)
{
    (void)ctx;
    if (!has_motion())
    {
        refuse("reverse (cm)", (int)cm);
        return;
    }
    b5_motion_straight(&motion, (int32_t)(cm * 10.0f), true, cb_now_ms(NULL));
}

static void cb_motion_search_line(void *ctx, side_t toward)
{
    (void)ctx;
    if (!has_motion())
    {
        refuse("line search toward side", (int)toward);
        return;
    }
    b5_motion_search(&motion, toward == SIDE_LEFT);
}

static void cb_motion_resume(void *ctx)
{
    (void)ctx;
    motion_refused = false;
    b5_motion_resume(&motion); /* the controller resumes line tracking */
}

static bool cb_motion_done(void *ctx)
{
    (void)ctx;
    /* Without an executor nothing is ever left running. */
    return b5_motion_done(&motion);
}

static bool cb_motion_failed(void *ctx)
{
    (void)ctx;
    return motion_refused || b5_motion_failed(&motion);
}

static bool cb_line_detected(void *ctx)
{
    (void)ctx;
    return b5_motion_line_seen(&motion);
}

static float cb_motion_travelled(void *ctx)
{
    int32_t mm;
    (void)ctx;
    return b5_motion_travelled_mm(&motion, &mm) ? (float)mm / 10.0f : -1.0f;
}

/* ---------------------------------------------------------------------------
 * Per-encounter result line (Week 10 metrics evidence)
 *
 * One "[B5 RESULT]" line when an encounter ends, plus running totals, so a
 * logged run gives detection, width, side choice, rejoin and bypass time
 * without reading the whole trace. testing/subsystem5/b5_metrics.py turns a
 * log of these lines into the metrics table.
 * ------------------------------------------------------------------------- */

static bool enc_profile;   /* this encounter built a profile */
static bool enc_aligned;   /* the rejoin turned along the line first */
static bool enc_pass_clear; /* the side sonar ended the pass leg */
static uint32_t enc_capped0; /* pass_capped at detection */
static uint32_t rejoined_count;
static uint32_t cleared_count;
static uint32_t aborted_count;

static int cm_to_mm(float cm)
{
    return (int)(cm * 10.0f + ((cm < 0.0f) ? -0.5f : 0.5f));
}

static const char *edges_name(const obstacle_profile_t *p)
{
    if (p->left.found && p->right.found) { return "both"; }
    if (p->left.found) { return "left_only"; }
    if (p->right.found) { return "right_only"; }
    return "none";
}

static void print_result(const char *outcome, uint32_t ms, const char *reason)
{
    const av_metrics_t *m = av_metrics(&av);
    const obstacle_profile_t *p = av_profile(&av);

    tm_printf((UB*)"[B5 RESULT] n=%lu outcome=%s ms=%lu scan_ms=%lu",
        (unsigned long)m->detections, outcome, (unsigned long)ms,
        (unsigned long)(enc_profile ? m->last_scan_ms : 0U));
    if (enc_profile && (p != NULL))
    {
        tm_printf((UB*)" ahead_mm=%d width_mm=%d edges=%s centre_x_mm=%d",
            cm_to_mm(p->distance_cm), cm_to_mm(p->width_cm), edges_name(p),
            cm_to_mm(p->center.x_cm));
        tm_printf((UB*)" side=%s lclear_mm=%d rclear_mm=%d",
            side_name(av.choice.side), cm_to_mm(av.choice.left_clear_cm),
            cm_to_mm(av.choice.right_clear_cm));
    }
    else
    {
        tm_printf((UB*)" ahead_mm=- width_mm=- edges=- centre_x_mm=- side=- lclear_mm=- rclear_mm=-");
    }
    tm_printf((UB*)" aligned=%d backoffs=%d reverses=%d reacq=%d",
        enc_aligned ? 1 : 0, av.backoffs, av.reverses, av.reacquire_attempts);
    tm_printf((UB*)" pass=%s side_mm=", !av.cfg.pass_watch_side ? "off" :
        (enc_pass_clear ? "clear" : ((m->pass_capped != enc_capped0) ? "cap" : "-")));
    if (enc_pass_clear) { tm_printf((UB*)"%d", cm_to_mm(av.side_cm)); }
    else { tm_printf((UB*)"-"); }
    if (enc_pass_clear && (av.depth_cm >= 0.0f))
    {
        tm_printf((UB*)" depth_mm=%d", cm_to_mm(av.depth_cm));
    }
    else
    {
        tm_printf((UB*)" depth_mm=-");
    }
    tm_printf((UB*)" reason=\"%s\"\n", (reason != NULL) ? reason : "");
    tm_printf((UB*)"[B5 TOTALS] detections=%lu rejoined=%lu cleared=%lu failed=%lu aborted=%lu left=%lu right=%lu estops=%lu\n",
        (unsigned long)m->detections, (unsigned long)rejoined_count,
        (unsigned long)cleared_count, (unsigned long)m->failures,
        (unsigned long)aborted_count, (unsigned long)m->bypass_left,
        (unsigned long)m->bypass_right, (unsigned long)m->emergency_stops);
}

static void result_on_event(const av_event_t *e)
{
    switch (e->type)
    {
        case AV_EVT_OBSTACLE_DETECTED:
            enc_profile = false;
            enc_aligned = false;
            enc_pass_clear = false;
            enc_capped0 = av_metrics(&av)->pass_capped;
            break;
        case AV_EVT_PASS_CLEAR:
            enc_pass_clear = true;
            break;
        case AV_EVT_PROFILE_READY:
            enc_profile = true;
            break;
        case AV_EVT_LINE_ALIGN:
            enc_aligned = true;
            break;
        case AV_EVT_LINE_REACQUIRED:
            rejoined_count++;
            print_result("REJOINED", e->duration_ms, NULL);
            break;
        case AV_EVT_OBSTACLE_CLEARED:
            if (!av.off_line)   /* off the line it goes on to the line search */
            {
                cleared_count++;
                print_result("CLEARED", e->time_ms - av.t_detect, e->reason);
            }
            break;
        case AV_EVT_FAILED:
            print_result("FAILED", e->time_ms - av.t_detect, e->reason);
            break;
        default:
            break;
    }
}

#if APP_CAR_MQTT
/* ---------------------------------------------------------------------------
 * Section 5 profile / decision / plan (docs/protocol/BINARY_TELEMETRY.md)
 *
 * Each group is marked valid only if every value fits its field; nothing is
 * clamped except free range, which saturates at the sonar's 400 cm no-echo
 * value. Cleared when a new encounter starts or avoidance is reset.
 * ------------------------------------------------------------------------- */

static bool fits(float v, float lo, float hi)
{
    return isfinite(v) && (v >= lo) && (v <= hi);
}

static void wifi_profile(const obstacle_profile_t *p)
{
    const float lo = -6553.6f, hi = 6553.5f;   /* 17-bit signed mm, in cm */
    b5_wifi.profile_valid = 0U;
    if ((p == NULL) || !fits(p->distance_cm, 0.0f, 6553.5f) ||
        !fits(p->width_cm, 0.0f, 6553.5f) ||
        !fits(p->center.x_cm, lo, hi) || !fits(p->center.y_cm, lo, hi) ||
        !fits(p->left.angle_deg, -512.0f, 511.5f) || !fits(p->left.pt.x_cm, lo, hi) ||
        !fits(p->left.pt.y_cm, lo, hi) ||
        !fits(p->right.angle_deg, -512.0f, 511.5f) || !fits(p->right.pt.x_cm, lo, hi) ||
        !fits(p->right.pt.y_cm, lo, hi))
    {
        return;
    }
    b5_wifi.ahead_mm = (uint32_t)cm_to_mm(p->distance_cm);
    b5_wifi.width_mm = (uint32_t)cm_to_mm(p->width_cm);
    b5_wifi.centre_x_mm = cm_to_mm(p->center.x_cm);
    b5_wifi.centre_y_mm = cm_to_mm(p->center.y_cm);
    b5_wifi.left_found = p->left.found ? 1U : 0U;
    b5_wifi.left_angle_halfdeg = (int32_t)(p->left.angle_deg * 2.0f +
        ((p->left.angle_deg < 0.0f) ? -0.5f : 0.5f));
    b5_wifi.left_x_mm = cm_to_mm(p->left.pt.x_cm);
    b5_wifi.left_y_mm = cm_to_mm(p->left.pt.y_cm);
    b5_wifi.right_found = p->right.found ? 1U : 0U;
    b5_wifi.right_angle_halfdeg = (int32_t)(p->right.angle_deg * 2.0f +
        ((p->right.angle_deg < 0.0f) ? -0.5f : 0.5f));
    b5_wifi.right_x_mm = cm_to_mm(p->right.pt.x_cm);
    b5_wifi.right_y_mm = cm_to_mm(p->right.pt.y_cm);
    b5_wifi.profile_valid = 1U;
}

static uint32_t room_mm(float cm)
{
    if (!isfinite(cm) || (cm <= 0.0f)) { return 0U; }
    return (cm >= 400.0f) ? 4000U : (uint32_t)cm_to_mm(cm);
}

static void wifi_plan(const bypass_plan_t *plan)
{
    b5_wifi.plan_valid = 0U;
    if ((plan == NULL) || (plan->turn_deg < 1) || (plan->turn_deg > 89) ||
        !fits(plan->out_cm, 0.0f, 6553.5f) || !fits(plan->pass_cm, 0.0f, 6553.5f) ||
        !fits(plan->lateral_cm, 0.0f, 6553.5f))
    {
        return;
    }
    b5_wifi.turn_deg = (uint32_t)plan->turn_deg;
    b5_wifi.diagonal_mm = (uint32_t)cm_to_mm(plan->out_cm);
    b5_wifi.alongside_mm = (uint32_t)cm_to_mm(plan->pass_cm);
    b5_wifi.offset_mm = (uint32_t)cm_to_mm(plan->lateral_cm);
    b5_wifi.plan_valid = 1U;
}

/* Where the encounter is, every snapshot. Fields whose range is narrower
 * than the current config (fine_offset_deg is 0..10) are left at 0. */
static void wifi_progress(void)
{
    b5_wifi.leg = (av_state(&av) == AV_STATE_BYPASS) ? (uint32_t)av.leg_idx : 0U;
    b5_wifi.leg_active = av.leg_active ? 1U : 0U;
    b5_wifi.rescans = (av.rescans <= 3) ? (uint32_t)av.rescans : 3U;
    b5_wifi.reverses = (av.reverses <= 3) ? (uint32_t)av.reverses : 3U;
    b5_wifi.reacquire_attempts = (av.reacquire_attempts <= 3) ?
        (uint32_t)av.reacquire_attempts : 3U;
    b5_wifi.coarse_index = (av.coarse_idx <= 7) ? (uint32_t)av.coarse_idx : 7U;
    b5_wifi.turn_magnitude_deg = ((av.turn_mag >= 0) && (av.turn_mag <= 70)) ?
        (uint32_t)av.turn_mag : 0U;
    b5_wifi.refining = av.refining ? 1U : 0U;
}

static void wifi_clear_encounter(void)
{
    b5_wifi.decision_valid = 0U;
    b5_wifi.profile_valid = 0U;
    b5_wifi.plan_valid = 0U;
}
#endif

static void cb_on_event(void *ctx, const av_event_t *e)
{
    (void)ctx;
#if APP_CAR_MQTT
    if(e->type==AV_EVT_OBSTACLE_DETECTED){++b5_wifi.encounter;b5_wifi.decision_valid=0;}
    if(e->type==AV_EVT_DECISION){b5_wifi.decision_valid=1;b5_wifi.side=e->side;}
    if (e->type == AV_EVT_OBSTACLE_DETECTED) { wifi_clear_encounter(); }
    if (e->type == AV_EVT_PROFILE_READY) { wifi_profile(e->profile); }
    if ((e->type == AV_EVT_DECISION) && (e->choice != NULL))
    {
        b5_wifi.left_room_mm = room_mm(e->choice->left_clear_cm);
        b5_wifi.right_room_mm = room_mm(e->choice->right_clear_cm);
    }
    if (e->type == AV_EVT_BYPASS_START) { wifi_plan(e->plan); }
#endif
    result_on_event(e);
    switch (e->type)
    {
        case AV_EVT_OBSTACLE_DETECTED:
            tm_printf((UB*)"[B5] obstacle ahead: %d cm (status %d)\n",
                (int)e->point.reading.cm, (int)e->point.reading.status);
            break;
        case AV_EVT_OBSTACLE_CLEARED:
            tm_printf((UB*)"[B5] %s\n", e->reason);
            break;
        case AV_EVT_DECISION:
            tm_printf((UB*)"[B5] decision: side %d (0 none, 1 left, 2 right)\n",
                (int)e->side);
            break;
        case AV_EVT_REVERSING:
            tm_printf((UB*)"[B5] reversing (attempt %d)\n", e->attempt);
            break;
        case AV_EVT_EMERGENCY_STOP:
            tm_printf((UB*)"[B5] emergency stop\n");
            break;
        case AV_EVT_LINE_ALIGN:
            tm_printf((UB*)"[B5] line crossed at %d deg: centring, then turning along it\n",
                e->heading_deg);
            break;
        case AV_EVT_PASS_CLEAR:
            tm_printf((UB*)"[B5] passed the back corner: side %d mm, depth %d mm (-10 = unknown)\n",
                cm_to_mm(e->side_cm), (e->depth_cm >= 0.0f) ? cm_to_mm(e->depth_cm) : -10);
            break;
        case AV_EVT_FAILED:
            tm_printf((UB*)"[B5] FAILED: %s\n", e->reason);
            break;
        default:
            break;
    }
}

/* ---------------------------------------------------------------------------
 * Task
 * ------------------------------------------------------------------------- */

static void queue_events(const controller_event_t *events, size_t count)
{
    size_t i;
    for (i = 0U; i < count; i++)
    {
        if (pending_count == B5_PENDING_MAX)
        {
            /* Controller queue stuck: keep the newest. The motors are
             * already stopped directly; the dropped count shows the gap. */
            (void)memmove(&pending[0], &pending[1],
                (B5_PENDING_MAX - 1U) * sizeof(pending[0]));
            pending_count--;
            dropped_events++;
        }
        pending[pending_count++] = events[i];
    }
}

static void flush_events(void)
{
    size_t sent = 0U;
    while ((sent < pending_count) && bindings.post_event(&pending[sent]))
    {
        sent++;
    }
    if (sent > 0U)
    {
        (void)memmove(&pending[0], &pending[sent],
            (pending_count - sent) * sizeof(pending[0]));
        pending_count -= sent;
    }
}

static const char *mode_name(system_mode_t mode)
{
    static const char *const names[] = {
        "STARTUP", "IDLE", "AUTO", "MANUAL", "CALIBRATION", "STOPPED", "FAULT",
    };
    return ((unsigned)mode < (sizeof(names) / sizeof(names[0]))) ?
        names[mode] : "?";
}

static const char *mission_name(mission_state_t mission)
{
    static const char *const names[] = {
        "IDLE", "TRACK_LINE", "JUNCTION_APPROACH", "JUNCTION_EXECUTE",
        "VERIFY_LINE", "OBSTACLE_STOP", "SCAN", "PLAN_BYPASS", "BYPASS",
        "RECOVERY", "SAFE_STOP",
    };
    return ((unsigned)mission < (sizeof(names) / sizeof(names[0]))) ?
        names[mission] : "?";
}

/* One line a second, so the console shows the task is alive even when
 * nothing is in front of the sonar. */
static void print_status(void)
{
    range_reading_t reading;
    controller_status_t status;

    tm_printf((UB*)"[B5] state=%s servo=%d ", av_state_name(av_state(&av)),
        av.servo_deg);
    if (!av_last_reading(&av, &reading))
    {
        tm_printf((UB*)"dist=none");
    }
    else if (reading.status == RANGE_OK)
    {
        tm_printf((UB*)"dist=%dcm", (int)reading.cm);
    }
    else
    {
        tm_printf((UB*)"dist=%s", range_status_name(reading.status));
    }
    if ((bindings.controller_status != NULL) && bindings.controller_status(&status))
    {
        tm_printf((UB*)" | ctrl mode=%s mission=%s stops=%lu",
            mode_name(status.mode), mission_name(status.mission),
            (unsigned long)status.stop_commands);
    }
    tm_printf((UB*)" | move=%d refused=%lu dropped=%lu\n", (int)motion.state,
        (unsigned long)refused_count, (unsigned long)dropped_events);
}

/* With an executor (CAR_RUN), avoidance only runs while the controller is
 * driving in AUTO. Otherwise a hand in front of the parked car would run a
 * whole encounter, fail (no authority to move) and leave avoidance in
 * FAILED, which only av_reset() leaves. So: reset when AUTO ends, run again
 * when it starts. Not reset on arming: av_reset() stops the motion, which
 * would also refuse the controller's first line requests of the new run.
 * Without an executor (B5_RUN) it always runs, as before. */
static bool avoidance_armed(bool was_armed)
{
    controller_status_t status;

    if (!has_motion() || (bindings.controller_status == NULL))
    {
        return true;
    }
    if (!bindings.controller_status(&status))
    {
        return was_armed;   /* controller busy: keep the last answer */
    }
    return status.mode == SYSTEM_MODE_AUTO;
}

static void subsystem5_task(INT stacd, void *exinf)
{
    controller_event_t events[B5_BRIDGE_MAX_EVENTS];
    size_t count;
    uint32_t last_status_ms;
    bool armed = false;
    bool now_armed;

    (void)stacd;
    (void)exinf;
    last_status_ms = cb_now_ms(NULL);
    for (;;)
    {
        now_armed = avoidance_armed(armed);
        if (!now_armed && armed)
        {
            if ((av_state(&av) != AV_STATE_MONITOR) && (av_state(&av) != AV_STATE_FAILED))
            {
                /* Run stopped mid-encounter: count it, so detections add up. */
                aborted_count++;
                print_result("ABORTED", cb_now_ms(NULL) - av.t_detect, "AUTO ended");
            }
            av_reset(&av);              /* back to MONITOR for the next run */
#if APP_CAR_MQTT
            b5_wifi.front_status=4;b5_wifi.last_status=4;b5_wifi.decision_valid=0;
            wifi_clear_encounter();
#endif
            b5_bridge_init(&bridge);
        }
        armed = now_armed;
        servo_service(cb_now_ms(NULL));
        if (armed)
        {
            b5_motion_service(&motion, cb_now_ms(NULL));
            av_tick(&av);
            count = b5_bridge_update(&bridge, av_state(&av),
                av.fault_streak >= av.cfg.max_consecutive_faults, events);
            queue_events(events, count);
        }
        flush_events();
#if APP_CAR_MQTT
        {
            uint32_t at=cb_now_ms(NULL);
            if((uint32_t)(at-b5_wifi_at)>=100U){
                const av_metrics_t *m=av_metrics(&av);
                b5_wifi.state=av_state(&av);
                b5_wifi.heading_deg=av.heading_deg;
                b5_wifi.servo_valid=av.servo_known;
                b5_wifi.servo_deg=av.servo_deg;
                b5_wifi.detections=m->detections;b5_wifi.cleared=m->obstacle_cleared;
                b5_wifi.bypass_left=m->bypass_left;b5_wifi.bypass_right=m->bypass_right;
                b5_wifi.total_rescans=m->rescans;b5_wifi.total_reverses=m->reverses;
                b5_wifi.emergency_stops=m->emergency_stops;b5_wifi.sensor_faults=m->sensor_faults;
                b5_wifi.failures=m->failures;b5_wifi.motion_timeouts=m->motion_timeouts;
                b5_wifi.line_search_ok=m->reacquire_ok;b5_wifi.line_search_retry=m->reacquire_retries;
                b5_wifi.line_search_fail=m->reacquire_fail;b5_wifi.car_turns=m->car_turns;
                b5_wifi.profiles_complete=m->profiles_complete;b5_wifi.profiles_partial=m->profiles_partial;
                b5_wifi.last_scan_ms=m->last_scan_ms;b5_wifi.last_encounter_ms=m->last_encounter_ms;
                b5_wifi.max_encounter_ms=m->max_encounter_ms;
                wifi_progress();
                /* Profile, decision and plan are filled from the avoidance
                 * events (cb_on_event); no event stream is exported. */
                const bool bypass=av_state(&av)==AV_STATE_BYPASS;
                b1_live_bypass live={.at_ms=at,.fast=0,.active=bypass,
                    .backing_off=av_state(&av)==AV_STATE_REVERSE && av.backing_off,.segment=bypass?(uint32_t)av.leg_idx:0,
                    .segments=bypass?(uint32_t)7:0,.side=av.choice.side,
                    .backoffs=(uint32_t)av.backoffs,.elapsed_ms=av_is_active(&av)?at-av.t_detect:0};
                (void)b1_submit_live_status(5,&live);
                if(b1_submit_section(5,&b5_wifi,at)) b5_wifi_at=at;
            }
        }
#endif
        if ((uint32_t)(cb_now_ms(NULL) - last_status_ms) >= B5_STATUS_PERIOD_MS)
        {
            last_status_ms = cb_now_ms(NULL);
            print_status();
        }
        (void)tk_dly_tsk(B5_TICK_MS);
    }
}

/* ---------------------------------------------------------------------------
 * Lifecycle
 * ------------------------------------------------------------------------- */

ER subsystem5_bind(const subsystem5_bindings_t *b)
{
    if ((b == NULL) || (b->dispatcher == NULL) || (b->urgent_stop == NULL) ||
        (b->post_event == NULL) || !b5_motion_ops_valid(&b->motion))
    {
        return E_PAR;
    }
    if (bound)
    {
        return E_OBJ;
    }
    bindings = *b;
    bound = true;
    return E_OK;
}

EXPORT ER subsystem5_init(void)
{
    av_config_t cfg;
    av_platform_t platform = {
        NULL, cb_now_ms, cb_servo_set, cb_measure, cb_motion_stop,
        cb_motion_turn, cb_motion_forward, cb_motion_reverse,
        cb_motion_search_line, cb_motion_resume, cb_motion_done,
        cb_motion_failed, cb_line_detected, cb_on_event,
        cb_motion_travelled,
    };
    T_CTSK ctsk = {0};
    controller_junction_config_t junction;
    ER er;
    ID id;

    if (!bound)
    {
        return E_NOSPT;
    }
    if (initialized)
    {
        return E_OBJ;
    }

    av_config_default(&cfg); /* servo 20/90/160: free mount, see avoidance.c */
    if (has_motion())
    {
        /* Rejoin: centre the turning point on the line and turn along it
         * before handing over (avoidance.c line_align_cm). The same line
         * sensors-to-axle distance the controller drives before a junction
         * pivot, so one measurement sets both. */
        controller_junction_default_config(&junction);
        cfg.line_align_cm = (float)junction.approach_mm / 10.0f;
        cfg.line_align_min_deg = B5_LINE_ALIGN_MIN_DEG;
        /* Pass leg ends once the side sonar sees the back corner go by;
         * the return drive covers the measured sideways offset. */
        cfg.pass_watch_side = true;
        cfg.pass_beside_cm = B5_PASS_BESIDE_CM;
        cfg.pass_max_extra_cm = B5_PASS_MAX_EXTRA_CM;
        cfg.pass_clear_margin_cm = B5_PASS_CLEAR_MARGIN_CM;
        cfg.pass_guard_every = B5_PASS_GUARD_EVERY;
        cfg.return_from_side = true;
    }
    er = drv_servo_init(&servo, PIN_SCAN_SERVO, B5_SERVO_MIN_US,
        B5_SERVO_MAX_US, cfg.center_deg);
    if (er != E_OK)
    {
        return er;
    }
    servo_target_mdeg = (int32_t)cfg.center_deg * 1000;
    servo_pos_mdeg = servo_target_mdeg;
    servo_step_ms = cb_now_ms(NULL);
    er = drv_sonar_init(&sonar, bindings.dispatcher, PIN_SONAR_TRIGGER,
        PIN_SONAR_ECHO);
    if (er != E_OK)
    {
        return er;
    }
    if (!av_init(&av, &cfg, &platform))
    {
        (void)drv_sonar_deinit(&sonar);
        return E_PAR;
    }
    b5_bridge_init(&bridge);
    b5_motion_init(&motion, &bindings.motion, NULL);

    /* Core 0, the same core that services the GPIO interrupt writing the
     * echo timestamps this task reads. */
    ctsk.tskatr = TA_HLNG | TA_RNG0 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1; /* TP_PRC1 is core 0 */
    ctsk.task = (FP)subsystem5_task;
    ctsk.itskpri = B5_TASK_PRIORITY;
    ctsk.stksz = B5_TASK_STACK;
    id = tk_cre_tsk(&ctsk);
    if (id < E_OK)
    {
        (void)drv_sonar_deinit(&sonar);
        return id;
    }
    task = id;
    initialized = true;
    return E_OK;
}

EXPORT ER subsystem5_start(void)
{
    ER er;

    if (!initialized || started)
    {
        return E_OBJ;
    }
    er = tk_sta_tsk(task, 0);
    if (er == E_OK)
    {
        started = true;
    }
    return er;
}
