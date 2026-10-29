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

static void cb_servo_set(void *ctx, int angle_deg)
{
    (void)ctx;
    drv_servo_set_angle(&servo, angle_deg);
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

static void cb_on_event(void *ctx, const av_event_t *e)
{
    (void)ctx;
#if APP_CAR_MQTT
    if(e->type==AV_EVT_OBSTACLE_DETECTED){++b5_wifi.encounter;b5_wifi.decision_valid=0;}
    if(e->type==AV_EVT_DECISION){b5_wifi.decision_valid=1;b5_wifi.side=e->side;}
#endif
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
        case AV_EVT_EMERGENCY_STOP:
            tm_printf((UB*)"[B5] emergency stop\n");
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
            av_reset(&av);              /* back to MONITOR for the next run */
#if APP_CAR_MQTT
            b5_wifi.front_status=4;b5_wifi.last_status=4;b5_wifi.decision_valid=0;
#endif
            b5_bridge_init(&bridge);
        }
        armed = now_armed;
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
                /* No event stream/profile/plan API is exported by this hookup. */
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
    };
    T_CTSK ctsk = {0};
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

    av_config_default(&cfg); /* servo 78/90/102: measured on the car */
    er = drv_servo_init(&servo, PIN_SCAN_SERVO, B5_SERVO_MIN_US,
        B5_SERVO_MAX_US, cfg.center_deg);
    if (er != E_OK)
    {
        return er;
    }
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
