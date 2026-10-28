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
    return drv_sonar_read(&sonar, B5_SAMPLES, B5_SAMPLE_SPACING_MS);
}

static void cb_motion_stop(void *ctx)
{
    (void)ctx;
    motion_refused = false;
    /* Stopping is always safe, so Buddy 5 does it directly instead of
     * waiting for the controller queue (emergency stop mid-manoeuvre). */
    if (!bindings.urgent_stop())
    {
        tm_printf((UB*)"[B5] urgent stop NOT applied\n");
    }
}

static void refuse(const char *what, int amount)
{
    motion_refused = true;
    refused_count++;
    tm_printf((UB*)"[B5] %s %d refused: no motion executor yet\n", what, amount);
}

static void cb_motion_turn(void *ctx, float deg)
{
    (void)ctx;
    refuse("turn (deg)", (int)deg);
}

static void cb_motion_forward(void *ctx, float cm)
{
    (void)ctx;
    refuse("forward (cm)", (int)cm);
}

static void cb_motion_reverse(void *ctx, float cm)
{
    (void)ctx;
    refuse("reverse (cm)", (int)cm);
}

static void cb_motion_search_line(void *ctx, side_t toward)
{
    (void)ctx;
    refuse("line search toward side", (int)toward);
}

static void cb_motion_resume(void *ctx)
{
    (void)ctx;
    motion_refused = false; /* the controller starts line tracking */
}

static bool cb_motion_done(void *ctx)
{
    (void)ctx;
    return true; /* nothing is ever left running: every move is refused */
}

static bool cb_motion_failed(void *ctx)
{
    (void)ctx;
    return motion_refused;
}

static bool cb_line_detected(void *ctx)
{
    (void)ctx;
    return false; /* Buddy 3 line sensors not wired to Buddy 5 yet */
}

static void cb_on_event(void *ctx, const av_event_t *e)
{
    (void)ctx;
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
    tm_printf((UB*)" | refused=%lu dropped=%lu\n", (unsigned long)refused_count,
        (unsigned long)dropped_events);
}

static void subsystem5_task(INT stacd, void *exinf)
{
    controller_event_t events[B5_BRIDGE_MAX_EVENTS];
    size_t count;
    uint32_t last_status_ms;

    (void)stacd;
    (void)exinf;
    last_status_ms = cb_now_ms(NULL);
    for (;;)
    {
        av_tick(&av);
        count = b5_bridge_update(&bridge, av_state(&av),
            av.fault_streak >= av.cfg.max_consecutive_faults, events);
        queue_events(events, count);
        flush_events();
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
        (b->post_event == NULL))
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
