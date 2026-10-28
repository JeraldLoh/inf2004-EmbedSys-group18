/* Buddy 3 bench target. Built only with "make S3_BENCH=1" or "S3_BENCH=2".
 *
 *   S3_BENCH=1  SENSORS ONLY. Motors are held off by Buddy 2's driver and are
 *               never armed. Prints raw line/barcode readings every 100 ms.
 *               Use for S3-T01 (line polarity) and S3-T02 (barcode contrast).
 *
 *   S3_BENCH=2  LINE FOLLOW. Hold GP20 >= 1 s then release to start a run.
 *               The car follows the line; at a junction it stops for
 *               S3_BENCH_JUNCTION_WAIT_MS, then turns as the last barcode
 *               said, or goes STRAIGHT if no barcode was read (line_drive.c).
 *
 * Safety (reuses Buddy 2's bench platform, b2_platform_rp2040.c):
 *   - GP21 STOP at any time: its interrupt stops the motors directly.
 *   - Hardware watchdog: TIMER alarm0 is re-armed every cycle for
 *     S3_BENCH_WATCHDOG_US. If this task stalls, the alarm stops the motors.
 *   - Run time limit S3_BENCH_RUN_MS; effort capped at S3_BENCH_MAX_EFFORT.
 *   - Efforts are never negative, so the driver never sees a reversal.
 *   - Any driver error stops the run. A new run needs a fresh GP20 gesture.
 *   FIRST RUN WITH THE WHEELS RAISED.
 *
 * Buddy 2 owns the motors: this file only calls his bench seams
 * (subsystem2_bench_motor) under the same rules as his b2_bench_target.c,
 * and needs his review before use on the floor.
 */
#include "s3_bench.h"

#if APP_S3_BENCH
#include "subsystem3.h"
#include "line_drive.h"
#include "../subsystem2/subsystem2_internal.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"

#ifndef S3_BENCH_RUN_MS
#define S3_BENCH_RUN_MS 20000U        /* whole run, then stop */
#endif
#ifndef S3_BENCH_WATCHDOG_US
#define S3_BENCH_WATCHDOG_US 100000U  /* 100 ms, must be <= B2_TRIAL_MAX_US */
#endif
#define S3_BENCH_PERIOD_MS 5
#define S3_BENCH_PRINT_MS 250U

typedef struct
{
    b2_trial_io_t io;
    vehicle_controller_t controller;
    drv_motor_t *motor;
    uint32_t generation;
    uint32_t started_ms;
    bool running;
} s3_bench_t;

#if APP_S3_BENCH == 2
static uint32_t now_ms(void)
{
    return b2_platform_now_us() / 1000U;
}

#endif

static const char *command_name(route_command_t command)
{
    switch (command)
    {
        case ROUTE_COMMAND_LEFT: return "LEFT";
        case ROUTE_COMMAND_RIGHT: return "RIGHT";
        case ROUTE_COMMAND_STRAIGHT: return "STRAIGHT";
        case ROUTE_COMMAND_UTURN: return "UTURN";
        default: return "NONE";
    }
}

#if APP_S3_BENCH == 2
/* Stop everything. Safe to call more than once. */
static void bench_stop(s3_bench_t *bench, const char *why, stop_reason_mask_t reasons)
{
    (void)drv_motor_stop(bench->motor);
    bench->io.cancel_cutoff(bench->io.context);
    if (reasons != STOP_REASON_NONE)
    {
        (void)vehicle_controller_latch_stop(&bench->controller, reasons);
    }
    else
    {
        (void)vehicle_controller_set_mode(&bench->controller, SYSTEM_MODE_IDLE);
    }
    if (bench->running)
    {
        tm_printf((UB*)"[S3 BENCH] STOPPED: %s after %d ms\n", why,
            (INT)(now_ms() - bench->started_ms));
    }
    bench->running = false;
}

/* Same arming order as Buddy 2's b2_trial_begin(): clear only a released
 * physical STOP, fresh generation, clear inhibit, arm, timer BEFORE PWM. */
static bool bench_begin(s3_bench_t *bench)
{
    motion_motor_state_t state;
    controller_action_t action;

    if (bench->io.stop_pressed(bench->io.context) ||
        bench->io.stop_pending(bench->io.context))
    {
        tm_printf((UB*)"[S3 BENCH] refused: STOP held or pending\n");
        return false;
    }
    if (!vehicle_controller_clear_stop(&bench->controller, STOP_REASON_PHYSICAL) ||
        (vehicle_controller_hardware_faults(&bench->controller) != 0U))
    {
        tm_printf((UB*)"[S3 BENCH] refused: controller stop/fault latched\n");
        return false;
    }
    action = vehicle_controller_set_mode(&bench->controller, SYSTEM_MODE_CALIBRATION);
    if ((drv_motor_stop(bench->motor) != MOTION_OK) ||
        (drv_motor_read(bench->motor, &state) != MOTION_OK) ||
        (drv_motor_clear_inhibit(bench->motor, state.stop_sequence) != MOTION_OK) ||
        (drv_motor_arm(bench->motor, state.stop_sequence,
            action.authority_generation) != MOTION_OK) ||
        !bench->io.arm_cutoff(bench->io.context, S3_BENCH_WATCHDOG_US))
    {
        (void)drv_motor_stop(bench->motor);
        bench->io.cancel_cutoff(bench->io.context);
        tm_printf((UB*)"[S3 BENCH] refused: motor arm failed\n");
        return false;
    }
    bench->generation = action.authority_generation;
    bench->started_ms = now_ms();
    bench->running = true;
    return true;
}

/* Drop events that arrived while stopped, so an old junction or barcode
 * cannot act on the next run. */
static void drain_events(void)
{
    route_instruction_t route;
    junction_observation_t junction;
    while (subsystem3_take_route(&route)) { }
    while (subsystem3_take_junction(&junction)) { }
}
#endif

static void bench_run(void)
{
    static s3_bench_t bench;
    motion_hal_t hal;
#if APP_S3_BENCH == 2
    b2_platform_events_t events = {0};
    b2_gesture_t gesture = {0};
    line_drive_config_t config;
    line_drive_t drive;
    uint32_t last_print = 0U;
#endif

    /* Buddy 2 init holds GP8-GP11 low: the motors are safe from here on. */
    if (!b2_platform_init(&hal, &bench.io) || (subsystem2_bind_hal(&hal) != E_OK) ||
        (subsystem2_init() != E_OK) || (subsystem2_start() != E_OK))
    {
        (void)subsystem2_urgent_stop();
        tm_printf((UB*)"[S3 BENCH] motion init FAILED; nothing will move. Reset after diagnosis.\n");
        return;
    }
    bench.motor = subsystem2_bench_motor();
    vehicle_controller_init(&bench.controller);

    if ((subsystem3_init() != E_OK) || (subsystem3_start() != E_OK))
    {
        tm_printf((UB*)"[S3 BENCH] subsystem 3 init FAILED\n");
        return;
    }
    tm_printf((UB*)"[S3 BENCH] mode %d. Barcode calibration: sweep the barcode sensor over black and white now (3 s).\n",
        (INT)APP_S3_BENCH);

#if APP_S3_BENCH == 1
    tm_printf((UB*)"[S3 BENCH] SENSORS ONLY: motors stay off. Watch the [S3 RAW] lines.\n");
    for (;;)
    {
        route_instruction_t route;
        junction_observation_t junction;
        if (subsystem3_take_route(&route))
        {
            tm_printf((UB*)"[S3 BENCH] barcode '%c' -> %s\n",
                (INT)route.decoded_character, command_name(route.command));
        }
        if (subsystem3_take_junction(&junction))
        {
            tm_printf((UB*)"[S3 BENCH] junction seen (confirmed=%d)\n", (INT)junction.confirmed);
        }
        (void)tk_dly_tsk(20);
    }
#else
    line_drive_default_config(&config);
    tm_printf((UB*)"[S3 BENCH] LINE FOLLOW. base=%d gain=%d max=%d wait=%dms cross=%dms turn=%dms run<=%dms\n",
        (INT)config.base_effort, (INT)config.gain, (INT)config.max_effort,
        (INT)config.junction_wait_ms, (INT)config.cross_ms, (INT)config.turn_ms,
        (INT)S3_BENCH_RUN_MS);
    tm_printf((UB*)"[S3 BENCH] WHEELS RAISED FIRST. GP21 = STOP. Hold GP20 >= 1 s then RELEASE to start.\n");

    for (;;)
    {
        uint32_t t = now_ms();
        route_instruction_t route;
        junction_observation_t junction;

        /* STOP button / watchdog alarm evidence from the ISRs. */
        b2_platform_take_events(&events);
        if (bench.running && ((events.reasons != STOP_REASON_NONE) ||
            events.cutoff_fired || bench.io.stop_pressed(bench.io.context)))
        {
            bench_stop(&bench, events.cutoff_fired ? "watchdog/cutoff alarm" : "STOP button",
                (events.reasons != STOP_REASON_NONE) ? events.reasons : STOP_REASON_PHYSICAL);
        }

        if (!bench.running)
        {
            b2_ui_event_t ui = b2_gesture_update(&gesture, b2_platform_now_us(),
                b2_platform_start_pressed(), b2_platform_stop_pressed(), false);
            if (ui == B2_UI_CONFIRM)
            {
                tm_printf((UB*)"[S3 BENCH] release GP20 to start; GP21 cancels.\n");
            }
            else if (ui == B2_UI_RUN)
            {
                drain_events();
                line_drive_init(&drive, &config);
                if (bench_begin(&bench))
                {
                    tm_printf((UB*)"[S3 BENCH] RUN started\n");
                }
            }
            else
            {
                drain_events();
            }
        }
        else
        {
            int16_t left;
            int16_t right;
            int8_t steer;
            line_drive_event_t ev;
            motion_status_t status;

            /* Route BEFORE junction, every cycle. */
            if (subsystem3_take_route(&route))
            {
                line_drive_route(&drive, route.command, route.decoded_character);
                tm_printf((UB*)"[S3 BENCH] barcode '%c' -> %s (used at next junction)\n",
                    (INT)route.decoded_character, command_name(route.command));
            }
            if (subsystem3_take_junction(&junction))
            {
                line_drive_junction(&drive, t);
            }

            steer = subsystem3_steer();
            ev = line_drive_step(&drive, steer, t, &left, &right);
            if (ev == LINE_DRIVE_EVENT_JUNCTION_STOP)
            {
                tm_printf((UB*)"[S3 BENCH] JUNCTION: stopping for %d ms\n",
                    (INT)config.junction_wait_ms);
            }
            else if (ev == LINE_DRIVE_EVENT_GO)
            {
                tm_printf((UB*)"[S3 BENCH] GO %s (%s)\n", command_name(drive.go_command),
                    (drive.go_character == '-') ? "no barcode: default" : "from barcode");
            }
            else if (ev == LINE_DRIVE_EVENT_RESUME)
            {
                tm_printf((UB*)"[S3 BENCH] following line again\n");
            }

            if (ev == LINE_DRIVE_EVENT_DONE)
            {
                bench_stop(&bench, "U-turn requested (not done on the bench)", STOP_REASON_NONE);
            }
            else if ((t - bench.started_ms) >= S3_BENCH_RUN_MS)
            {
                bench_stop(&bench, "run time limit", STOP_REASON_NONE);
            }
            else if (!bench.io.arm_cutoff(bench.io.context, S3_BENCH_WATCHDOG_US))
            {
                bench_stop(&bench, "watchdog re-arm refused", STOP_REASON_CONTROLLER_FAULT);
            }
            else
            {
                status = drv_motor_apply(bench.motor, bench.generation, left, right);
                if (status != MOTION_OK)
                {
                    tm_printf((UB*)"[S3 BENCH] drv_motor_apply status=%d\n", (INT)status);
                    bench_stop(&bench, "motor driver error", STOP_REASON_MOTION_FAULT);
                }
                else if ((t - last_print) >= S3_BENCH_PRINT_MS)
                {
                    line_status_t s = subsystem3_line_status();
                    last_print = t;
                    tm_printf((UB*)"[S3 BENCH] L=%d R=%d steer=%d effort L=%d R=%d state=%d\n",
                        (INT)s.left_black, (INT)s.right_black, (INT)steer,
                        (INT)left, (INT)right, (INT)drive.state);
                }
            }
        }
        (void)tk_dly_tsk(S3_BENCH_PERIOD_MS);
    }
#endif
}

static _Alignas(8) uint32_t s3_bench_stack[1024];
static ID s3_bench_task_id;

static void s3_bench_task(INT code, void *exinf)
{
    (void)code; (void)exinf;
    bench_run();
    tk_ext_tsk();   /* init failure returns here; never enter a drive loop */
}

ER s3_bench_start(void)
{
    T_CTSK task = {0};
    ER status;
    if (s3_bench_task_id != 0) { return E_OBJ; }
    task.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF | TA_ASSPRC;
    task.task = (FP)s3_bench_task;
    task.itskpri = 5;
    task.stksz = sizeof(s3_bench_stack);
    task.bufptr = s3_bench_stack;
    task.assprc = TP_PRC1;   /* core 0: Buddy 2's platform requires it */
    s3_bench_task_id = tk_cre_tsk(&task);
    if (s3_bench_task_id < E_OK)
    {
        status = s3_bench_task_id;
        s3_bench_task_id = 0;
        return status;
    }
    status = tk_sta_tsk(s3_bench_task_id, 0);
    if ((status != E_OK) && (tk_del_tsk(s3_bench_task_id) == E_OK))
    {
        s3_bench_task_id = 0;
    }
    return status;
}
#endif
