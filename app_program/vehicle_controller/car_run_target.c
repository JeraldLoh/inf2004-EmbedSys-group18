#include "car_run_target.h"
#if APP_CAR_RUN
#include "controller_task.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/subsystem2_motion.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"
#if APP_CAR_MQTT
#include "../subsystem1/subsystem1.h"
#include "../subsystem1/buddy1_bridge.h"
#include "../subsystem1/buddy1_car_policy.h"
static uint32_t wifi_command_id;
static uint32_t wifi_telemetry_at;
static void wifi_telemetry(uint32_t now)
{
    subsystem2_motion_telemetry_t m;
    controller_status_t c;
    b1_s2_telemetry s={0};
    if((uint32_t)(now-wifi_telemetry_at)<200U ||
       !subsystem2_motion_telemetry(&m) || !controller_get_status(&c) || !m.valid) return;
    s.left_speed_mm_s=m.left_mm_s;s.right_speed_mm_s=m.right_mm_s;
    s.left_speed_valid=1;s.right_speed_valid=1;
    s.motion_active=m.mode!=0U;s.sequence=(m.sampled_ms/5U)&255U;s.uptime_ms=m.sampled_ms;
    s.system_mode=c.mode;s.mission_state=c.mission;s.route_command=c.last_route;
    s.stop_reasons=c.stop_reasons;s.hardware_faults=c.hardware_faults;
    s.authority_generation=c.authority_generation;s.last_command_id=wifi_command_id;
    s.communication_state=b1_is_online()?1U:0U;
    /* No target, integrated distance or goal acknowledgement accessor here. */
    if(b1_submit_section(2,&s,m.sampled_ms)) wifi_telemetry_at=now;
}
#endif

#define SUPERVISOR_PERIOD_MS 5
#define STATUS_PERIOD_MS 1000U
#define CONSOLE_WAIT_MS 5000U
#ifndef CAR_RUN_LIMIT_MS
#define CAR_RUN_LIMIT_MS 60000U   /* whole run, then stop; override with -D */
#endif
#ifndef CAR_LINE_LOST_MS
#define CAR_LINE_LOST_MS 3000U    /* no black while following, then stop */
#endif

/* Operator stops a fresh START gesture may clear. Motion, controller and
 * sensor faults are not in this list: they stay latched until reset. */
#define OPERATOR_CLEARABLE (STOP_REASON_PHYSICAL | STOP_REASON_OBSTACLE | \
    STOP_REASON_RECOVERY_FAILED | STOP_REASON_ROUTE_REQUIRED | \
    STOP_REASON_MANUAL_EXPIRED | STOP_REASON_REMOTE)

#if TM_CONSOLE_USB_CDC
IMPORT UW tm_usb_state(void); /* libtm USB console: 3 = port opened (DTR) */
#endif

/* B2's driver keeps pointers to these, so they outlive startup. */
static motion_hal_t hal;
static b2_trial_io_t trial_io;

static uint32_t now_ms(void)
{
    SYSTIM now;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

/* ---- Outputs handed to the controller and Buddy 5 ----------------------- */

static bool controller_stop(void *context)
{
    (void)context;
    subsystem2_motion_halt();
    return true;
}

static bool controller_drive(void *context, const motion_continuous_request_t *request)
{
    (void)context;
    return subsystem2_motion_continuous(request);
}

static bool controller_read_line(void *context, controller_line_input_t *line)
{
    line_status_t status = subsystem3_line_status();
    (void)context;
    line->steer = subsystem3_steer();
    line->barcode_active = status.barcode_active != 0U;
    return true;
}

/* Junctions: Buddy 3's events and probe, Buddy 2's goals. */
static bool junction_take_route(void *context, route_instruction_t *route)
{
    (void)context;
    return subsystem3_take_route(route) != FALSE;
}

static bool junction_take(void *context, junction_observation_t *junction)
{
    (void)context;
    return subsystem3_take_junction(junction) != FALSE;
}

static bool junction_probe_requested(void *context)
{
    (void)context;
    return subsystem3_probe_requested() != FALSE;
}

static bool junction_probe_begin(void *context)
{
    (void)context;
    return subsystem3_probe_begin() == E_OK;
}

static bool junction_probe_end(void *context)
{
    (void)context;
    return subsystem3_probe_end() == E_OK;
}

static bool junction_goal(void *context, const motion_goal_request_t *goal)
{
    (void)context;
    return subsystem2_motion_goal(goal);
}

static bool junction_goal_result(void *context, goal_id_t goal_id,
    motion_goal_result_t *result)
{
    (void)context;
    return subsystem2_motion_goal_result(goal_id, result);
}

static bool urgent_stop(void)
{
    return subsystem2_urgent_stop() == MOTION_OK;
}

static bool line_seen(void)
{
    line_status_t status = subsystem3_line_status();
    return (status.state != LINE_STATE_REACQUIRE) &&
        ((status.left_black != 0U) || (status.right_black != 0U));
}

/* True once neither line sensor has seen black for CAR_LINE_LOST_MS while
 * the controller is following the line. Junction and obstacle moves leave
 * the line on purpose, so they restart the timer instead. */
static bool line_lost(uint32_t *last_line_ms, uint32_t now)
{
    controller_status_t status;

    if (!controller_get_status(&status) || line_seen() ||
        ((status.mission != MISSION_TRACK_LINE) &&
         (status.mission != MISSION_VERIFY_LINE)))
    {
        *last_line_ms = now;
        return false;
    }
    return (now - *last_line_ms) >= CAR_LINE_LOST_MS;
}

static void line_reacquire(bool toward_left)
{
    subsystem3_reacquire(toward_left ? LINE_SIDE_LEFT : LINE_SIDE_RIGHT);
}

/* ---- Bring-up: every init, then every start ----------------------------- */

static const char *failure;

static ER bring_up(void)
{
    controller_outputs_t outputs = {NULL, controller_stop, controller_drive,
        controller_read_line};
    controller_junction_outputs_t junction = {NULL, junction_take_route,
        junction_take, junction_probe_requested, junction_probe_begin,
        junction_probe_end, junction_goal, junction_goal_result};
    subsystem5_bindings_t b5 = {0};
    ER er;

    /* Buddy 2 init holds GP8-GP11 low: the motors are safe from here on. */
    if (!b2_platform_init(&hal, &trial_io) || (subsystem2_bind_hal(&hal) != E_OK) ||
        (subsystem2_init() != E_OK) || (subsystem2_start() != E_OK))
    {
        (void)subsystem2_urgent_stop();
        failure = "motion foundation failed";
        return E_SYS;
    }

    b5.dispatcher = b2_platform_dispatcher();
    b5.urgent_stop = urgent_stop;
    b5.post_event = controller_post;
    b5.controller_status = controller_get_status;
    b5.motion.authority = controller_authority;
    b5.motion.goal = subsystem2_motion_goal;
    b5.motion.continuous = subsystem2_motion_continuous;
    b5.motion.goal_result = subsystem2_motion_goal_result;
    b5.motion.halt = subsystem2_motion_halt;
    b5.motion.line_seen = line_seen;
    b5.motion.line_reacquire = line_reacquire;

    er = subsystem2_motion_init(&trial_io);
    if (er == E_OK) { er = controller_task_init(&outputs); }
    if (er == E_OK) { er = controller_task_bind_junction(&junction); }
#if APP_CAR_MQTT
    if (er == E_OK) { er = subsystem1_init(); }
#endif
    if (er == E_OK) { er = subsystem3_init(); }
    if (er == E_OK) { er = subsystem5_bind(&b5); }
    if (er == E_OK) { er = subsystem5_init(); }
    if (er != E_OK)
    {
        (void)subsystem2_urgent_stop();
        failure = "init failed; no task started";
        return er;
    }

    /* Mover first, then the controller that feeds it, then the sensors. */
    er = subsystem2_motion_start();
    if (er == E_OK) { er = controller_task_start(); }
    if (er == E_OK) { er = subsystem3_start(); }
    if (er == E_OK) { er = subsystem5_start(); }
#if APP_CAR_MQTT
    if (er == E_OK) { er = subsystem1_start(); }
#endif
    if (er != E_OK)
    {
        (void)subsystem2_urgent_stop();
        failure = "start failed";
    }
    return er;
}

/* Startup lines are lost if printed before the PC opens the COM port. */
static void wait_for_console(void)
{
#if TM_CONSOLE_USB_CDC
    UW waited = 0U;
    while ((tm_usb_state() < 3U) && (waited < CONSOLE_WAIT_MS))
    {
        (void)tk_dly_tsk(100);
        waited += 100U;
    }
    (void)tk_dly_tsk(200);
#endif
}

/* ---- Supervisor: buttons, stop evidence, run limit ---------------------- */

static bool post(controller_event_type_t type, system_mode_t mode,
    stop_reason_mask_t reasons)
{
    controller_event_t event = {type, mode, reasons, MISSION_IDLE, false};
    return controller_post(&event);
}

static void print_status(bool running)
{
    subsystem2_motion_telemetry_t t;
    controller_status_t c;

    if (!subsystem2_motion_telemetry(&t) || !controller_get_status(&c))
    {
        return;
    }
    tm_printf((UB*)"[CAR] run=%d mode=%d mission=%d stops=0x%lx steer=%d | "
        "effort L=%d R=%d speed L=%ld R=%ld mm/s | drive=%lu rej=%lu exec_rej=%lu drv_err=%lu | "
        "junc phase=%d seen=%lu done=%lu ign=%lu route=%d\n",
        (int)running, (int)c.mode, (int)c.mission, (unsigned long)c.stop_reasons,
        (int)c.last_steer, (int)t.left_effort, (int)t.right_effort,
        (long)t.left_mm_s, (long)t.right_mm_s, (unsigned long)c.drive_requests,
        (unsigned long)c.drive_rejected, (unsigned long)t.rejected,
        (unsigned long)t.driver_errors, (int)c.junction_phase,
        (unsigned long)c.junctions_seen, (unsigned long)c.junctions_done,
        (unsigned long)c.junctions_ignored, (int)c.last_route);
}

static void stop_run(bool *running, const char *why, stop_reason_mask_t reasons)
{
    subsystem2_motion_halt();
    if (reasons != STOP_REASON_NONE)
    {
        (void)post(CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_IDLE, reasons);
    }
    else
    {
        (void)post(CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_IDLE, STOP_REASON_NONE);
    }
    if (*running)
    {
        tm_printf((UB*)"[CAR] STOPPED: %s\n", why);
    }
    *running = false;
}

static void supervise(void)
{
    b2_platform_events_t events = {0};
    b2_gesture_t gesture = {0};
    controller_status_t status;
    uint32_t started_ms = 0U;
    uint32_t last_line_ms = 0U;
    uint32_t last_status = now_ms();
    bool running = false;
#if APP_CAR_MQTT
    bool remote_start_waiting = false;
#endif

    for (;;)
    {
        uint32_t t = now_ms();

        /* STOP button / cutoff alarm evidence from Buddy 2's ISRs. The ISR
         * has already stopped the motors; latch it in the controller. */
        b2_platform_take_events(&events);
        if ((events.reasons != STOP_REASON_NONE) || events.cutoff_fired)
        {
            stop_run(&running, events.cutoff_fired ? "cutoff alarm (executor stalled?)"
                : "STOP button",
                (events.reasons != STOP_REASON_NONE) ? events.reasons
                    : STOP_REASON_CONTROLLER_FAULT);
        }

#if APP_CAR_MQTT
        {
            b1_command command;unsigned choice=0U;uint32_t id=0U;bool start_queued=false;
            for(unsigned n=0;n<8U && b1_receive_vehicle(&command);++n){
                unsigned next=b1_car_command_choice(choice,command.kind);
                if(choice!=B1_STOP || command.kind==B1_STOP) id=command.command_id;
                choice=next;
            }
            if(choice==B1_STOP){
                wifi_command_id=id;remote_start_waiting=false;
                stop_run(&running,"MQTT STOP",STOP_REASON_REMOTE);
            } else if(choice==B1_START && events.reasons==STOP_REASON_NONE && !events.cutoff_fired &&
                      !controller_urgent_stop_pending() && controller_get_status(&status) &&
                      (status.mode==SYSTEM_MODE_STARTUP || status.mode==SYSTEM_MODE_IDLE || status.mode==SYSTEM_MODE_STOPPED) &&
                      b1_car_start_allowed(status.stop_reasons,b2_platform_stop_pressed(),running,status.hardware_faults)){
                if(controller_post_remote_start()){
                    wifi_command_id=id;running=true;started_ms=t;last_line_ms=t;start_queued=true;remote_start_waiting=true;
                    tm_printf((UB*)"[CAR] MQTT START requested at configured car speed\n");
                }
            }
            wifi_telemetry(t);
            /* A physical start gesture cannot undo remote STOP in this cycle. */
            if(choice==B1_STOP || start_queued){(void)tk_dly_tsk(SUPERVISOR_PERIOD_MS);continue;}
        }
#endif
#if APP_CAR_MQTT
        if(remote_start_waiting && controller_get_status(&status)){
            int progress=b1_car_start_progress(status.mode,status.stop_reasons,started_ms,t);
            if(progress!=0) remote_start_waiting=false;
            if(progress<0) stop_run(&running,"MQTT START not applied",STOP_REASON_REMOTE);
        }
#endif
        if (!running)
        {
            b2_ui_event_t ui = b2_gesture_update(&gesture, b2_platform_now_us(),
                b2_platform_start_pressed(), b2_platform_stop_pressed(), false);
            if (ui == B2_UI_CONFIRM)
            {
                tm_printf((UB*)"[CAR] release GP20 to start; GP21 cancels.\n");
            }
            else if ((ui == B2_UI_RUN) && controller_get_status(&status))
            {
                if ((status.stop_reasons & ~OPERATOR_CLEARABLE) != 0U)
                {
                    tm_printf((UB*)"[CAR] refused: fault latched (0x%lx); reset the board.\n",
                        (unsigned long)status.stop_reasons);
                }
                else if (((status.stop_reasons == STOP_REASON_NONE) ||
                        post(CONTROLLER_EVENT_CLEAR_STOP, SYSTEM_MODE_IDLE,
                            status.stop_reasons)) &&
                    post(CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO, STOP_REASON_NONE))
                {
                    running = true;
                    started_ms = t;
                    last_line_ms = t;
                    tm_printf((UB*)"[CAR] RUN: following the line (limit %lu s)\n",
                        (unsigned long)(CAR_RUN_LIMIT_MS / 1000U));
                    if (!subsystem3_barcode_calibrated())
                    {
                        tm_printf((UB*)"[CAR] barcode NOT calibrated: barcodes ignored, "
                            "every junction uses the probe. Reset to recalibrate.\n");
                    }
                }
            }
        }
        else if ((t - started_ms) >= CAR_RUN_LIMIT_MS)
        {
            stop_run(&running, "run time limit", STOP_REASON_NONE);
        }
        else if (line_lost(&last_line_ms, t))
        {
            /* Without this the curve hold steer turns the car in circles. */
            stop_run(&running, "line lost (no black for 3 s); GP20 restarts",
                STOP_REASON_NONE);
        }
        else if (controller_get_status(&status) && (status.mode != SYSTEM_MODE_AUTO))
        {
#if APP_CAR_MQTT
            if(!remote_start_waiting)
#endif
            {
            /* Controller stopped by itself (avoidance failed, fault...). */
            tm_printf((UB*)"[CAR] controller left AUTO: mission=%d stops=0x%lx\n",
                (int)status.mission, (unsigned long)status.stop_reasons);
            running = false;
            }
        }

        if ((t - last_status) >= STATUS_PERIOD_MS)
        {
            last_status = t;
            print_status(running);
        }
        (void)tk_dly_tsk(SUPERVISOR_PERIOD_MS);
    }
}

static _Alignas(8) uint32_t supervisor_stack[1024];
static ID supervisor_id;

/* B2's platform and the GPIO dispatcher only accept setup from core 0. */
static void supervisor_task(INT code, void *exinf)
{
    ER er;

    (void)code;
    (void)exinf;
    er = bring_up();
    wait_for_console();
    if (er != E_OK)
    {
        tm_printf((UB*)"[CAR] %s (%d). Motors held stopped. Reset after diagnosis.\n",
            failure, (int)er);
        tk_ext_tsk();
    }
    tm_printf((UB*)"[CAR] whole-car build: B2 draft executor, controller, B3 line, B5 obstacle.\n");
#if APP_CAR_MQTT
    tm_printf((UB*)"[CAR] Buddy 1 LIVE MQTT enabled: B2/B3/B5 telemetry and controller START/STOP.\n");
#endif
    tm_printf((UB*)"[CAR] WHEELS RAISED FIRST. GP21 = STOP. Hold GP20 >= 1 s then RELEASE to start.\n");
    tm_printf((UB*)"[CAR] Sweep the barcode sensor over black/white now (3 s calibration).\n");
    subsystem3_calibrate_barcode();
    tm_printf((UB*)"[CAR] Junctions: barcode route, else Buddy 3's probe. Turn geometry is a placeholder.\n");
    supervise();
}

ER car_run_start(void)
{
    T_CTSK task = {0};
    ER status;

    if (supervisor_id != 0)
    {
        return E_OBJ;
    }
    task.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF | TA_ASSPRC;
    task.task = (FP)supervisor_task;
    task.itskpri = 9;   /* provisional: below the executor (8) */
    task.stksz = sizeof(supervisor_stack);
    task.bufptr = supervisor_stack;
    task.assprc = TP_PRC1; /* TP_PRC1 is core 0 */
    supervisor_id = tk_cre_tsk(&task);
    if (supervisor_id < E_OK)
    {
        status = supervisor_id;
        supervisor_id = 0;
        return status;
    }
    status = tk_sta_tsk(supervisor_id, 0);
    if ((status != E_OK) && (tk_del_tsk(supervisor_id) == E_OK))
    {
        supervisor_id = 0;
    }
    return status;
}
#endif
