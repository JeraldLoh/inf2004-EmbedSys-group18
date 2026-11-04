#include "car_run_target.h"
#if APP_CAR_RUN
#include "controller_task.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/subsystem2_motion.h"
#include "../subsystem2/measured_motion_executor.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem4/s4_imu.h"

/* ---- Buddy 4: IMU terrain and hump monitoring --------------------------- */
/* Buddy 4 only observes. If it cannot start, the car still runs without it:
 * no terrain cap, no hump telemetry. Its faults are reported in its own
 * telemetry, not latched as hardware faults, because any hardware fault
 * blocks junctions and MQTT START (an unplugged IMU must not stop a run). */
static bool b4_ready;
static ER b4_init_status = E_OBJ;
static ER b4_start_status = E_OBJ;

/* Called by the IMU task every 20 ms: Buddy 2's measured wheel speeds,
 * integrated into a signed path distance for the hump height. Each wheel is
 * signed by its last non-zero target (encoders give magnitude only), so a
 * pivot adds nothing. A brief miss repeats the last value (the next call
 * catches up by time); 10 misses in a row report odometry unavailable. */
static bool b4_read_odometry(int32_t *cumulative_mm)
{
    static int64_t path_um;
    static uint32_t last_sampled_ms;
    static int8_t left_sign = 1;
    static int8_t right_sign = 1;
    static uint8_t misses;
    static bool primed;
    subsystem2_motion_telemetry_t t;
    uint32_t dt_ms;

    if (!subsystem2_motion_telemetry(&t) || !t.valid || !t.speed_valid)
    {
        if (primed && (misses < 10U))
        {
            misses++;
            *cumulative_mm = (int32_t)(path_um / 1000);
            return true;
        }
        primed = false;
        return false;
    }
    misses = 0U;
    if (t.left_target_mm_s != 0) { left_sign = (t.left_target_mm_s > 0) ? 1 : -1; }
    if (t.right_target_mm_s != 0) { right_sign = (t.right_target_mm_s > 0) ? 1 : -1; }
    if (!primed)
    {
        last_sampled_ms = t.sampled_ms;
        primed = true;
    }
    dt_ms = t.sampled_ms - last_sampled_ms;
    last_sampled_ms = t.sampled_ms;
    if (dt_ms <= 200U)   /* a longer gap would invent distance */
    {
        /* mm/s x ms = micrometres; the average of both wheels. */
        path_um += ((int64_t)(left_sign * t.left_mm_s) +
                    (int64_t)(right_sign * t.right_mm_s)) * (int64_t)dt_ms / 2;
    }
    *cumulative_mm = (int32_t)(path_um / 1000);
    return true;
}

/* Controller task, every 10 ms tick (IF-03): terrain state and speed cap. */
static bool b4_read_terrain(terrain_report_t *report)
{
    uint32_t words[S4_WORDS];
    SYSTIM now;

    if (!b4_ready || !s4_shared_read(words))
    {
        return false;
    }
    (void)tk_get_otm(&now);
    s4_terrain_report_from_words(words, (system_time_ms_t)now.lo, report);
    return true;
}

static void b4_print_status(void)
{
    static const char *const terrain[] = {
        "UNKNOWN", "LEVEL", "ASCENDING", "CREST", "DESCENDING", "SETTLING", "INVALID"
    };
    static const char *const cal[] = {"none", "RUNNING-keep still", "level", "level+mag"};
    uint32_t w[S4_WORDS];
    uint32_t state;
    int32_t pitch;
    int32_t roll;

    if (!b4_ready)
    {
        return;
    }
    if (!s4_shared_read(w))
    {
        tm_printf((UB*)"[B4] no IMU snapshot yet\n");
        return;
    }
    state = s4_get(w[S4_W_STATUS], S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH);
    pitch = s4_get_signed(w[S4_W_ATTITUDE], S4_AT_PITCH_SHIFT, S4_AT_PITCH_WIDTH) * 5;
    roll = s4_get_signed(w[S4_W_ATTITUDE], S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH) * 5;
    tm_printf((UB*)"[B4] valid=%d cal=%s terrain=%s cap=%lu pitch=%ld.%ld roll=%ld.%ld deg "
        "vib=%lu mg motion=%lu odom=%d faults=0x%lx fail=0x%lx i2c_err=%lu live_h=%lu mm | "
        "last hump %lu mm +/-%lu peak %lu deg %lu cm %s #%lu | %lu us/sample\n",
        (int)s4_get(w[S4_W_STATUS], S4_ST_VALID_SHIFT, 1U),
        cal[s4_get(w[S4_W_STATUS], S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH)],
        (state < 7U) ? terrain[state] : "?",
        (unsigned long)(s4_get(w[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) *
            S4_CAP_UNIT_MM_S),
        (long)(pitch / 10), (long)((pitch < 0 ? -pitch : pitch) % 10),
        (long)(roll / 10), (long)((roll < 0 ? -roll : roll) % 10),
        (unsigned long)(s4_get(w[S4_W_MOTION], S4_MO_VIB_SHIFT, S4_MO_VIB_WIDTH) *
            S4_VIB_UNIT_MG),
        (unsigned long)s4_get(w[S4_W_STATUS], S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH),
        (int)s4_get(w[S4_W_STATUS], S4_ST_ODOM_SHIFT, 1U),
        (unsigned long)s4_get(w[S4_W_STATUS], S4_ST_FAULT_SHIFT, S4_ST_FAULT_WIDTH),
        (unsigned long)s4_sensor_failures(),
        (unsigned long)s4_bus_errors,
        (unsigned long)s4_get(w[S4_W_MOTION], S4_MO_LIVEH_SHIFT, S4_MO_LIVEH_WIDTH),
        (unsigned long)s4_get(w[S4_W_HUMP], S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH),
        (unsigned long)s4_get(w[S4_W_HUMP], S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH),
        (unsigned long)s4_get(w[S4_W_HUMP], S4_HU_PITCH_SHIFT, S4_HU_PITCH_WIDTH),
        (unsigned long)s4_get(w[S4_W_HUMP], S4_HU_LEN_SHIFT, S4_HU_LEN_WIDTH),
        s4_get(w[S4_W_HUMP], S4_HU_VALID_SHIFT, 1U) ? "valid" : "invalid/none",
        (unsigned long)s4_get(w[S4_W_HUMP], S4_HU_SEQ_SHIFT, S4_HU_SEQ_WIDTH),
        (unsigned long)s4_sample_time_us);
}

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
    s.left_speed_valid=m.speed_valid;s.right_speed_valid=m.speed_valid;
    s.motion_active=m.mode==MOTION_EXEC_CONTINUOUS || m.mode==MOTION_EXEC_GOAL;s.sequence=(m.sampled_ms/5U)&255U;s.uptime_ms=m.sampled_ms;
    s.system_mode=c.mode;s.mission_state=c.mission;s.route_command=c.last_route;
    s.stop_reasons=c.stop_reasons;s.hardware_faults=c.hardware_faults;
    s.authority_generation=c.authority_generation;s.last_command_id=wifi_command_id;
    s.communication_state=b1_is_online()?1U:0U;
    b1_live_motion live={.at_ms=m.sampled_ms,.left_target=m.left_target_mm_s,
        .right_target=m.right_target_mm_s,.targets_valid=1,.qualified=m.qualified,
        .searching=c.line_searching,.searches=c.line_searches,.search_fails=c.line_search_fails,
        .accepted=m.accepted,.rejected=m.rejected,.driver_errors=m.driver_errors,.encoder_left_counts=m.encoder_left_counts,
        .encoder_right_counts=m.encoder_right_counts,.wheel_diameter_mm=m.wheel_diameter_mm};
    (void)b1_submit_live_status(2,&live);
    /* Integrated distance and goal acknowledgement are not supplied here. */
    {
        uint32_t w[S4_WORDS];
        if(b4_ready && s4_shared_read(w))
            s.terrain_state=s4_get(w[S4_W_STATUS],S4_ST_TERRAIN_SHIFT,S4_ST_TERRAIN_WIDTH);
    }
    if(b1_submit_section(2,&s,m.sampled_ms)) wifi_telemetry_at=now;
}
/* Buddy 4 section: its own 17-byte packed frame, every 200 ms. Independent
 * of Buddy 2's section, so IMU data still flows when the motors are idle. */
static void b4_wifi_telemetry(uint32_t now)
{
    static uint32_t sent_at;
    uint8_t frame[S4_TLM_SIZE];
    if(!b4_ready || (uint32_t)(now-sent_at)<200U) return;
    if(s4_telemetry_pack(frame,sizeof frame)==S4_TLM_SIZE && b1_submit_section(4,frame,now))
        sent_at=now;
}
#endif

#define SUPERVISOR_PERIOD_MS 5
#define STATUS_PERIOD_MS 1000U
#define CONSOLE_WAIT_MS 5000U
#ifndef CAR_RUN_LIMIT_MS
#if APP_CAR_B2_TRIAL
#define CAR_RUN_LIMIT_MS 0U       /* no whole-run limit; STOP/faults still apply */
#else
#define CAR_RUN_LIMIT_MS 60000U   /* whole run, then stop; override with -D */
#endif
#endif
#ifndef CAR_LINE_LOST_MS
#if APP_CAR_B2_TRIAL
#define CAR_LINE_LOST_MS 0U       /* both white can also mean centred */
#else
#define CAR_LINE_LOST_MS 3000U    /* no black while following, then stop */
#endif
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
    motion_motor_state_t state;
    (void)context;
    subsystem2_motion_halt();
    return subsystem2_read_motor(&state) == MOTION_OK &&
        state.left_effort == 0 && state.right_effort == 0;
}

static bool controller_drive(void *context, const motion_continuous_request_t *request)
{
    (void)context;
    return subsystem2_motion_continuous(request);
}

#if APP_OBSTACLE_TABLE
/* Table test: no line. The controller drives straight (steer 0, never lost),
 * junctions and barcodes are ignored, and Buddy 5 sees no line during the
 * bypass. Its line search after the bypass "finds" the line at once, so the
 * encounter ends normally; the supervisor then stops the car. */
static volatile bool table_bypass_done;
#endif

static bool controller_read_line(void *context, controller_line_input_t *line)
{
    line_status_t status = subsystem3_line_status();
    (void)context;
#if APP_OBSTACLE_TABLE
    (void)status;
    line->steer = 0;
    line->barcode_active = false;
    line->on_line = true;
    return true;
#endif
    line->steer = subsystem3_steer();
    line->barcode_active = status.barcode_active != 0U;
    line->on_line = (status.left_black != 0U) || (status.right_black != 0U);
    return true;
}

/* Junctions: Buddy 3's events and probe, Buddy 2's goals. */
static bool junction_take_route(void *context, route_instruction_t *route)
{
    (void)context;
#if APP_OBSTACLE_TABLE
    (void)route;
    return false;
#endif
    return subsystem3_take_route(route) != FALSE;
}

static bool junction_take(void *context, junction_observation_t *junction)
{
    (void)context;
#if APP_OBSTACLE_TABLE
    (void)junction;
    return false;
#endif
    return subsystem3_take_junction(junction) != FALSE;
}

static bool junction_probe_requested(void *context)
{
    (void)context;
#if APP_OBSTACLE_TABLE
    return false;
#endif
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
    return subsystem2_motion_goal_result_for(MOTION_SOURCE_JUNCTION, goal_id, result);
}

static bool obstacle_goal_result(goal_id_t id, motion_goal_result_t *result)
{ return subsystem2_motion_goal_result_for(MOTION_SOURCE_OBSTACLE, id, result); }

static bool urgent_stop(void)
{
    return subsystem2_urgent_stop() == MOTION_OK;
}

static bool line_seen(void)
{
    line_status_t status = subsystem3_line_status();
#if APP_OBSTACLE_TABLE
    (void)status;
    return table_bypass_done;
#endif
    return (status.state != LINE_STATE_REACQUIRE) &&
        ((status.left_black != 0U) || (status.right_black != 0U));
}

/* True once neither line sensor has seen black for CAR_LINE_LOST_MS while
 * the controller is following the line. Junction and obstacle moves leave
 * the line on purpose, so they restart the timer instead. */
static bool line_lost(uint32_t *last_line_ms, uint32_t now)
{
#if CAR_LINE_LOST_MS == 0U
    (void)last_line_ms;
    (void)now;
    return false;
#else
    controller_status_t status;

    if (!controller_get_status(&status) || line_seen() ||
        ((status.mission != MISSION_TRACK_LINE) &&
         (status.mission != MISSION_VERIFY_LINE)))
    {
        *last_line_ms = now;
        return false;
    }
    return (now - *last_line_ms) >= CAR_LINE_LOST_MS;
#endif
}

static void line_reacquire(bool toward_left)
{
#if APP_OBSTACLE_TABLE
    (void)toward_left;
    table_bypass_done = true;
    return;
#endif
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
        (subsystem2_init() != E_OK))
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
    b5.motion.goal_result = obstacle_goal_result;
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

    /* Buddy 4 last: optional, so its failure is recorded, not fatal. */
    {
        static const s4_bindings_t b4 = {b4_read_odometry};
        b4_init_status = subsystem4_init();
        if (b4_init_status == E_OK) { b4_init_status = s4_imu_bind(&b4); }
        if (b4_init_status == E_OK) { b4_init_status = controller_task_bind_terrain(b4_read_terrain); }
    }

    /* Mover first, then the controller that feeds it, then the sensors. */
    er = subsystem2_start();
    if (er == E_OK) { er = subsystem2_motion_start(); }
    if (er == E_OK) { er = controller_task_start(); }
    if (er == E_OK) { er = subsystem3_start(); }
    if (er == E_OK) { er = subsystem5_start(); }
    if ((er == E_OK) && (b4_init_status == E_OK))
    {
        b4_start_status = subsystem4_start();
        b4_ready = b4_start_status == E_OK;
    }
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
    line_status_t line = subsystem3_line_status();

    if (!subsystem2_motion_telemetry(&t) || !controller_get_status(&c))
    {
        return;
    }
    tm_printf((UB*)"[CAR] run=%d mode=%d mission=%d stops=0x%lx steer=%d | "
        "line_black L=%d R=%d state=%d last_side=%d | "
        "effort L=%d R=%d speed L=%ld R=%ld mm/s | drive=%lu rej=%lu exec_rej=%lu drv_err=%lu | "
        "target L=%ld R=%ld mm/s valid=%d qualified=%d | "
        "junc phase=%d seen=%lu done=%lu ign=%lu route=%d | search=%d n=%lu fail=%lu\n",
        (int)running, (int)c.mode, (int)c.mission, (unsigned long)c.stop_reasons,
        (int)c.last_steer, (int)line.left_black, (int)line.right_black,
        (int)line.state, (int)line.last_side, (int)t.left_effort, (int)t.right_effort,
        (long)t.left_mm_s, (long)t.right_mm_s, (unsigned long)c.drive_requests,
        (unsigned long)c.drive_rejected, (unsigned long)t.rejected,
        (unsigned long)t.driver_errors, (long)t.left_target_mm_s,
        (long)t.right_target_mm_s, (int)t.speed_valid, (int)t.qualified,
        (int)c.junction_phase,
        (unsigned long)c.junctions_seen, (unsigned long)c.junctions_done,
        (unsigned long)c.junctions_ignored, (int)c.last_route,
        (int)c.line_searching, (unsigned long)c.line_searches,
        (unsigned long)c.line_search_fails);
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
    uint32_t posted_evidence = 0U;
#if APP_CAR_MQTT
    bool remote_start_waiting = false;
#endif

    for (;;)
    {
        uint32_t t = now_ms();

        /* STOP button / cutoff alarm evidence from Buddy 2's ISRs. The ISR
         * has already stopped the motors; latch it in the controller. */
        b2_platform_peek_events(&events);
        if ((events.reasons != STOP_REASON_NONE) || events.cutoff_fired)
        {
            stop_reason_mask_t reasons = events.reasons |
                (events.cutoff_fired ? STOP_REASON_CONTROLLER_FAULT : STOP_REASON_NONE);
            if (posted_evidence != events.stop_events &&
                post(CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED, reasons))
            {
                posted_evidence = events.stop_events;
                subsystem2_motion_halt();
                running = false;
                tm_printf((UB*)"[CAR] physical STOP/cutoff evidence latched; outputs off.\n");
            }
            if (posted_evidence == events.stop_events && controller_get_status(&status) &&
                (status.stop_reasons & reasons) == reasons)
            { (void)b2_platform_ack_events(events.stop_events); }
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
            b4_wifi_telemetry(t);
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
                if (!subsystem2_motion_qualified() && !subsystem2_motion_trial_enabled())
                {
                    tm_printf((UB*)"[CAR] START refused: measured PID/load/STOP/pivot calibration unqualified.\n");
                }
                else if ((status.stop_reasons & ~OPERATOR_CLEARABLE) != 0U)
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
#if APP_OBSTACLE_TABLE
                    table_bypass_done = false;
                    tm_printf((UB*)"[TABLE] RUN: driving straight at the trial speed; "
                        "obstacle under 20 cm = stop, scan, bypass, stop.\n");
#else
                    if (CAR_RUN_LIMIT_MS == 0U)
                    { tm_printf((UB*)"[CAR] RUN: following the line (no time limit)\n"); }
                    else
                    { tm_printf((UB*)"[CAR] RUN: following the line (limit %lu s)\n",
                        (unsigned long)(CAR_RUN_LIMIT_MS / 1000U)); }
#endif
                    if (!APP_OBSTACLE_TABLE && !subsystem3_barcode_calibrated())
                    {
                        tm_printf((UB*)"[CAR] barcode NOT calibrated: barcodes ignored, "
                            "junction fallback policy applies. Reset to recalibrate.\n");
                    }
                }
            }
        }
        else if ((CAR_RUN_LIMIT_MS != 0U) && ((t - started_ms) >= CAR_RUN_LIMIT_MS))
        {
            stop_run(&running, "run time limit", STOP_REASON_NONE);
        }
#if APP_OBSTACLE_TABLE
        else if (table_bypass_done)
        {
            table_bypass_done = false;
            stop_run(&running, "table test: bypass done, back on the starting heading. "
                "Reposition, then GP20 runs again", STOP_REASON_NONE);
        }
#endif
        else if (line_lost(&last_line_ms, t))
        {
            stop_run(&running, "line lost (configured no-black timeout); GP20 restarts",
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
            b4_print_status();
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
#if APP_CAR_B2_TRIAL
    if(subsystem2_motion_qualified())
    { tm_printf((UB*)"[CAR B2] v6 wheel PID, QUALIFIED (Buddy 2 sign-off 2026-11-03). Manual START,no run limit,30%% effort (pivots 40%%),100ms watchdog.\n"); }
#endif
    if(subsystem2_motion_trial_enabled())
    { tm_printf((UB*)"[CAR B2 TRIAL] Buddy3 restored to prior testing source/steering; 50mm/s B2 trial; junction-pause fix; v6 wheel PID. Production qualification FALSE. Manual START,no run limit,30%% effort (pivots 40%%),100ms watchdog.\n"); }
    tm_printf((UB*)"[CAR] measured B2 PID executor, controller, B3 junction/line, B5 obstacle.\n");
    if (b4_ready)
    {
        tm_printf((UB*)"[CAR] B4 IMU running: terrain cap -> controller, B2 odometry -> hump height. "
            "Keep the car STILL and LEVEL until the [B4] line shows cal=level.\n");
    }
    else
    {
        tm_printf((UB*)"[CAR] B4 IMU NOT running (init %d, start %d); the car runs without terrain data.\n",
            (int)b4_init_status, (int)b4_start_status);
    }
    if (!subsystem2_motion_qualified() && !subsystem2_motion_trial_enabled())
    { tm_printf((UB*)"[CAR] UNQUALIFIED calibration: START is disabled; no automatic motion.\n"); }
#if APP_CAR_MQTT
    tm_printf((UB*)"[CAR] Buddy 1 LIVE MQTT enabled: B2/B3/B5 telemetry and controller START/STOP.\n");
#endif
#if APP_OBSTACLE_TABLE
    tm_printf((UB*)"[TABLE] OBSTACLE TABLE TEST: no line following; line sensors, junctions and\n");
    tm_printf((UB*)"        barcodes are ignored. Drives STRAIGHT until an obstacle is under 20 cm,\n");
    tm_printf((UB*)"        then scans, bypasses and stops facing the starting direction.\n");
    tm_printf((UB*)"        Clear table ~40 cm to each side and ~1 m ahead. Hand on GP21 STOP.\n");
#endif
    tm_printf((UB*)"[CAR] WHEELS RAISED FIRST. GP21 = STOP. Hold GP20 >= 1 s then RELEASE to start.\n");
#if APP_NO_BARCODE_CAL
    tm_printf((UB*)"[CAR] Barcode calibration SKIPPED (NO_BARCODE_CAL=1): barcodes ignored.\n");
#else
    tm_printf((UB*)"[CAR] Sweep the barcode sensor over black/white now (3 s calibration).\n");
    subsystem3_calibrate_barcode();
#endif
#if APP_CAR_B2_TRIAL
    tm_printf((UB*)"[CAR] Junctions: barcode route; otherwise STRAIGHT THROUGH, no stop/probe (including T).\n");
#else
    tm_printf((UB*)"[CAR] Junctions: barcode route, else Buddy 3's probe. Turn geometry is a placeholder.\n");
#endif
    tm_printf((UB*)"[CAR] No-black timeout=%lu ms; whole-run limit=%lu s (0=disabled for each).\n",
        (unsigned long)CAR_LINE_LOST_MS, (unsigned long)(CAR_RUN_LIMIT_MS / 1000U));
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
    task.itskpri = 9;   /* provisional: below the measured executor (6) */
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
