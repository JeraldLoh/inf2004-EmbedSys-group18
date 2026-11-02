#include "b2_move_test.h"
#if APP_B2_MOVE_TEST
#include "b2_move_profile.h"
#include "../subsystem2/subsystem2_internal.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"
#include "../subsystem2/motion_calibration.h"

#define CASES 10U
static const char *const names[CASES]={
    "LEFT 30% 5s [RAISED]", "RIGHT 30% 5s [RAISED]", "BOTH 30% 5s / test GP21 [RAISED]",
    "PID STRAIGHT 100mm/s 5s", "FORWARD 300mm", "FORWARD 1000mm",
    "REVERSE 300mm", "LEFT PIVOT 90deg", "RIGHT PIVOT 90deg", "WATCHDOG cutoff [RAISED]"
};
static motion_hal_t hal;
static b2_trial_io_t io;
static b2_trial_t trial;
static vehicle_controller_t controller;
static measured_motion_executor_t executor;
/* Read-only observer must never invoke the PID owner's failure/STOP path. */
static wheel_measurement_t observer;
static motion_status_t observer_status=MOTION_UNAVAILABLE;
static uint32_t observer_last_us;
static bool observer_started;
static bool raw_feedback_valid;
static void observe(void)
{
    motion_encoder_snapshot_t raw;
    if(subsystem2_read_encoders(&raw)!=MOTION_OK)
    { observer_status=MOTION_UNAVAILABLE; observer.sample.speed_valid=false;
      if(trial.result.state==B2_TRIAL_RUNNING) { raw_feedback_valid=false; } return; }
    if(observer_started && raw.acquired_us-observer_last_us<20000U) { return; }
    observer_started=true; observer_last_us=raw.acquired_us;
    observer_status=wheel_measurement_update(&observer,&raw);
    if(observer_status!=MOTION_OK && trial.result.state==B2_TRIAL_RUNNING)
    { raw_feedback_valid=false; }
}
static bool left_ok,right_ok,stop_ok,short_ok;
static ID task_id;
static _Alignas(8) uint32_t stack[1536];

static uint32_t now_ms(void) { SYSTIM t; (void)tk_get_otm(&t); return (uint32_t)t.lo; }
static void menu(unsigned selected)
{
    tm_printf((UB*)"[B2 MOVE] case=%u %s | checks L=%d R=%d STOP=%d SHORT=%d (info only) | tap GP20=next; hold>=1s/release=ONE; GP21=STOP\n",
        selected,names[selected],(int)left_ok,(int)right_ok,(int)stop_ok,(int)short_ok);
}
static void report(bool active)
{
    motion_motor_state_t m;
    motion_encoder_snapshot_t raw;
    const wheel_measurement_sample_t *speed=active?&executor.status.wheels:&observer.sample;
    bool valid;
    if (subsystem2_read_motor(&m)!=MOTION_OK || subsystem2_read_encoders(&raw)!=MOTION_OK) { return; }
    valid=speed->speed_valid && (active || observer_status==MOTION_OK) &&
        raw.acquired_us-speed->acquired_us<=executor.config.measurement.max_gap_us;
    tm_printf((UB*)"[B2 MOVE] target L=%ld R=%ld measured L=%lu R=%lu mm/s valid=%d | effort L=%d R=%d | counts L=%lu R=%lu | production_qualified=%d owner=%s coalesced_ever=%d batches L=%lu R=%lu speed_status=%u\n",
        (long)(executor.status.target_um_s[0]/1000),(long)(executor.status.target_um_s[1]/1000),
        (unsigned long)(valid?speed->speed_um_s[0]/1000U:0U),
        (unsigned long)(valid?speed->speed_um_s[1]/1000U:0U),
        (int)valid,(int)m.left_effort,(int)m.right_effort,
        (unsigned long)raw.left_count,(unsigned long)raw.right_count,(int)executor.config.qualified,
        active?"PID":(trial.result.state==B2_TRIAL_RUNNING?"RAW":"IDLE"),
        (int)raw.coalesced_edges_seen,(unsigned long)raw.coalesced_batches[0],
        (unsigned long)raw.coalesced_batches[1],(unsigned)(active?(valid?MOTION_OK:MOTION_UNAVAILABLE):observer_status));
}
static void end_test(void)
{
    measured_motion_executor_cancel(&executor,MOTION_RESULT_CANCELLED);
    io.cancel_cutoff(io.context);
    (void)vehicle_controller_set_mode(&controller,SYSTEM_MODE_IDLE);
}
static void watchdog_test(void)
{
    b2_platform_events_t evidence={0};
    motion_motor_state_t state;
    bool observed;
    motion_status_t r=b2_trial_begin(&trial,300,300,100000U);
    if(r!=MOTION_OK) { tm_printf((UB*)"[B2 MOVE] WATCHDOG admission refused=%u\n",(unsigned)r); return; }
    /* Deliberately withhold task-level trial servicing for150ms. IRQs and
     * kernel continue running; alarm0 must independently disable PWM at100ms.
     * This is not a debugger halt or a physical pin-latency measurement. */
    (void)tk_dly_tsk(150);
    b2_platform_take_events(&evidence);
    observed=subsystem2_read_motor(&state)==MOTION_OK && evidence.cutoff_fired &&
        evidence.reasons==STOP_REASON_NONE && state.inhibited && !state.armed &&
        state.left_effort==0 && state.right_effort==0;
    (void)subsystem2_urgent_stop(); /* Also stop explicitly if the alarm failed. */
    b2_trial_service(&trial,evidence.reasons,evidence.cutoff_fired);
    tm_printf((UB*)"[B2 MOVE] WATCHDOG cutoff_observed=%d after task withheld150ms; physical pins/timing UNMEASURED.\n",(int)observed);
    if(!observed && evidence.reasons==STOP_REASON_NONE)
    { (void)vehicle_controller_latch_stop(&controller,STOP_REASON_MOTION_FAULT); }
}
static void move_main(INT code,void *exinf)
{
    measured_motion_executor_config_t config;
    measured_motion_executor_io_t engine_io;
    b2_gesture_t gesture={0};
    b2_platform_events_t events={0};
    b2_trial_result_t raw_result;
    motion_goal_result_t result;
    unsigned selected=0U;
    uint32_t started=0U,last_feed=0U,last_print=0U,command_id=0U,goal_id=0U;
    uint64_t timed_base[2]={0U,0U};
    bool active=false;
    (void)code; (void)exinf;
    if (!b2_platform_init(&hal,&io) || subsystem2_bind_hal(&hal)!=E_OK ||
        subsystem2_init()!=E_OK || subsystem2_start()!=E_OK)
    { (void)subsystem2_urgent_stop(); tm_printf((UB*)"[B2 MOVE] CRITICAL init failed; NO movement.\n"); tk_ext_tsk(); return; }
    vehicle_controller_init(&controller);
    b2_move_candidate_profile(&config);
    engine_io=(measured_motion_executor_io_t){io.context,io.stop_pending,io.arm_cutoff,io.cancel_cutoff};
    if (!b2_trial_init_movement(&trial,subsystem2_bench_motor(),subsystem2_bench_encoder(),
            &controller,&io) ||
        !wheel_measurement_init(&observer,&config.measurement) ||
        !measured_motion_executor_init(&executor,subsystem2_bench_motor(),subsystem2_bench_encoder(),&config,&engine_io))
    { (void)subsystem2_urgent_stop(); tm_printf((UB*)"[B2 MOVE] CRITICAL test setup failed.\n"); tk_ext_tsk(); return; }
    tm_printf((UB*)"[B2 MOVEMENT TEST v6] NO boot motion. Manual bounded PID TUNING, not production qualification.\n");
    tm_printf((UB*)"[B2 MOVE] calibration v%u diameter=%u mm counts/rev L=%u R=%u track=%u mm; TUNING gains x1000\n",
        (unsigned)MOTION_CALIBRATION_VERSION,(unsigned)config.measurement.diameter_mm,
        (unsigned)config.measurement.counts_per_rev[0],(unsigned)config.measurement.counts_per_rev[1],
        (unsigned)config.track_width_mm);
    tm_printf((UB*)"[B2 MOVE] L P/I/D/FF=%lu/%lu/%lu/%lu R=%lu/%lu/%lu/%lu; max effort=30%%, speed=120mm/s, authority<=20s.\n",
        (unsigned long)config.pid[0].kp_milli,(unsigned long)config.pid[0].ki_milli,
        (unsigned long)config.pid[0].kd_milli,(unsigned long)config.pid[0].feedforward_milli,
        (unsigned long)config.pid[1].kp_milli,(unsigned long)config.pid[1].ki_milli,
        (unsigned long)config.pid[1].kd_milli,(unsigned long)config.pid[1].feedforward_milli);
    tm_printf((UB*)"[B2 MOVE] rolling floors linear=%u pivot=%u mm/s, tolerance=%u mm, overshoot cutoff=%u mm; TUNING candidates.\n",
        (unsigned)config.linear_goal_min_speed_mm_s,(unsigned)config.pivot_goal_min_speed_mm_s,
        (unsigned)(config.tolerance_um/1000U),
        (unsigned)(config.overshoot_um/1000U));
    tm_printf((UB*)"[B2 MOVE] TEST batch policy: <=2 batches/100ms AND <=5%% observed edges per wheel; excessive batching blocks PID. coalesced_ever is history.\n");
    tm_printf((UB*)"[B2 MOVE] Cases0/1: verify powered wheel feedback and actual FORWARD direction with wheels raised. Case2: interrupt with GP21.\n");
    tm_printf((UB*)"[B2 MOVE] INDEPENDENT cases0..9: no prerequisite sequence. Checks are informational. One test at a time; STOP/watchdog/fault protections remain. No sensor navigation.\n");
    menu(selected);
    for (;;)
    {
        uint32_t now=b2_platform_now_us();
        bool stop=b2_platform_stop_pressed();
        bool raw_active=trial.result.state==B2_TRIAL_RUNNING;
        b2_ui_event_t ui;
        b2_platform_take_events(&events);
        if (raw_active || !active)
        { b2_trial_service(&trial,events.reasons,events.cutoff_fired); }
        if (b2_trial_take_result(&trial,&raw_result))
        {
            motion_encoder_snapshot_t check;
            bool captured=subsystem2_read_encoders(&check)==MOTION_OK;
            bool clean=raw_feedback_valid && captured &&
                observer_status==MOTION_OK && observer.sample.speed_valid;
            bool left_recent=clean && check.left_edge_seen && check.acquired_us-check.left_last_edge_us<=500000U;
            bool right_recent=clean && check.right_edge_seen && check.acquired_us-check.right_last_edge_us<=500000U;
            if (raw_result.counts_valid && raw_result.state==B2_TRIAL_TIME_ENDED)
            {
                if (selected==0U && raw_result.left_edges>=100U && raw_result.right_edges<=2U && left_recent) { left_ok=true; }
                if (selected==1U && raw_result.right_edges>=100U && raw_result.left_edges<=2U && right_recent) { right_ok=true; }
            }
            if (raw_result.state==B2_TRIAL_STOPPED && (stop || (events.reasons&STOP_REASON_PHYSICAL)!=0U)) { stop_ok=true; }
            tm_printf((UB*)"[B2 MOVE] RAW end=%u L=%lu R=%lu valid=%d elapsed=%lu ms coalesced=%d feedback_ok=%d reasons=0x%lx | checks L=%d R=%d STOP=%d\n",
                (unsigned)raw_result.state,(unsigned long)raw_result.left_edges,(unsigned long)raw_result.right_edges,
                (int)raw_result.counts_valid,(unsigned long)((now-trial.start.acquired_us)/1000U),
                captured?(int)check.coalesced_edges_seen:-1,(int)raw_feedback_valid,(unsigned long)controller.safety.active_reasons,(int)left_ok,(int)right_ok,(int)stop_ok);
            gesture.phase=0U; gesture.since=now;
        }
        if (active && (stop || events.reasons!=STOP_REASON_NONE || events.cutoff_fired))
        {
            measured_motion_executor_cancel(&executor,events.cutoff_fired?MOTION_RESULT_TIMEOUT:MOTION_RESULT_CANCELLED);
            if (stop || events.reasons!=STOP_REASON_NONE)
            { (void)vehicle_controller_latch_stop(&controller,events.reasons | (stop?STOP_REASON_PHYSICAL:0U)); }
        }
        /* Only the active PID owner may execute its failure/STOP path. The
         * independent passive observer never writes motors or watchdogs. */
        observe();
        if(active) { (void)measured_motion_executor_step(&executor); }
        if (active && selected==3U && executor.status.authorized)
        {
            if (now-started>=5000000U || executor.measurement.sample.travel_um[0]-timed_base[0]>=500000U ||
                executor.measurement.sample.travel_um[1]-timed_base[1]>=500000U)
            { tm_printf((UB*)"[B2 MOVE] timed straight test ended (5s / 500mm wheel-rotation cap).\n"); end_test(); active=false; }
            else if (now-last_feed>=100000U)
            {
                motion_continuous_request_t r={0};
                r.command_id=++command_id; r.authority_generation=executor.status.generation;
                r.source=MOTION_SOURCE_CALIBRATION; r.speed_mm_s=100; r.speed_cap_mm_s=100U;
                r.created_ms=now_ms(); r.expires_ms=r.created_ms+200U;
                (void)measured_motion_executor_continuous(&executor,&r,now,200000U); last_feed=now;
            }
        }
        if (active && measured_motion_executor_result(&executor,&result))
        {
            if(selected==4U && result.code==MOTION_RESULT_COMPLETED && result.odometry_valid)
            { short_ok=true; }
            tm_printf((UB*)"[B2 MOVE] goal wheel travel L=%lu R=%lu target=%lu mm (wheel rotation, not floor position).\n",
                (unsigned long)((executor.measurement.sample.travel_um[0]-executor.goal_base_um[0])/1000U),
                (unsigned long)((executor.measurement.sample.travel_um[1]-executor.goal_base_um[1])/1000U),
                (unsigned long)(executor.goal_target_um/1000U));
            tm_printf((UB*)"[B2 MOVE] RESULT id=%lu gen=%lu code=%u mm=%ld mdeg=%ld odom_valid=%d (3=completed,4=cancelled,5=timeout,6=aborted,7=fault)\n",
                (unsigned long)result.goal_id,(unsigned long)result.authority_generation,(unsigned)result.code,
                (long)result.travelled_mm,(long)result.turned_mdeg,(int)result.odometry_valid);
            if (result.code>=MOTION_RESULT_TIMEOUT)
            { (void)vehicle_controller_latch_stop(&controller,STOP_REASON_MOTION_FAULT); }
            (void)measured_motion_executor_ack(&executor,result.goal_id,result.authority_generation);
            end_test(); active=false;
        }
        if (active && !executor.status.authorized)
        {
            tm_printf((UB*)"[B2 MOVE] stopped code=%u; outputs disabled.\n",(unsigned)executor.status.last_stop);
            if (executor.status.last_stop>=MOTION_RESULT_TIMEOUT)
            { (void)vehicle_controller_latch_stop(&controller,STOP_REASON_MOTION_FAULT); }
            end_test(); active=false;
        }
        ui=b2_gesture_update(&gesture,now,b2_platform_start_pressed(),stop,
            active || trial.result.state==B2_TRIAL_RUNNING);
        if (ui==B2_UI_SELECTED) { selected=(selected+1U)%CASES; menu(selected); }
        if (ui==B2_UI_CONFIRM)
        { tm_printf((UB*)"[B2 MOVE] RELEASE to run case=%u %s ONCE; GP21 cancels.\n",selected,names[selected]); }
        if (ui==B2_UI_RUN)
        {
            if (io.stop_pressed(io.context) || io.stop_pending(io.context) ||
                !vehicle_controller_clear_stop(&controller,STOP_REASON_PHYSICAL))
            { tm_printf((UB*)"[B2 MOVE] refused: STOP/fault latched. Faults require reset after diagnosis.\n"); }
            else if (selected<3U)
            {
                motion_status_t r;
                if(selected==0U) { left_ok=false; }
                if(selected==1U) { right_ok=false; }
                raw_feedback_valid=true;
                r=b2_trial_begin(&trial,selected==1U?0:300,selected==0U?0:300,5000000U);
                tm_printf((UB*)"[B2 MOVE] raw request=%u power=30%% duration=5s.\n",(unsigned)r);
            }
            else if(selected==9U) { watchdog_test(); }
            else
            {
                controller_action_t a=vehicle_controller_set_mode(&controller,SYSTEM_MODE_CALIBRATION);
                uint32_t limit=selected==5U?15000000U:(selected==3U?6500000U:10000000U);
                motion_status_t r;
                /* Fresh acquisition baseline after raw trials/long idle. No
                 * encoder reset and no clearing sticky acquisition evidence. */
                if(!measured_motion_executor_init(&executor,subsystem2_bench_motor(),
                    subsystem2_bench_encoder(),&config,&engine_io)) { r=MOTION_INVALID; }
                else { r=measured_motion_executor_authorize_calibration(&executor,a.authority_generation,now,limit); }
                if (r==MOTION_OK && selected==3U)
                {
                    motion_continuous_request_t request={0};
                    request.command_id=++command_id; request.authority_generation=a.authority_generation;
                    request.source=MOTION_SOURCE_CALIBRATION; request.speed_mm_s=100; request.speed_cap_mm_s=100U;
                    request.created_ms=now_ms(); request.expires_ms=request.created_ms+200U;
                    r=measured_motion_executor_continuous(&executor,&request,now,200000U);
                }
                else if (r==MOTION_OK)
                {
                    motion_goal_request_t request={0};
                    request.goal_id=++goal_id; request.authority_generation=a.authority_generation;
                    request.source=MOTION_SOURCE_CALIBRATION; request.max_speed_mm_s=100U;
                    request.type=selected==6U?MOTION_GOAL_REVERSE:(selected==7U?MOTION_GOAL_PIVOT_LEFT:
                        (selected==8U?MOTION_GOAL_PIVOT_RIGHT:MOTION_GOAL_FORWARD));
                    if (selected>=7U) { request.target_mdeg=90000; }
                    else { request.target_mm=selected==5U?1000:300; }
                    r=measured_motion_executor_goal(&executor,&request,now,1000000U);
                }
                active=r==MOTION_OK; started=now; last_feed=now;
                timed_base[0]=executor.measurement.sample.travel_um[0]; timed_base[1]=executor.measurement.sample.travel_um[1];
                if (!active) { end_test(); }
                tm_printf((UB*)"[B2 MOVE] CALIBRATION request=%u bounded=%lu ms; production_qualified=%d\n",
                    (unsigned)r,(unsigned long)(limit/1000U),(int)executor.config.qualified);
            }
        }
        if (now-last_print>=((active || trial.result.state==B2_TRIAL_RUNNING)?100000U:1000000U))
        { report(active); if (!active && trial.result.state!=B2_TRIAL_RUNNING) { menu(selected); } last_print=now; }
        (void)tk_dly_tsk(5);
    }
}
ER b2_move_test_start(void)
{
    T_CTSK ct={0}; ER r;
    if(task_id!=0) { return E_OBJ; }
    ct.tskatr=TA_HLNG|TA_RNG0|TA_ASSPRC|TA_USERBUF; ct.assprc=TP_PRC1;
    ct.task=(FP)move_main; ct.itskpri=6; ct.stksz=sizeof(stack); ct.bufptr=stack;
    task_id=tk_cre_tsk(&ct);
    if(task_id<E_OK) { r=task_id; task_id=0; return r; }
    r=tk_sta_tsk(task_id,0);
    if(r!=E_OK) { (void)tk_del_tsk(task_id); task_id=0; }
    return r;
}
#endif
