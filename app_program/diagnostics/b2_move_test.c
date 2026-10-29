#include "b2_move_test.h"
#if APP_B2_MOVE_TEST
#include "b2_move_profile.h"
#include "../subsystem2/subsystem2_internal.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"
#include "../subsystem2/motion_calibration.h"

#define CASES 10U
static const char *const names[CASES]={
    "LEFT 30% 400ms [RAISED]", "RIGHT 30% 400ms [RAISED]", "BOTH 30% 400ms / test GP21 [RAISED]",
    "PID STRAIGHT 100mm/s 1.5s", "FORWARD 100mm", "FORWARD 1000mm",
    "REVERSE 100mm", "LEFT PIVOT 90deg", "RIGHT PIVOT 90deg", "WATCHDOG cutoff [RAISED]"
};
static motion_hal_t hal;
static b2_trial_io_t io;
static b2_trial_t trial;
static vehicle_controller_t controller;
static measured_motion_executor_t executor;
static bool left_ok,right_ok,stop_ok,short_ok;
static ID task_id;
static _Alignas(8) uint32_t stack[1536];

static uint32_t now_ms(void) { SYSTIM t; (void)tk_get_otm(&t); return (uint32_t)t.lo; }
static void menu(unsigned selected)
{
    tm_printf((UB*)"[B2 MOVE] case=%u %s | ready L=%d R=%d STOP=%d SHORT=%d | tap GP20=next; hold>=1s/release=ONE; GP21=STOP\n",
        selected,names[selected],(int)left_ok,(int)right_ok,(int)stop_ok,(int)short_ok);
}
static void report(void)
{
    motion_motor_state_t m;
    motion_encoder_snapshot_t raw;
    if (subsystem2_read_motor(&m)!=MOTION_OK || subsystem2_read_encoders(&raw)!=MOTION_OK) { return; }
    tm_printf((UB*)"[B2 MOVE] target L=%ld R=%ld measured L=%lu R=%lu mm/s valid=%d | effort L=%d R=%d | counts L=%lu R=%lu | production_qualified=%d\n",
        (long)(executor.status.target_um_s[0]/1000),(long)(executor.status.target_um_s[1]/1000),
        (unsigned long)(executor.status.wheels.speed_um_s[0]/1000U),
        (unsigned long)(executor.status.wheels.speed_um_s[1]/1000U),
        (int)executor.status.wheels.speed_valid,(int)m.left_effort,(int)m.right_effort,
        (unsigned long)raw.left_count,(unsigned long)raw.right_count,(int)executor.config.qualified);
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
    if (!b2_trial_init_limited(&trial,subsystem2_bench_motor(),subsystem2_bench_encoder(),
            &controller,&io,300,400000U) ||
        !measured_motion_executor_init(&executor,subsystem2_bench_motor(),subsystem2_bench_encoder(),&config,&engine_io))
    { (void)subsystem2_urgent_stop(); tm_printf((UB*)"[B2 MOVE] CRITICAL test setup failed.\n"); tk_ext_tsk(); return; }
    tm_printf((UB*)"[B2 MOVEMENT TEST] NO boot motion. Manual bounded PID TUNING, not production qualification.\n");
    tm_printf((UB*)"[B2 MOVE] calibration v%u diameter=%u mm counts/rev L=%u R=%u track=%u mm; TUNING gains x1000\n",
        (unsigned)MOTION_CALIBRATION_VERSION,(unsigned)config.measurement.diameter_mm,
        (unsigned)config.measurement.counts_per_rev[0],(unsigned)config.measurement.counts_per_rev[1],
        (unsigned)config.track_width_mm);
    tm_printf((UB*)"[B2 MOVE] L P/I/D/FF=%lu/%lu/%lu/%lu R=%lu/%lu/%lu/%lu; max effort=30%%, speed=120mm/s, authority<=20s.\n",
        (unsigned long)config.pid[0].kp_milli,(unsigned long)config.pid[0].ki_milli,
        (unsigned long)config.pid[0].kd_milli,(unsigned long)config.pid[0].feedforward_milli,
        (unsigned long)config.pid[1].kp_milli,(unsigned long)config.pid[1].ki_milli,
        (unsigned long)config.pid[1].kd_milli,(unsigned long)config.pid[1].feedforward_milli);
    tm_printf((UB*)"[B2 MOVE] Cases0/1: verify powered wheel feedback and actual FORWARD direction with wheels raised. Case2: interrupt with GP21.\n");
    tm_printf((UB*)"[B2 MOVE] Cases3..9 LOCKED until powered L/R feedback and in-trial STOP are observed. Case4 must complete before cases5..8. No sensor navigation.\n");
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
            if (raw_result.counts_valid && raw_result.state==B2_TRIAL_TIME_ENDED)
            {
                if (selected==0U && raw_result.left_edges>=3U && raw_result.right_edges<=2U) { left_ok=true; }
                if (selected==1U && raw_result.right_edges>=3U && raw_result.left_edges<=2U) { right_ok=true; }
            }
            if (raw_result.state==B2_TRIAL_STOPPED && (stop || (events.reasons&STOP_REASON_PHYSICAL)!=0U)) { stop_ok=true; }
            tm_printf((UB*)"[B2 MOVE] RAW end=%u L=%lu R=%lu valid=%d | ready L=%d R=%d STOP=%d\n",
                (unsigned)raw_result.state,(unsigned long)raw_result.left_edges,(unsigned long)raw_result.right_edges,
                (int)raw_result.counts_valid,(int)left_ok,(int)right_ok,(int)stop_ok);
            gesture.phase=0U; gesture.since=now;
        }
        if (active && (stop || events.reasons!=STOP_REASON_NONE || events.cutoff_fired))
        {
            measured_motion_executor_cancel(&executor,events.cutoff_fired?MOTION_RESULT_TIMEOUT:MOTION_RESULT_CANCELLED);
            if (stop || events.reasons!=STOP_REASON_NONE)
            { (void)vehicle_controller_latch_stop(&controller,events.reasons | (stop?STOP_REASON_PHYSICAL:0U)); }
        }
        /* Maintain coherent speed acquisition during raw trials too. The
         * measured engine is unauthorized/idle then and never writes PWM. */
        (void)measured_motion_executor_step(&executor);
        if (active && selected==3U && executor.status.authorized)
        {
            if (now-started>=1500000U || executor.measurement.sample.travel_um[0]-timed_base[0]>=200000U ||
                executor.measurement.sample.travel_um[1]-timed_base[1]>=200000U)
            { tm_printf((UB*)"[B2 MOVE] timed straight test ended (1.5s / 200mm wheel-rotation cap).\n"); end_test(); active=false; }
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
                motion_status_t r=b2_trial_begin(&trial,selected==1U?0:300,selected==0U?0:300,400000U);
                tm_printf((UB*)"[B2 MOVE] raw request=%u power=30%% duration=400ms.\n",(unsigned)r);
            }
            else if (!left_ok || !right_ok || !stop_ok)
            { tm_printf((UB*)"[B2 MOVE] LOCKED: complete cases0/1 powered feedback and interrupt a raw pulse with GP21 first.\n"); }
            else if(selected>=5U && selected<=8U && !short_ok)
            { tm_printf((UB*)"[B2 MOVE] LOCKED: complete forward100mm first and measure actual travel before longer/reverse/pivot tests.\n"); }
            else if(selected==9U) { watchdog_test(); }
            else
            {
                controller_action_t a=vehicle_controller_set_mode(&controller,SYSTEM_MODE_CALIBRATION);
                uint32_t limit=selected==5U?15000000U:(selected==3U?2500000U:7000000U);
                motion_status_t r=measured_motion_executor_authorize_calibration(&executor,a.authority_generation,now,limit);
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
                    else { request.target_mm=selected==5U?1000:100; }
                    r=measured_motion_executor_goal(&executor,&request,now,1000000U);
                }
                active=r==MOTION_OK; started=now; last_feed=now;
                timed_base[0]=executor.measurement.sample.travel_um[0]; timed_base[1]=executor.measurement.sample.travel_um[1];
                if (!active) { end_test(); }
                tm_printf((UB*)"[B2 MOVE] CALIBRATION request=%u bounded=%lu ms; production_qualified=%d\n",
                    (unsigned)r,(unsigned long)(limit/1000U),(int)executor.config.qualified);
            }
        }
        if (now-last_print>=(active?100000U:1000000U))
        { report(); if (!active && trial.result.state!=B2_TRIAL_RUNNING) { menu(selected); } last_print=now; }
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
