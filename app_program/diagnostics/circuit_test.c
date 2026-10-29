/* Manual circuit bring-up, not an autonomous or qualified PID drive image. */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include "circuit_test.h"
#include "circuit_controls.h"
#if APP_CIRCUIT_TEST
#include "../subsystem2/subsystem2_internal.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../subsystem2/b2_gesture.h"
#include "../subsystem2/motion_calibration.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/s4_imu.h"
#include "../subsystem5/drv_sonar.h"
#include "../subsystem5/drv_servo.h"
#include "../project_config/pinmap.h"
#include <bsp/libbsp.h>

#define CASES 12U
#define PULSE_US 250000U
static const char *const names[CASES] = {
    "LEFT +", "LEFT -", "RIGHT +", "RIGHT -", "BOTH +", "BOTH -",
    "BARCODE CAL 3s", "SONAR PING", "IMU LEVEL CAL", "SERVO 78", "SERVO 90", "SERVO 102"
};
static const int16_t left[6] = {1,-1,0,0,1,-1};
static const int16_t right[6] = {0,0,1,-1,1,-1};
static circuit_controls_t controls={150U,0U,0U};
static bool imu_seen;
static uint32_t imu_sequence,imu_updated_us;
static motion_hal_t hal;
static b2_trial_io_t io;
static b2_trial_t trial;
static vehicle_controller_t controller;
static drv_sonar_t sonar;
static drv_servo_t servo;
static bool s3_ready, s4_ready, sonar_ready;
static uint32_t aux_sequence; /* protected by the common HAL lock, saturating */

/* Called under the same lock from STOP/cutoff ISR and task context.
 * BSP functions here perform bounded MMIO only, no kernel object calls. */
static void servo_off(void *context)
{
    (void)context;
    (void)pwm_set_enabled(PIN_SCAN_SERVO,FALSE);
    (void)pwm_set_cc(PIN_SCAN_SERVO,0U);
    (void)gpio_set_val(PIN_SCAN_SERVO,0U);
    (void)gpio_set_pin(PIN_SCAN_SERVO,GPIO_MODE_OUT);
    if (aux_sequence != UINT32_MAX) { ++aux_sequence; }
}
static void servo_disable(void)
{
    motion_lock_key_t key = hal.enter(hal.context);
    servo_off(NULL);
    hal.leave(hal.context,key);
    servo.ready=false;
}
static bool servo_begin(int angle)
{
    uint32_t ticket;
    bool ok;
    motion_lock_key_t key;
    if (io.stop_pressed(io.context) || io.stop_pending(io.context) ||
        controller.safety.active_reasons != STOP_REASON_NONE) { return false; }
    key=hal.enter(hal.context); ticket=aux_sequence; hal.leave(hal.context,key);
    if (ticket==UINT32_MAX || !io.arm_cutoff(io.context,PULSE_US)) { return false; }
    key=hal.enter(hal.context);
    /* A STOP between watchdog admission and commit cannot be cleared by a
     * later servo PWM setup. Held level and saturating ticket both matter. */
    ok=aux_sequence==ticket && !b2_platform_stop_pressed();
    if (ok) { ok=drv_servo_init(&servo,PIN_SCAN_SERVO,500U,2500U,angle)==E_OK; }
    if (!ok) { servo_off(NULL); }
    hal.leave(hal.context,key);
    if (!ok) { io.cancel_cutoff(io.context); }
    return ok;
}

static void menu(unsigned selected)
{
    tm_printf((UB*)"[CIRCUIT v2] case=%u %s power=%u%% | GP20 tap=next, hold >=1s/release=ONE; GP21=STOP\n",
        selected,names[selected],(unsigned)(controls.effort/10U));
}
static void report(void)
{
    motion_encoder_snapshot_t e;
    uint16_t adc=0U;
    line_status_t line={0};
    uint32_t words[S4_WORDS];
    bool have_adc=s3_ready && subsystem3_barcode_raw(&adc);
    if (s3_ready) { line=subsystem3_line_status(); }
    if (subsystem2_read_encoders(&e)==MOTION_OK)
    {
        tm_printf((UB*)"[CIRCUIT] ENC L=%lu R=%lu epoch=%lu | GPIO7=%u GPIO0=%u black L=%u R=%u | ADC26=%u valid=%d barcode_cal=%d\n",
            (unsigned long)e.left_count,(unsigned long)e.right_count,(unsigned long)e.reset_epoch,
            gpio_get_val(PIN_LINE_LEFT),gpio_get_val(PIN_LINE_RIGHT),
            (unsigned)line.left_black,(unsigned)line.right_black,(unsigned)adc,(int)have_adc,
            (int)(s3_ready && subsystem3_barcode_calibrated()));
    }
    if (s4_ready && s4_shared_read(words))
    {
        uint32_t s=words[S4_W_STATUS];
        tm_printf((UB*)"[CIRCUIT] IMU valid=%lu cal=%lu faults=0x%lx pitch=%ld roll=%ld (0.5deg) heading=%lu deg seq=%lu failures=0x%lx fresh=%d\n",
            (unsigned long)s4_get(s,S4_ST_VALID_SHIFT,1U),
            (unsigned long)s4_get(s,S4_ST_CAL_SHIFT,S4_ST_CAL_WIDTH),
            (unsigned long)s4_get(s,S4_ST_FAULT_SHIFT,S4_ST_FAULT_WIDTH),
            (long)s4_get_signed(words[S4_W_ATTITUDE],S4_AT_PITCH_SHIFT,S4_AT_PITCH_WIDTH),
            (long)s4_get_signed(words[S4_W_ATTITUDE],S4_AT_ROLL_SHIFT,S4_AT_ROLL_WIDTH),
            (unsigned long)s4_get(words[S4_W_ATTITUDE],S4_AT_HEADING_SHIFT,S4_AT_HEADING_WIDTH),
            (unsigned long)s4_get(words[S4_W_ATTITUDE],S4_AT_SEQ_SHIFT,S4_AT_SEQ_WIDTH),
            (unsigned long)s4_sensor_failures(),
            (int)(imu_seen && b2_platform_now_us()-imu_updated_us<=250000U));
    }
    else { tm_printf((UB*)"[CIRCUIT] IMU waiting/unavailable; init=%d\n",(int)s4_ready); }
}

static void circuit_main(INT code,void *exinf)
{
    b2_gesture_t gesture={0};
    b2_platform_events_t events={0};
    b2_trial_result_t result;
    unsigned selected=0U;
    uint32_t last_print=0U,servo_since=0U,busy_since=0U,busy_us=0U;
    bool servo_active=false;
    ER s3_init,s4_init,sonar_init;
    (void)code; (void)exinf;
    if (!b2_platform_init(&hal,&io) || subsystem2_bind_hal(&hal)!=E_OK ||
        subsystem2_init()!=E_OK || !b2_platform_bind_aux_stop(servo_off,NULL))
    {
        (void)subsystem2_urgent_stop();
        tm_printf((UB*)"[CIRCUIT] CRITICAL motor/STOP init failed: NO trials. Reset after diagnosis.\n");
        tk_ext_tsk(); return;
    }
    servo_disable(); /* No servo positioning at boot. */
    vehicle_controller_init(&controller);
    s3_init=subsystem3_init();
    s4_init=s4_imu_init();
    sonar_init=drv_sonar_init(&sonar,b2_platform_dispatcher(),PIN_SONAR_TRIGGER,PIN_SONAR_ECHO);
    if (subsystem2_start()!=E_OK || !b2_trial_init_limited(&trial,subsystem2_bench_motor(),
        subsystem2_bench_encoder(),&controller,&io,B2_TRIAL_DIAGNOSTIC_MAX_EFFORT,PULSE_US))
    {
        (void)subsystem2_urgent_stop(); servo_disable();
        tm_printf((UB*)"[CIRCUIT] CRITICAL motor start failed: NO trials.\n");
        tk_ext_tsk(); return;
    }
    s3_ready=s3_init==E_OK && subsystem3_start()==E_OK;
    s4_ready=s4_init==E_OK && s4_imu_start()==E_OK;
    sonar_ready=sonar_init==E_OK;
    tm_printf((UB*)"[CIRCUIT TEST v2] manual raised-wheel diagnostics. NO automatic drive/servo.\n");
    tm_printf((UB*)"[CIRCUIT] calibration v%u: diameter=%u mm, B both-edges/rev L=%u R=%u; track=112 mm (pivot not qualified).\n",
        (unsigned)MOTION_CALIBRATION_VERSION,(unsigned)MOTION_WHEEL_DIAMETER_MM_CANDIDATE,
        (unsigned)MOTION_LEFT_BOTH_EDGE_COUNTS_PER_REV,(unsigned)MOTION_RIGHT_BOTH_EDGE_COUNTS_PER_REV);
    tm_printf((UB*)"[CIRCUIT] S3 init=%d running=%d; IMU init=%d running=%d; sonar init=%d\n",
        (int)s3_init,(int)s3_ready,(int)s4_init,(int)s4_ready,(int)sonar_init);
    tm_printf((UB*)"[CIRCUIT] Wheel power defaults 15%%; while holding GP21 STOP, tap GP20 to cycle 15/20/25/30%%. Release STOP, then fresh hold/release GP20 runs 250ms.\n");
    tm_printf((UB*)"[CIRCUIT] Cases0..5 wheels; 6=barcode cal; 7=sonar; 8=IMU cal; 9/10/11=servo 78/90/102 for 250ms.\n");
    tm_printf((UB*)"[CIRCUIT] Keep still/level for IMU calibration. Serial output only; use board buttons. h/w STOP latency UNMEASURED.\n");
    menu(selected);
    for (;;)
    {
        uint32_t now=b2_platform_now_us();
        bool stop=b2_platform_stop_pressed();
        bool start=b2_platform_start_pressed();
        bool busy;
        b2_ui_event_t ui;
        b2_platform_take_events(&events);
        b2_trial_service(&trial,events.reasons,events.cutoff_fired);
        if(circuit_controls_update(&controls,now,start,stop)) {
            tm_printf((UB*)"[CIRCUIT] power selected=%u%%; STOP held, NO motion. Release both; fresh START required.\n",
                (unsigned)(controls.effort/10U));
        }
        if(s4_ready) {
            uint32_t words[S4_WORDS];
            if(s4_shared_read(words)) {
                uint32_t seq=s4_get(words[S4_W_ATTITUDE],S4_AT_SEQ_SHIFT,S4_AT_SEQ_WIDTH);
                if(!imu_seen || seq!=imu_sequence) { imu_updated_us=now; imu_sequence=seq; imu_seen=true; }
            }
        }
        if (servo_active && (events.reasons!=STOP_REASON_NONE || events.cutoff_fired ||
            stop || now-servo_since>=PULSE_US))
        {
            servo_disable(); io.cancel_cutoff(io.context); servo_active=false;
            tm_printf((UB*)"[CIRCUIT] servo pulse ended; PWM OFF (actual angle has no feedback).\n");
        }
        if (b2_trial_take_result(&trial,&result))
        {
            uint64_t l_um=(uint64_t)result.left_edges*MOTION_WHEEL_DIAMETER_MM_CANDIDATE*3141593U /
                ((uint64_t)MOTION_LEFT_BOTH_EDGE_COUNTS_PER_REV*1000U);
            uint64_t r_um=(uint64_t)result.right_edges*MOTION_WHEEL_DIAMETER_MM_CANDIDATE*3141593U /
                ((uint64_t)MOTION_RIGHT_BOTH_EDGE_COUNTS_PER_REV*1000U);
            tm_printf((UB*)"[CIRCUIT] trial=%lu gen=%lu end=%u L=%lu R=%lu counts_valid=%d (2=time,3=STOP,4=failure)\n",
                (unsigned long)result.trial_id,(unsigned long)result.generation,(unsigned)result.state,
                (unsigned long)result.left_edges,(unsigned long)result.right_edges,(int)result.counts_valid);
            if (result.counts_valid)
            { tm_printf((UB*)"[CIRCUIT] estimated wheel travel L=%lu.%03lu R=%lu.%03lu mm (rotation magnitude, not floor position)\n",
                (unsigned long)(l_um/1000U),(unsigned long)(l_um%1000U),
                (unsigned long)(r_um/1000U),(unsigned long)(r_um%1000U)); }
            gesture.phase=0U; gesture.since=now;
        }
        if (stop) { busy_us=0U; }
        if (busy_us!=0U && now-busy_since>=busy_us) { busy_us=0U; }
        busy=servo_active || busy_us!=0U || trial.result.state==B2_TRIAL_RUNNING;
        ui=b2_gesture_update(&gesture,now,start,stop,busy);
        if (ui==B2_UI_SELECTED) { selected=(selected+1U)%CASES; menu(selected); }
        if (ui==B2_UI_CONFIRM)
        { tm_printf((UB*)"[CIRCUIT] RELEASE GP20 to run case=%u %s ONCE; GP21 cancels.\n",selected,names[selected]); }
        if (ui==B2_UI_RUN)
        {
            /* Only a fresh local gesture clears a released physical STOP.
             * Sensor/motor faults are never silently cleared by this action. */
            bool allowed=!io.stop_pressed(io.context) && !io.stop_pending(io.context) &&
                vehicle_controller_clear_stop(&controller,STOP_REASON_PHYSICAL);
            if (!allowed) { tm_printf((UB*)"[CIRCUIT] refused: STOP/fault remains active.\n"); }
            else if (selected<6U)
            {
                motion_status_t r=b2_trial_begin(&trial,
                    (int16_t)(left[selected]*(int16_t)controls.effort),
                    (int16_t)(right[selected]*(int16_t)controls.effort),PULSE_US);
                tm_printf((UB*)"[CIRCUIT] wheel request status=%u effort=%u%% duration=250ms\n",
                    (unsigned)r,(unsigned)(controls.effort/10U));
            }
            else if (selected==6U && s3_ready)
            { subsystem3_calibrate_barcode(); busy_since=now; busy_us=3000000U; }
            else if (selected==7U && sonar_ready)
            {
                range_reading_t r=drv_sonar_read(&sonar,1U,60U);
                tm_printf((UB*)"[CIRCUIT] sonar status=%d distance=%ld mm (valid only status=0)\n",
                    (int)r.status,(long)(r.cm*10.0f));
            }
            else if (selected==8U && s4_ready)
            {
                ER r=s4_request(S4_REQ_RECALIBRATE);
                tm_printf((UB*)"[CIRCUIT] IMU recalibration request=%d; keep still/level.\n",(int)r);
                busy_since=now; busy_us=3000000U;
            }
            else if (selected>=9U)
            {
                int angle=selected==9U ? 78 : (selected==10U ? 90 : 102);
                servo_active=servo_begin(angle); servo_since=now;
                tm_printf((UB*)"[CIRCUIT] servo request %d deg, 250ms, admitted=%d\n",angle,(int)servo_active);
            }
            else { tm_printf((UB*)"[CIRCUIT] component unavailable; see init/status.\n"); }
        }
        if (s3_ready)
        {
            route_instruction_t route;
            junction_observation_t junction;
            if (subsystem3_take_route(&route))
            { tm_printf((UB*)"[CIRCUIT] barcode '%c' route=%d (reported only, no driving)\n",route.decoded_character,(int)route.command); }
            if (subsystem3_take_junction(&junction))
            { tm_printf((UB*)"[CIRCUIT] junction candidate=%d confirmed=%d probe=%d (no autonomous pivot)\n",
                (int)junction.candidate,(int)junction.confirmed,(int)subsystem3_probe_requested()); }
        }
        if (now-last_print>=1000000U)
        {
            /* Never block on console reporting while driving a trial. */
            if (!servo_active && trial.result.state!=B2_TRIAL_RUNNING)
            { report(); menu(selected); last_print=now; }
        }
        (void)tk_dly_tsk(5);
    }
}

static ID task_id;
static _Alignas(8) uint32_t stack[1536];
ER circuit_test_start(void)
{
    T_CTSK ct={0};
    ER result;
    if (task_id!=0) { return E_OBJ; }
    ct.tskatr=TA_HLNG | TA_RNG0 | TA_ASSPRC | TA_USERBUF;
    ct.assprc=TP_PRC1; ct.task=(FP)circuit_main; ct.itskpri=8;
    ct.stksz=sizeof(stack); ct.bufptr=stack;
    task_id=tk_cre_tsk(&ct);
    if (task_id<E_OK) { result=task_id; task_id=0; return result; }
    result=tk_sta_tsk(task_id,0);
    if (result!=E_OK) { (void)tk_del_tsk(task_id); task_id=0; }
    return result;
}
#endif
