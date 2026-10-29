/* Actual diagnostic loop + real driver/trial/servo code with fake HAL and
 * bounded peripherals. This is software evidence, not a hardware test. */
#include "fake_motion_hal.h"
#include "../diagnostics/circuit_test.h"
#include "subsystem2_internal.h"
#include "b2_platform_rp2040.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/s4_imu.h"
#include "../subsystem5/drv_sonar.h"
#include <sys/sysdef.h>
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern fake_motion_hal_t *bsp_fake;
static fake_motion_hal_t fake;
static motion_hal_t hal;
static FP entry;
static jmp_buf boundary;
static const char *scenario;
static unsigned step,selected,arm_calls,pings,barcode_cal,imu_cal,servo_setup;
static unsigned s3_starts,s4_starts,deleted;
static uint32_t deadline;
static bool armed,pressed,saw_motor,saw_servo,stopped;
static bool saw_left,saw_right;
static unsigned power_steps;
static unsigned prelude(void) { return power_steps==0U?0U:4U+power_steps*4U; }
static int effort(void) { return power_steps==4U?150:(int)(150U+power_steps*50U); }
static void (*aux)(void *);
static void *aux_user;
static b2_platform_events_t evidence;
static gpio_dispatcher_t dispatcher;
static char output[32768];
static bool is(const char *s) { return strcmp(scenario,s)==0; }
static void stop_event(bool cutoff) {
    motion_lock_key_t key=hal.enter(hal.context);
    if(aux!=NULL) { aux(aux_user); }
    hal.leave(hal.context,key);
    (void)subsystem2_urgent_stop();
    evidence.cutoff_fired=cutoff;
    evidence.reasons=cutoff?STOP_REASON_NONE:STOP_REASON_PHYSICAL;
    ++evidence.stop_events; armed=false;
    if(!cutoff) { pressed=true; }
}
static bool stop_pressed(void *p) { (void)p; return pressed; }
static bool pending(void *p) { (void)p; return pressed || evidence.cutoff_fired || evidence.reasons!=0U; }
static bool arm(void *p,uint32_t us) {
    (void)p; assert(us==250000U); ++arm_calls;
    armed=true; deadline=fake.now_us+us;
    if(is("late_stop")) { stop_event(false); }
    return true;
}
static void cancel(void *p) { (void)p; armed=false; }
bool b2_platform_init(motion_hal_t *h,b2_trial_io_t *io) {
    if(is("critical_failure")) { return false; }
    *h=hal; *io=(b2_trial_io_t){NULL,stop_pressed,pending,arm,cancel}; return true;
}
bool b2_platform_bind_aux_stop(void (*callback)(void *),void *user) {
    assert(aux==NULL); aux=callback; aux_user=user; return true;
}
gpio_dispatcher_t *b2_platform_dispatcher(void) { return &dispatcher; }
uint32_t b2_platform_now_us(void) { return fake.now_us; }
bool b2_platform_stop_pressed(void) { return pressed; }
bool b2_platform_start_pressed(void) {
    unsigned base=prelude();
    unsigned hold=base+3U+selected*4U;
    if(is("boot") || is("sensor_failure") || is("imu_stale")) { return false; }
    if(step<base) { return step>=3U && step<3U+power_steps*4U && (step-3U)%4U==0U; }
    if(is("power_no_run")) { return false; }
    if(step>=base+3U && step<hold) { return (step-base-3U)%4U==0U; }
    return step>=hold && step<hold+22U;
}
void b2_platform_take_events(b2_platform_events_t *e) {
    *e=evidence; evidence.reasons=0U; evidence.cutoff_fired=false;
}
ID tk_cre_tsk(const T_CTSK *c) {
    assert(c->assprc==TP_PRC1 && c->stksz==6144 && c->bufptr!=NULL && c->itskpri==8);
    entry=c->task; return is("create_failure")?E_SYS:7;
}
ER tk_sta_tsk(ID id,INT code) { (void)code; assert(id==7); return is("start_failure")?E_SYS:E_OK; }
ER tk_del_tsk(ID id) { assert(id==7); ++deleted; return E_OK; }
void tk_ext_tsk(void) { longjmp(boundary,1); }
ER tk_dly_tsk(UINT us) {
    bool lm=fake.enabled[4] && (fake.compare[8]!=0U || fake.compare[9]!=0U);
    bool rm=fake.enabled[5] && (fake.compare[10]!=0U || fake.compare[11]!=0U);
    bool motor=lm || rm;
    bool servo=fake.enabled[6] && fake.compare[12]!=0U;
    assert(us==5U); saw_motor|=motor; saw_servo|=servo;
    saw_left|=lm; saw_right|=rm;
    if(motor) {
        motion_motor_state_t state;
        assert(subsystem2_read_motor(&state)==MOTION_OK);
        if(selected==0U || selected==4U) { assert(state.left_effort==effort()); }
        if(selected==1U || selected==5U) { assert(state.left_effort==-effort()); }
        if(selected==2U || selected==4U) { assert(state.right_effort==effort()); }
        if(selected==3U || selected==5U) { assert(state.right_effort==-effort()); }
    }
    fake.trace_count=0U;
    if(lm) { drv_encoder_on_edges(subsystem2_bench_encoder(),1UL<<27,0U,fake.now_us); }
    if(rm) { drv_encoder_on_edges(subsystem2_bench_encoder(),1UL<<5,0U,fake.now_us); }
    fake.now_us+=50000U; ++step;
    if(power_steps!=0U && !stopped) { pressed=step<prelude(); }
    if(is("servo_stop") && servo && !stopped) { stopped=true; stop_event(false); }
    if((is("motor_stop") || is("power30_stop")) && motor && !stopped) { stopped=true; stop_event(false); }
    if(armed && fake.now_us-deadline<0x80000000UL) { stop_event(true); }
    if(step>prelude()+3U+selected*4U+34U) { longjmp(boundary,1); }
    return E_OK;
}
void tm_printf(UB *fmt,...) {
    va_list args; size_t n=strlen(output);
    if(strstr((char *)fmt,"power selected=")!=NULL) { assert(pressed && !armed && !saw_motor && !saw_servo); }
    va_start(args,fmt); (void)vsnprintf(output+n,sizeof(output)-n,(char *)fmt,args); va_end(args);
}
void out_w(UW address,UW value) {
    assert(fake.locked && address==PWM_BASE+PWM_CHx_DIV+0x14U*6U && value==(125U<<4));
    ++servo_setup;
}
ER subsystem3_init(void) { assert(subsystem2_bench_motor()==NULL); return is("sensor_failure")?E_SYS:E_OK; }
ER subsystem3_start(void) { assert(subsystem2_bench_motor()!=NULL); ++s3_starts; return E_OK; }
line_status_t subsystem3_line_status(void) { line_status_t s={0}; s.left_black=1U; return s; }
bool subsystem3_barcode_raw(uint16_t *raw) { *raw=1234U; return true; }
BOOL subsystem3_barcode_calibrated(void) { return barcode_cal!=0U; }
void subsystem3_calibrate_barcode(void) { ++barcode_cal; }
BOOL subsystem3_take_route(route_instruction_t *r) { (void)r; return FALSE; }
BOOL subsystem3_take_junction(junction_observation_t *j) { (void)j; return FALSE; }
BOOL subsystem3_probe_requested(void) { return FALSE; }
ER s4_imu_init(void) { assert(subsystem2_bench_motor()==NULL); return is("sensor_failure")?E_SYS:E_OK; }
ER s4_imu_start(void) { ++s4_starts; return E_OK; }
ER s4_request(UINT bits) { assert(bits==S4_REQ_RECALIBRATE); ++imu_cal; return E_OK; }
UW s4_sensor_failures(void) { return 0U; }
bool s4_shared_read(uint32_t words[S4_WORDS]) {
    memset(words,0,S4_WORDS*sizeof(*words)); words[0]=1UL<<S4_ST_VALID_SHIFT;
    words[S4_W_ATTITUDE]=is("imu_stale")?0U:((fake.now_us/20000U)%32U)<<S4_AT_SEQ_SHIFT;
    return true;
}
ER drv_sonar_init(drv_sonar_t *s,gpio_dispatcher_t *d,UINT trig,UINT echo) {
    (void)s; assert(d==&dispatcher && trig==16U && echo==17U);
    assert(subsystem2_bench_motor()==NULL); return is("sensor_failure")?E_SYS:E_OK;
}
range_reading_t drv_sonar_read(drv_sonar_t *s,size_t count,UINT spacing) {
    (void)s; assert(count==1U && spacing==60U && !armed && !saw_motor && !saw_servo);
    ++pings; return (range_reading_t){RANGE_OK,12.3f};
}
int main(int argc,char **argv) {
    assert(argc==2); scenario=argv[1];
    fake_motion_init(&fake); bsp_fake=&fake; hal=fake_motion_hal(&fake);
    if(is("power20") || is("power_no_run")) { power_steps=1U; }
    if(is("power25")) { power_steps=2U; }
    if(is("power30") || is("power30_stop")) { power_steps=3U; }
    if(is("power_wrap")) { power_steps=4U; }
    pressed=power_steps!=0U;
    if(is("left_reverse")) { selected=1U; }
    if(is("right")) { selected=2U; }
    if(is("right_reverse")) { selected=3U; }
    if(is("both")) { selected=4U; }
    if(is("both_reverse")) { selected=5U; }
    if(is("barcode")) { selected=6U; }
    if(is("sonar")) { selected=7U; }
    if(is("imu")) { selected=8U; }
    if(is("servo") || is("servo_stop") || is("late_stop")) { selected=9U; }
    if(is("create_failure") || is("start_failure")) {
        assert(circuit_test_start()==E_SYS);
        assert(deleted==(is("start_failure")?1U:0U));
    } else {
        assert(circuit_test_start()==E_OK && circuit_test_start()==E_OBJ);
        if(setjmp(boundary)==0) { ((void (*)(INT,void *))entry)(0,NULL); }
        if(is("critical_failure")) { assert(strstr(output,"CRITICAL")!=NULL && arm_calls==0U); }
        else {
            assert(s3_starts==(is("sensor_failure")?0U:1U));
            assert(s4_starts==(is("sensor_failure")?0U:1U));
            assert(strstr(output,"diameter=65 mm")!=NULL && strstr(output,"L=1271 R=1264")!=NULL);
            if(selected<6U && !is("boot") && !is("sensor_failure") && !is("power_no_run") && !is("imu_stale")) {
                assert(saw_motor && arm_calls==1U && strstr(output,"estimated wheel travel")!=NULL);
                assert(saw_left==(selected==0U || selected==1U || selected>=4U));
                assert(saw_right==(selected>=2U));
                if(is("motor_stop") || is("power30_stop")) { assert(strstr(output,"end=3")!=NULL); }
            }
            else { assert(!saw_motor); }
            if(is("servo") || is("servo_stop")) { assert(saw_servo && servo_setup==1U && !fake.enabled[6]); }
            else { assert(!saw_servo); }
            if(is("late_stop")) { assert(arm_calls==1U && servo_setup==0U); }
            if(is("boot") || is("sensor_failure") || is("power_no_run") || is("imu_stale"))
            { assert(arm_calls==0U && servo_setup==0U); }
            if(power_steps!=0U) { assert(strstr(output,"power selected=")!=NULL); }
            if(is("imu_stale")) { assert(strstr(output,"fresh=0")!=NULL); }
            assert(barcode_cal==(is("barcode")?1U:0U));
            assert(pings==(is("sonar")?1U:0U));
            assert(imu_cal==(is("imu")?1U:0U));
            fake_assert_safe(&fake); assert(!fake.enabled[6]);
        }
    }
    printf("PASS circuit diagnostic %s\n",scenario); return 0;
}
