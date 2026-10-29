/* Actual movement-test task with real HAL-backed drivers and executor.
 * Scripted operator gestures and raw wheel feedback, not physical evidence. */
#include "fake_motion_hal.h"
#include "../diagnostics/b2_move_test.h"
#include "subsystem2_internal.h"
#include "b2_platform_rp2040.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
static fake_motion_hal_t fake;
static motion_hal_t hal;
static FP entry;
static jmp_buf boundary;
static const char *scenario;
static unsigned raw_arms,cal_arms,deleted;
static bool armed,pressed,did_raw_stop,did_cal_stop,saw_power,raw_mode;
static uint32_t deadline,window,raw_since,release_stop;
static b2_platform_events_t evidence;
static char output[65536];
static bool is(const char *s) { return strcmp(scenario,s)==0; }
static void stop_event(bool cutoff) {
    (void)subsystem2_urgent_stop(); armed=false; ++evidence.stop_events;
    evidence.cutoff_fired=cutoff; evidence.reasons=cutoff?0U:STOP_REASON_PHYSICAL;
    if(!cutoff) { pressed=true; release_stop=fake.now_us+50000U; }
}
static bool stop(void *p) { (void)p; return pressed; }
static bool pending(void *p) { (void)p; return pressed || evidence.cutoff_fired || evidence.reasons!=0U; }
static bool arm(void *p,uint32_t us) {
    (void)p; assert(us>0U && us<=100000U);
    if(!armed) {
        raw_mode=raw_arms<3U;
        if(raw_mode) { ++raw_arms; raw_since=fake.now_us; }
    }
    if(!raw_mode) { ++cal_arms; }
    window=us; deadline=fake.now_us+us; armed=true;
    return true;
}
static void cancel(void *p) { (void)p; armed=false; }
bool b2_platform_init(motion_hal_t *h,b2_trial_io_t *io) {
    if(is("critical")) { return false; }
    *h=hal; *io=(b2_trial_io_t){NULL,stop,pending,arm,cancel}; return true;
}
uint32_t b2_platform_now_us(void) { return fake.now_us; }
bool b2_platform_stop_pressed(void) { return pressed; }
static bool between(uint32_t a,uint32_t b) {
    if(!is("locked")) {
        uint32_t shift=a>=4000000U?9200000U:(a>=2000000U?4600000U:0U);
        a+=shift; b+=shift;
    }
    return fake.now_us>=a && fake.now_us<b;
}
bool b2_platform_start_pressed(void) {
    if(is("boot")) { return false; }
    if(is("locked")) {
        return between(300000U,350000U)||between(500000U,550000U)||between(700000U,750000U)||
            between(900000U,950000U)||between(1200000U,2300000U);
    }
    if(is("watchdog")) {
        return between(200000U,1300000U)||between(2000000U,2050000U)||between(2200000U,3300000U)||
            between(4000000U,4050000U)||between(4200000U,5300000U)||between(6000000U,6050000U)||
            between(6200000U,6250000U)||between(6400000U,6450000U)||between(6600000U,6650000U)||
            between(6800000U,6850000U)||between(7000000U,7050000U)||between(7200000U,7250000U)||
            between(7500000U,8600000U);
    }
    return between(200000U,1300000U)||between(2000000U,2050000U)||between(2200000U,3300000U)||
        between(4000000U,4050000U)||between(4200000U,5300000U)||between(6000000U,6050000U)||
        (!is("flow_time")&&between(6200000U,6250000U))||(is("long_locked")&&between(6400000U,6450000U))||between(6500000U,7600000U);
}
void b2_platform_take_events(b2_platform_events_t *e) {
    *e=evidence; evidence.reasons=0U; evidence.cutoff_fired=false;
}
ID tk_cre_tsk(const T_CTSK *c) {
    assert(c->assprc==TP_PRC1 && c->stksz==6144 && c->bufptr!=NULL && c->itskpri==6);
    entry=c->task; return is("create")?E_SYS:7;
}
ER tk_sta_tsk(ID id,INT code) { (void)code; assert(id==7); return is("start")?E_SYS:E_OK; }
ER tk_del_tsk(ID id) { assert(id==7); ++deleted; return E_OK; }
void tk_ext_tsk(void) { longjmp(boundary,1); }
ER tk_get_otm(SYSTIM *t) { t->hi=0U; t->lo=fake.now_us/1000U; return E_OK; }
ER tk_dly_tsk(UINT ms) {
    bool lm=fake.enabled[4]&&(fake.compare[8]!=0U||fake.compare[9]!=0U);
    bool rm=fake.enabled[5]&&(fake.compare[10]!=0U||fake.compare[11]!=0U);
    assert(ms==5U || (is("watchdog") && ms==150U)); saw_power|=lm||rm; fake.trace_count=0U;
    if(raw_mode || is("flow_time")) {
        if(lm && !is("missing_feedback")) { drv_encoder_on_edges(subsystem2_bench_encoder(),1UL<<27,0U,fake.now_us); }
        if(rm) { drv_encoder_on_edges(subsystem2_bench_encoder(),1UL<<5,0U,fake.now_us); }
    }
    fake.now_us+=ms*1000U;
    if(pressed && fake.now_us>=release_stop) { pressed=false; }
    if(raw_arms==3U && armed && raw_mode && fake.now_us-raw_since>=100000U && !did_raw_stop)
    { did_raw_stop=true; stop_event(false); }
    if(is("flow_stop") && !raw_mode && (lm||rm) && !did_cal_stop)
    { did_cal_stop=true; stop_event(false); }
    if(armed && fake.now_us-deadline<0x80000000UL) { stop_event(true); }
    if(fake.now_us>24000000U) { longjmp(boundary,1); }
    return E_OK;
}
void tm_printf(UB *fmt,...) {
    va_list args; size_t n=strlen(output);
    va_start(args,fmt); (void)vsnprintf(output+n,sizeof(output)-n,(char*)fmt,args); va_end(args);
}
int main(int argc,char **argv) {
    assert(argc==2); scenario=argv[1]; fake_motion_init(&fake); hal=fake_motion_hal(&fake);
    if(is("create") || is("start")) {
        assert(b2_move_test_start()==E_SYS && deleted==(is("start")?1U:0U));
    } else {
        assert(b2_move_test_start()==E_OK && b2_move_test_start()==E_OBJ);
        if(setjmp(boundary)==0) { ((void (*)(INT,void *))entry)(0,NULL); }
        if(!is("critical")) { fake_assert_safe(&fake); }
        else { assert(!fake.enabled[4] && !fake.enabled[5]); }
        if(is("boot") || is("critical")) { assert(!saw_power && raw_arms==0U && cal_arms==0U); }
        else if(is("locked")) { assert(!saw_power && strstr(output,"LOCKED")!=NULL); }
        else {
            assert(raw_arms==3U && did_raw_stop);
            if(is("missing_feedback") || is("long_locked")) { assert(cal_arms==0U && strstr(output,"LOCKED")!=NULL); }
            else if(is("watchdog")) { assert(cal_arms==1U && strstr(output,"WATCHDOG cutoff_observed=1")!=NULL); }
            else {
                assert(cal_arms>0U && strstr(output,"production_qualified=0")!=NULL);
                assert(strstr(output,"CALIBRATION request=0")!=NULL);
                if(is("flow_stop")) { assert(did_cal_stop && strstr(output,"code=4")!=NULL); }
                if(is("flow_stall")) { assert(strstr(output,"code=7")!=NULL); }
                if(is("flow_time")) { assert(strstr(output,"timed straight test ended (5s / 500mm")!=NULL); }
            }
        }
    }
    printf("PASS movement-test task %s\n",scenario); return 0;
}
