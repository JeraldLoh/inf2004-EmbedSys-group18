#include "fake_motion_hal.h"
#include "measured_motion_executor.h"
#include "../vehicle_controller/controller_motion.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    fake_motion_hal_t fake;
    drv_motor_t motor;
    drv_encoder_t encoder;
    measured_motion_executor_t executor;
    bool pending, race, watchdog_fail, armed;
    uint32_t acknowledged;
} fixture_t;
static bool pending(void *p) { return ((fixture_t *)p)->pending; }
static bool watchdog(void *p, uint32_t duration) {
    fixture_t *f = p;
    assert(duration == 100000U);
    f->armed = true;
    if (f->race) { assert(drv_motor_stop(&f->motor) == MOTION_OK); }
    return !f->watchdog_fail;
}
static void cancel_watchdog(void *p) { ((fixture_t *)p)->armed = false; }
static measured_motion_executor_config_t config(void) {
    measured_motion_executor_config_t c = {0};
    c.measurement = (wheel_measurement_config_t){65U, {1271U,1264U}, 100000U,200000U,20000U};
    c.pid[0] = (wheel_pid_config_t){2000U,1000U,0U,4000U,300U,80U,0U,0U};
    c.pid[1] = c.pid[0];
    c.period_us=20000U; c.watchdog_us=100000U;
    c.track_width_mm=112U; c.max_speed_mm_s=100U; c.acceleration_um_s2=100000U;
    c.reversal_us=100000U; c.settle_us=100000U; c.settle_speed_um_s=1000U;
    c.tolerance_um=1000U; c.overshoot_um=5000U;
    c.progress_timeout_us=500000U; c.startup_grace_us=500000U;
    c.goal_margin_ms=2000U; c.max_goal_mm=10000U;
    c.qualified=true; /* Synthetic test only, NOT hardware qualification. */
    return c;
}
static void init(fixture_t *f) {
    motion_hal_t hal;
    measured_motion_executor_config_t c = config();
    measured_motion_executor_io_t io = {f,pending,watchdog,cancel_watchdog};
    memset(f,0,sizeof(*f)); fake_motion_init(&f->fake);
    hal=fake_motion_hal(&f->fake);
    assert(drv_motor_init(&f->motor,&hal,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f->encoder,&hal)==MOTION_OK);
    assert(measured_motion_executor_init(&f->executor,&f->motor,&f->encoder,&c,&io));
}
static void tick(fixture_t *f, unsigned left, unsigned right) {
    unsigned i;
    f->fake.trace_count=0U;
    f->fake.now_us+=20000U;
    /* Real edge callback and coherent capture; no direct sample mutation. */
    for(i=0U;i<left;++i) { drv_encoder_on_edges(&f->encoder,1UL<<27,0U,f->fake.now_us); }
    for(i=0U;i<right;++i) { drv_encoder_on_edges(&f->encoder,1UL<<5,0U,f->fake.now_us); }
    (void)measured_motion_executor_step(&f->executor);
}
static motion_goal_request_t request(fixture_t *f, motion_goal_type_t type) {
    motion_goal_request_t r={0};
    r.goal_id=1U; r.authority_generation=f->executor.status.generation;
    r.source=MOTION_SOURCE_CALIBRATION; r.type=type; r.max_speed_mm_s=30U;
    if(type==MOTION_GOAL_FORWARD || type==MOTION_GOAL_REVERSE) { r.target_mm=100; }
    else { r.target_mdeg=90000; }
    return r;
}
static void start_goal(fixture_t *f,motion_goal_type_t type) {
    unsigned i;
    motion_goal_request_t r;
    assert(measured_motion_executor_authorize(&f->executor,1U)==MOTION_OK);
    r=request(f,type);
    assert(measured_motion_executor_goal(&f->executor,&r,f->fake.now_us,1000000U)==MOTION_OK);
    for(i=0U;i<7U;++i) { tick(f,0U,0U); }
    assert(f->executor.goal_started);
}
static void measurement(void) {
    wheel_measurement_t m;
    measured_motion_executor_config_t c=config();
    motion_encoder_snapshot_t s={0};
    unsigned i;
    assert(wheel_measurement_init(&m,&c.measurement));
    s.acquisition_valid=true; s.edge_times_valid=true; s.command_basis_valid=true;
    s.left_command_sign=1; s.right_command_sign=1;
    s.acquired_us=UINT32_MAX-50000U; s.left_count=UINT32_MAX-5U; s.right_count=UINT32_MAX-5U;
    assert(wheel_measurement_update(&m,&s)==MOTION_OK);
    for(i=0;i<100U;++i) {
        s.acquired_us+=20000U; ++s.left_count; ++s.right_count;
        assert(wheel_measurement_update(&m,&s)==MOTION_OK);
    }
    assert(m.sample.travel_um[0]==(100ULL*65U*3141593U)/(1271U*1000U));
    assert(m.sample.travel_um[1]==(100ULL*65U*3141593U)/(1264U*1000U));
    assert(m.sample.travel_um[1]>m.sample.travel_um[0]);
    assert(m.sample.speed_valid && m.sample.signed_speed_valid);
    ++s.direction_epoch; s.acquired_us+=20000U;
    assert(wheel_measurement_update(&m,&s)==MOTION_OK && !m.sample.signed_speed_valid);
    ++s.reset_epoch; s.acquired_us+=20000U;
    assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE && !m.sample.speed_valid);
    assert(wheel_measurement_init(&m,&c.measurement)); s.coalesced_edges_seen=true;
    assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE);
    s.coalesced_edges_seen=false; assert(wheel_measurement_update(&m,&s)==MOTION_OK);
    s.acquired_us+=200001U; assert(wheel_measurement_update(&m,&s)==MOTION_UNAVAILABLE);
}
static void pid(void) {
    wheel_pid_t p,q;
    wheel_pid_config_t c={1000U,1000U,0U,0U,100U,50U,0U,0U};
    uint16_t out,other;
    unsigned i;
    assert(wheel_pid_init(&p,&c)); assert(wheel_pid_init(&q,&c));
    assert(wheel_pid_step(&p,10000U,0U,100000U,&out) && out==11U);
    assert(wheel_pid_step(&q,10000U,0U,50000U,&other) && other==10U);
    for(i=0;i<1000U;++i) { assert(wheel_pid_step(&p,500000U,0U,20000U,&out)); }
    assert(out==100U && p.integral_um_us<=50000000000000LL);
    assert(wheel_pid_step(&p,0U,0U,20000U,&out) && out==0U && p.integral_um_us==0);
    c.kp_milli=0U; c.ki_milli=0U; c.launch_effort=70U; c.launch_us=40000U;
    assert(wheel_pid_init(&p,&c));
    assert(wheel_pid_step(&p,10000U,0U,20000U,&out) && out==70U);
    assert(wheel_pid_step(&p,10000U,0U,20000U,&out) && out==70U);
    assert(wheel_pid_step(&p,10000U,0U,20000U,&out) && out==0U);
    assert(!wheel_pid_step(&p,10000U,0U,0U,&out));
    c.launch_us=0U; c.launch_effort=0U; c.kd_milli=1000U; c.feedforward_milli=1000U;
    assert(wheel_pid_init(&p,&c));
    assert(wheel_pid_step(&p,10000U,5000U,20000U,&out) && out==10U);
    assert(wheel_pid_step(&p,20000U,5000U,20000U,&out) && out==20U); /* No setpoint D kick. */
    assert(wheel_pid_step(&p,20000U,6000U,20000U,&out) && out==0U);
}
static void admission(void) {
    fixture_t f;
    motion_goal_request_t r;
    init(&f); f.executor.config.qualified=false;
    assert(measured_motion_executor_authorize(&f.executor,1U)==MOTION_UNAVAILABLE);
    fake_assert_safe(&f.fake); f.executor.config.qualified=true;
    assert(measured_motion_executor_authorize(&f.executor,1U)==MOTION_OK); fake_assert_safe(&f.fake);
    r=request(&f,MOTION_GOAL_FORWARD); r.target_mm=1000; r.timeout_ms=5000U;
    assert(measured_motion_executor_goal(&f.executor,&r,0U,1000000U)==MOTION_INVALID);
    r.timeout_ms=0U;
    assert(measured_motion_executor_goal(&f.executor,&r,0U,1000000U)==MOTION_OK);
    assert(f.executor.goal.timeout_ms>33333U);
    assert(measured_motion_executor_goal(&f.executor,&r,0U,1000000U)==MOTION_STATE);
    measured_motion_executor_cancel(&f.executor,MOTION_RESULT_CANCELLED);
    assert(f.executor.status.terminal_pending);
    assert(!measured_motion_executor_ack(&f.executor,2U,1U));
    assert(measured_motion_executor_authorize(&f.executor,2U)==MOTION_STATE);
    assert(measured_motion_executor_ack(&f.executor,1U,1U));
    assert(measured_motion_executor_authorize(&f.executor,1U)==MOTION_STALE);
    assert(measured_motion_executor_authorize(&f.executor,2U)==MOTION_OK);
}
static void finite(motion_goal_type_t type, bool delayed) {
    fixture_t f;
    motion_goal_result_t result;
    uint64_t target;
    unsigned l,r,i;
    init(&f); start_goal(&f,type);
    assert(f.executor.status.target_um_s[0]*(type==MOTION_GOAL_REVERSE || type==MOTION_GOAL_PIVOT_LEFT?-1:1)>0);
    assert(f.executor.status.target_um_s[1]*(type==MOTION_GOAL_REVERSE || type==MOTION_GOAL_PIVOT_RIGHT?-1:1)>0);
    target=f.executor.goal_target_um;
    /* Counts represent 30 mm/s, then an exact target rounded to nearest edge. */
    l=(unsigned)((target*1271U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
    r=(unsigned)((target*1264U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
    while(l || r) {
        unsigned dl=l>4U?4U:l;
        unsigned dr=r>4U?4U:r;
        if(delayed && dr>3U) { dr=3U; }
        tick(&f,dl,dr); l-=dl; r-=dr;
        assert(!f.executor.status.terminal_pending);
    }
    for(i=0;i<20U && !f.executor.status.terminal_pending;++i) { tick(&f,0U,0U); }
    assert(measured_motion_executor_result(&f.executor,&result));
    assert(result.code==MOTION_RESULT_COMPLETED && result.odometry_valid);
    if(type==MOTION_GOAL_REVERSE) { assert(result.travelled_mm<=-99); }
    if(type==MOTION_GOAL_PIVOT_LEFT) { assert(result.turned_mdeg>89000); }
    if(type==MOTION_GOAL_PIVOT_RIGHT) { assert(result.turned_mdeg < -89000); }
    fake_assert_safe(&f.fake);
    assert(f.executor.status.authorized);
    measured_motion_executor_cancel(&f.executor,MOTION_RESULT_FAULT);
    assert(measured_motion_executor_result(&f.executor,&result) && result.code==MOTION_RESULT_COMPLETED);
    assert(measured_motion_executor_ack(&f.executor,1U,1U));
}
static void failure(const char *name) {
    fixture_t f;
    unsigned i;
    motion_goal_result_t result;
    init(&f); start_goal(&f,MOTION_GOAL_FORWARD);
    if(strcmp(name,"stop_race")==0) { f.race=true; tick(&f,0U,0U); }
    else if(strcmp(name,"overshoot")==0) {
        for(i=0;i<180U && !f.executor.status.terminal_pending;++i) { tick(&f,4U,4U); }
    } else { for(i=0;i<80U && !f.executor.status.terminal_pending;++i) { tick(&f,2U,0U); } }
    assert(measured_motion_executor_result(&f.executor,&result));
    assert(result.code!=MOTION_RESULT_COMPLETED && !f.executor.status.authorized);
    fake_assert_safe(&f.fake);
}
static void ttl(void) {
    fixture_t f;
    motion_continuous_request_t r={0};
    unsigned i;
    init(&f); assert(measured_motion_executor_authorize(&f.executor,1U)==MOTION_OK);
    r.command_id=1U; r.authority_generation=1U; r.speed_mm_s=30;
    assert(measured_motion_executor_continuous(&f.executor,&r,0U,400000U)==MOTION_OK);
    assert(measured_motion_executor_continuous(&f.executor,&r,0U,400000U)==MOTION_STALE);
    for(i=0;i<21U;++i) { tick(&f,0U,0U); }
    assert(!f.executor.status.authorized && f.executor.status.last_stop==MOTION_RESULT_TIMEOUT);
    fake_assert_safe(&f.fake);
}
static bool stop(void *p) { return drv_motor_stop(&((fixture_t *)p)->motor)==MOTION_OK; }
static motion_status_t authorize(void *p,uint32_t g) { return measured_motion_executor_authorize(&((fixture_t*)p)->executor,g); }
static motion_status_t continuous(void *p,const motion_continuous_request_t *r,uint32_t ttl_us) {
    fixture_t *f=p; return measured_motion_executor_continuous(&f->executor,r,f->fake.now_us,ttl_us);
}
static motion_status_t goal(void *p,const motion_goal_request_t *r,uint32_t ttl_us) {
    fixture_t *f=p; return measured_motion_executor_goal(&f->executor,r,f->fake.now_us,ttl_us);
}
static bool cancel(void *p) { measured_motion_executor_cancel(&((fixture_t*)p)->executor,MOTION_RESULT_CANCELLED); return true; }
static void acknowledge(void *p,uint32_t id) { ((fixture_t*)p)->acknowledged=id; }
static void controller(void) {
    fixture_t f;
    controller_service_t s;
    controller_motion_t m;
    controller_outputs_t o={.context=&f,.stop_motion=stop};
    controller_motion_outputs_t mo={&f,42U,authorize,continuous,goal,cancel,acknowledge};
    controller_event_t mode={CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_MANUAL,0U,MISSION_IDLE,false};
    controller_motion_request_t r={0};
    unsigned i;
    init(&f); assert(controller_service_init(&s,&o));
    assert(controller_service_handle(&s,&mode));
    assert(controller_motion_init(&m,&s,&mo,42U,15U,30U));
    r.kind=CONTROLLER_MOTION_START; r.source=MOTION_SOURCE_MANUAL;
    r.session=41U; r.id=1U; r.generation=s.controller.authority_generation;
    r.created_ms=0U; r.expires_ms=1000U;
    assert(!controller_motion_handle(&m,&r,0U));
    r.session=42U; assert(controller_motion_handle(&m,&r,0U)); fake_assert_safe(&f.fake);
    r.generation=s.controller.authority_generation; r.id=2U;
    r.kind=CONTROLLER_MOTION_PROFILE; r.value.profile=MOTION_PROFILE_CAUTION;
    assert(controller_motion_handle(&m,&r,0U)); fake_assert_safe(&f.fake);
    r.kind=CONTROLLER_MOTION_CONTINUOUS; r.id=3U; r.value.continuous.speed_mm_s=100;
    r.value.continuous.curvature_u1_m=0; r.value.continuous.cap_mm_s=0U;
    assert(controller_motion_handle(&m,&r,0U));
    assert(f.executor.continuous.speed_mm_s==15);
    assert(!controller_motion_handle(&m,&r,0U));
    r.source=MOTION_SOURCE_LINE; r.id=1U;
    assert(!controller_motion_handle(&m,&r,0U));
    r.source=MOTION_SOURCE_MANUAL; r.id=3U;
    for(i=0;i<8U;++i) { tick(&f,0U,0U); }
    controller_motion_stop_evidence(&m,STOP_REASON_REMOTE,77U);
    assert(!m.status.enabled && f.acknowledged==77U);
    assert(m.status.selected_profile==MOTION_PROFILE_CAUTION); fake_assert_safe(&f.fake);
    r.id=4U; assert(!controller_motion_handle(&m,&r,1U));
    r.generation=s.controller.authority_generation;
    assert(!controller_motion_handle(&m,&r,1000U));
}
static void result_adapter(void) {
    fixture_t f;
    controller_service_t s;
    controller_motion_t m;
    controller_outputs_t o={.context=&f,.stop_motion=stop};
    controller_motion_outputs_t mo={&f,42U,authorize,continuous,goal,cancel,acknowledge};
    controller_event_t mode={CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_MANUAL,0U,MISSION_IDLE,false};
    controller_motion_request_t r={0};
    motion_goal_result_t terminal;
    init(&f); assert(controller_service_init(&s,&o));
    assert(controller_service_handle(&s,&mode));
    assert(controller_motion_init(&m,&s,&mo,42U,15U,30U));
    r.kind=CONTROLLER_MOTION_START; r.source=MOTION_SOURCE_MANUAL;
    r.session=42U; r.id=1U; r.generation=s.controller.authority_generation;
    r.expires_ms=1000U;
    assert(controller_motion_handle(&m,&r,0U));
    r.kind=CONTROLLER_MOTION_GOAL; r.id=2U; r.generation=s.controller.authority_generation;
    r.value.goal.type=MOTION_GOAL_FORWARD; r.value.goal.mm=100;
    assert(controller_motion_handle(&m,&r,0U) && s.controller.active_goal);
    terminal=(motion_goal_result_t){2U,r.generation-1U,MOTION_RESULT_COMPLETED,100,0,true};
    controller_motion_result(&m,&terminal);
    assert(s.controller.active_goal); /* Old generation cannot finish new work. */
    measured_motion_executor_cancel(&f.executor,MOTION_RESULT_REJECTED);
    assert(measured_motion_executor_result(&f.executor,&terminal));
    controller_motion_result(&m,&terminal);
    assert(!s.controller.active_goal && m.status.last_terminal.code==MOTION_RESULT_REJECTED);
    fake_assert_safe(&f.fake);
}
int main(int argc,char **argv) {
    const char *name=argc>1?argv[1]:"";
    if(strcmp(name,"measurement")==0) { measurement(); }
    else if(strcmp(name,"pid")==0) { pid(); }
    else if(strcmp(name,"admission")==0) { admission(); }
    else if(strcmp(name,"forward")==0 || strcmp(name,"both_wheels")==0) { finite(MOTION_GOAL_FORWARD,strcmp(name,"both_wheels")==0); }
    else if(strcmp(name,"reverse")==0) { finite(MOTION_GOAL_REVERSE,false); }
    else if(strcmp(name,"left")==0) { finite(MOTION_GOAL_PIVOT_LEFT,false); }
    else if(strcmp(name,"right")==0) { finite(MOTION_GOAL_PIVOT_RIGHT,false); }
    else if(strcmp(name,"ttl")==0) { ttl(); }
    else if(strcmp(name,"controller")==0) { controller(); }
    else if(strcmp(name,"result_adapter")==0) { result_adapter(); }
    else { failure(name); }
    printf("PASS %s\n",name); return 0;
}

