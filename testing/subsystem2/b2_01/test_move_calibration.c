#include "fake_motion_hal.h"
#include "../diagnostics/b2_move_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { fake_motion_hal_t fake; drv_motor_t motor; drv_encoder_t encoder;
    measured_motion_executor_t e; bool pending,race; } fixture_t;
static bool pending(void *p) { return ((fixture_t*)p)->pending; }
static bool arm(void *p,uint32_t us) {
    fixture_t *f=p; assert(us==100000U);
    if(f->race) { assert(drv_motor_stop(&f->motor)==MOTION_OK); }
    return true;
}
static void cancel(void *p) { (void)p; }
static void init(fixture_t *f) {
    motion_hal_t hal; measured_motion_executor_config_t c;
    measured_motion_executor_io_t io={f,pending,arm,cancel};
    memset(f,0,sizeof(*f)); fake_motion_init(&f->fake); hal=fake_motion_hal(&f->fake);
    assert(drv_motor_init(&f->motor,&hal,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f->encoder,&hal)==MOTION_OK);
    b2_move_candidate_profile(&c); assert(!c.qualified);
    assert(measured_motion_executor_init(&f->e,&f->motor,&f->encoder,&c,&io));
    assert(measured_motion_executor_authorize(&f->e,1U)==MOTION_UNAVAILABLE);
}
static void tick(fixture_t *f,unsigned l,unsigned r) {
    unsigned i; f->fake.trace_count=0U; f->fake.now_us+=20000U;
    for(i=0;i<l;++i) { drv_encoder_on_edges(&f->encoder,1UL<<27,0U,f->fake.now_us); }
    for(i=0;i<r;++i) { drv_encoder_on_edges(&f->encoder,1UL<<5,0U,f->fake.now_us); }
    (void)measured_motion_executor_step(&f->e);
}
static void bounds(void) {
    fixture_t f; init(&f);
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,20000001U)==MOTION_INVALID);
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,0U)==MOTION_INVALID);
    f.e.config.max_speed_mm_s=121U;
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,1000000U)==MOTION_INVALID);
    f.e.config.max_speed_mm_s=120U; f.e.config.pid[0].max_effort=301U;
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,1000000U)==MOTION_INVALID);
    f.e.config.pid[0].max_effort=300U; f.e.config.max_goal_mm=1001U;
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,1000000U)==MOTION_INVALID);
    f.e.config.max_goal_mm=1000U; f.pending=true;
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,1000000U)==MOTION_STALE);
    fake_assert_safe(&f.fake);
}
static void deadline(bool wrap,bool race) {
    fixture_t f; motion_continuous_request_t r={1U,1U,MOTION_SOURCE_CALIBRATION,100,0,100U,0U,1000U};
    unsigned i; init(&f); if(wrap) { f.fake.now_us=UINT32_MAX-90000U; }
    f.race=race;
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,f.fake.now_us,300000U)==MOTION_OK);
    fake_assert_safe(&f.fake); assert(!f.e.config.qualified);
    assert(measured_motion_executor_continuous(&f.e,&r,f.fake.now_us,1000000U)==MOTION_OK);
    for(i=0;i<20U;++i) { tick(&f,0U,0U); }
    assert(!f.e.status.authorized && !f.e.calibration_authorized);
    assert(f.e.status.last_stop==(race?MOTION_RESULT_FAULT:MOTION_RESULT_TIMEOUT));
    fake_assert_safe(&f.fake);
    assert(measured_motion_executor_authorize(&f.e,2U)==MOTION_UNAVAILABLE);
}
static void goal(motion_goal_type_t type) {
    fixture_t f; motion_goal_request_t r={1U,1U,MOTION_SOURCE_CALIBRATION,type,0,0,100U,0U};
    motion_goal_result_t out; unsigned i,l=0U,right=0U; bool counted=false;
    init(&f);
    assert(measured_motion_executor_authorize_calibration(&f.e,1U,0U,15000000U)==MOTION_OK);
    if(type==MOTION_GOAL_FORWARD) { r.target_mm=1000; }
    else if(type==MOTION_GOAL_REVERSE) { r.target_mm=100; }
    else { r.target_mdeg=90000; }
    assert(measured_motion_executor_goal(&f.e,&r,0U,1000000U)==MOTION_OK);
    for(i=0;i<700U && !f.e.status.terminal_pending;++i) {
        unsigned dl,dr;
        if(f.e.goal_started && !counted) {
            uint64_t target=f.e.goal_target_um;
            l=(unsigned)((target*1271U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
            right=(unsigned)((target*1264U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
            counted=true;
        }
        dl=l>13U?13U:l; dr=right>13U?13U:right;
        tick(&f,dl,dr); l-=dl; right-=dr;
    }
    assert(measured_motion_executor_result(&f.e,&out) && out.code==MOTION_RESULT_COMPLETED);
    assert(!f.e.config.qualified);
    if(type==MOTION_GOAL_FORWARD) { assert(out.travelled_mm>=998 && out.travelled_mm<=1001); }
    if(type==MOTION_GOAL_REVERSE) { assert(out.travelled_mm<=-98); }
    if(type==MOTION_GOAL_PIVOT_LEFT) { assert(out.turned_mdeg>89000); }
    if(type==MOTION_GOAL_PIVOT_RIGHT) { assert(out.turned_mdeg<-89000); }
    measured_motion_executor_cancel(&f.e,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.fake);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(strcmp(argv[1],"bounds")==0) { bounds(); }
    else if(strcmp(argv[1],"deadline")==0) { deadline(false,false); }
    else if(strcmp(argv[1],"wrap")==0) { deadline(true,false); }
    else if(strcmp(argv[1],"stop_race")==0) { deadline(false,true); }
    else if(strcmp(argv[1],"forward_1m")==0) { goal(MOTION_GOAL_FORWARD); }
    else if(strcmp(argv[1],"reverse")==0) { goal(MOTION_GOAL_REVERSE); }
    else if(strcmp(argv[1],"left")==0) { goal(MOTION_GOAL_PIVOT_LEFT); }
    else if(strcmp(argv[1],"right")==0) { goal(MOTION_GOAL_PIVOT_RIGHT); }
    else { assert(false); }
    printf("PASS bounded tuning authority %s\n",argv[1]); return 0;
}
