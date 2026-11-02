/* Deadband/coast plant regression. Synthetic dynamics are not car calibration. */
#include "fake_motion_hal.h"
#include "../diagnostics/b2_move_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { fake_motion_hal_t fake; drv_motor_t motor; drv_encoder_t encoder;
    measured_motion_executor_t e; uint32_t speed[2],edges[2]; uint64_t travel[2]; } fixture_t;
static bool pending(void *p) { (void)p; return false; }
static bool arm(void *p,uint32_t us) { (void)p; assert(us==100000U); return true; }
static void cancel(void *p) { (void)p; }
static void init(fixture_t *f,bool legacy) {
    motion_hal_t h; measured_motion_executor_config_t c;
    measured_motion_executor_io_t io={f,pending,arm,cancel};
    memset(f,0,sizeof(*f)); fake_motion_init(&f->fake); h=fake_motion_hal(&f->fake);
    assert(drv_motor_init(&f->motor,&h,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f->encoder,&h)==MOTION_OK); b2_move_candidate_profile(&c);
    if(legacy) { c.linear_goal_min_speed_mm_s=0U; c.tolerance_um=2000U; }
    assert(measured_motion_executor_init(&f->e,&f->motor,&f->encoder,&c,&io));
    assert(measured_motion_executor_authorize_calibration(&f->e,1U,0U,10000000U)==MOTION_OK);
}
static void plant(fixture_t *f,bool blocked,uint32_t deceleration) {
    motion_motor_state_t m; unsigned i;
    f->fake.trace_count=0U; assert(drv_motor_read(&f->motor,&m)==MOTION_OK);
    f->fake.now_us+=20000U;
    for(i=0U;i<2U;++i) {
        int effort=i==0U?m.left_effort:m.right_effort;
        unsigned duty=(unsigned)(effort<0?-effort:effort);
        uint32_t threshold=i==0U?120U:170U;
        uint32_t wanted=!blocked && duty>threshold?(duty-threshold)*1000U:0U;
        if(duty==0U || blocked) {
            uint32_t step=deceleration/50U;
            f->speed[i]=f->speed[i]>step?f->speed[i]-step:0U;
        } else {
            f->speed[i]=(3U*f->speed[i]+wanted)/4U;
            if(f->speed[i]<500U && wanted==0U) { f->speed[i]=0U; }
        }
        f->travel[i]+=f->speed[i]/50U;
        {
            uint32_t edges=(uint32_t)(f->travel[i]*(i==0U?1271U:1264U)*1000U/(65ULL*3141593U));
            while(f->edges[i]<edges) {
                ++f->edges[i]; drv_encoder_on_edges(&f->encoder,1UL<<(i==0U?27:5),0U,f->fake.now_us);
            }
        }
    }
    (void)measured_motion_executor_step(&f->e);
}
static void run(bool legacy,bool blocked,bool reverse,uint32_t coast) {
    fixture_t f; motion_goal_request_t r={1U,1U,MOTION_SOURCE_CALIBRATION,MOTION_GOAL_FORWARD,300,0,100U,0U};
    motion_goal_result_t out; unsigned i; init(&f,legacy);
    if(reverse) { r.type=MOTION_GOAL_REVERSE; }
    assert(measured_motion_executor_goal(&f.e,&r,0U,1000000U)==MOTION_OK);
    for(i=0U;i<600U && !f.e.status.terminal_pending;++i) {
        plant(&f,blocked,coast);
        if(!legacy && !blocked && f.e.goal_started && f.travel[0]>260000U &&
            f.travel[0]+f.e.config.tolerance_um<f.e.goal_target_um) {
            assert(f.e.status.target_um_s[0]>=60000 || reverse);
        }
    }
    assert(measured_motion_executor_result(&f.e,&out));
    printf("result code=%u mm=%ld physical plant L=%llu R=%llu um\n",(unsigned)out.code,(long)out.travelled_mm,
        (unsigned long long)f.travel[0],(unsigned long long)f.travel[1]);
    fflush(stdout);
    if(legacy) {
        assert(out.code==MOTION_RESULT_FAULT || out.code==MOTION_RESULT_TIMEOUT);
        assert(out.travelled_mm<298);
    }
    else if(blocked) { assert(out.code==MOTION_RESULT_FAULT); }
    else {
        assert(out.code==MOTION_RESULT_COMPLETED && out.odometry_valid);
        assert(f.travel[0]>=295000U && f.travel[0]<=305500U);
        assert(f.travel[1]>=295000U && f.travel[1]<=305500U);
        assert(!reverse || out.travelled_mm<0);
    }
    measured_motion_executor_cancel(&f.e,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.fake);
}
static void cap(bool pivot_goal) {
    fixture_t f; motion_goal_request_t r={1U,1U,MOTION_SOURCE_CALIBRATION,MOTION_GOAL_FORWARD,300,0,40U,0U};
    unsigned i; init(&f,false);
    if(pivot_goal) { r.type=MOTION_GOAL_PIVOT_LEFT; r.target_mm=0; r.target_mdeg=90000; }
    assert(measured_motion_executor_goal(&f.e,&r,0U,1000000U)==MOTION_OK);
    for(i=0U;i<40U;++i) {
        plant(&f,false,500000U);
        assert(f.e.status.target_um_s[0]>=-40000 && f.e.status.target_um_s[0]<=40000 &&
            f.e.status.target_um_s[1]>=-40000 && f.e.status.target_um_s[1]<=40000);
    }
    measured_motion_executor_cancel(&f.e,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.fake);
}
static void pivot(bool right,bool legacy) {
    fixture_t f; motion_goal_request_t r={1U,1U,MOTION_SOURCE_CALIBRATION,MOTION_GOAL_PIVOT_LEFT,0,90000,100U,0U};
    motion_goal_result_t out; unsigned i; init(&f,false);
    if(legacy) { f.e.config.pivot_goal_min_speed_mm_s=0U; }
    if(right) { r.type=MOTION_GOAL_PIVOT_RIGHT; }
    assert(measured_motion_executor_goal(&f.e,&r,0U,1000000U)==MOTION_OK);
    for(i=0U;i<600U && !f.e.status.terminal_pending;++i) { plant(&f,false,300000U); }
    assert(measured_motion_executor_result(&f.e,&out));
    printf("pivot result code=%u mdeg=%ld L=%llu R=%llu target=%llu um\n",(unsigned)out.code,(long)out.turned_mdeg,
        (unsigned long long)f.travel[0],(unsigned long long)f.travel[1],(unsigned long long)f.e.goal_target_um); fflush(stdout);
    if(legacy) { assert(out.code!=MOTION_RESULT_COMPLETED); }
    else {
        assert(out.code==MOTION_RESULT_COMPLETED && out.odometry_valid);
        assert(right?(out.turned_mdeg<=-80000 && out.turned_mdeg>=-100000):
            (out.turned_mdeg>=80000 && out.turned_mdeg<=100000));
    }
    measured_motion_executor_cancel(&f.e,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.fake);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(strcmp(argv[1],"pivot_left")==0) { pivot(false,false); }
    else if(strcmp(argv[1],"pivot_right")==0) { pivot(true,false); }
    else if(strcmp(argv[1],"pivot_legacy")==0) { pivot(false,true); }
    else if(strcmp(argv[1],"legacy_shortfall")==0) { run(true,false,false,300000U); }
    else if(strcmp(argv[1],"forward")==0) { run(false,false,false,300000U); }
    else if(strcmp(argv[1],"reverse")==0) { run(false,false,true,300000U); }
    else if(strcmp(argv[1],"coast")==0) { run(false,false,false,200000U); }
    else if(strcmp(argv[1],"blocked")==0) { run(false,true,false,300000U); }
    else if(strcmp(argv[1],"cap")==0) { cap(false); }
    else if(strcmp(argv[1],"pivot_cap")==0) { cap(true); }
    else { assert(false); }
    printf("PASS distance approach %s\n",argv[1]); return 0;
}
