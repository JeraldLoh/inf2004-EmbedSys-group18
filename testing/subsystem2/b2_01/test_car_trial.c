/* Explicit integrated trial is bounded and never flips production-qualified. */
#include "fake_motion_hal.h"
#include "car_motion.h"
#include "../diagnostics/b2_move_profile.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
typedef struct { fake_motion_hal_t f; drv_motor_t motor; drv_encoder_t encoder; car_motion_t car; bool stop; } fixture_t;
static bool pending(void *p) { return ((fixture_t*)p)->stop; }
static bool arm(void *p,uint32_t us) { (void)p; assert(us==100000U); return true; }
static void cancel(void *p) { (void)p; }
static void init(fixture_t *f) {
    motion_hal_t h; measured_motion_executor_config_t c;
    measured_motion_executor_io_t io={f,pending,arm,cancel};
    memset(f,0,sizeof(*f)); fake_motion_init(&f->f); h=fake_motion_hal(&f->f);
    assert(drv_motor_init(&f->motor,&h,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f->encoder,&h)==MOTION_OK); b2_move_candidate_profile(&c);
    assert(!c.qualified && c.track_width_mm==112U && c.max_speed_mm_s==120U);
    assert(c.pid[0].feedforward_milli==2163U && c.pid[1].feedforward_milli==2689U);
    assert(c.linear_goal_min_speed_mm_s==60U && c.pivot_goal_min_speed_mm_s==60U);
    assert(c.pid[0].max_effort==300U && c.pid[1].max_effort==300U);
    assert(car_motion_init(&f->car,&f->motor,&f->encoder,&c,&io));
}
static void tick(fixture_t *f,unsigned l,unsigned r) {
    motion_motor_state_t motor;
    unsigned i; f->f.trace_count=0; f->f.now_us+=20000U;
    assert(drv_motor_read(&f->motor,&motor)==MOTION_OK);
    if(motor.left_effort==0) { l=0U; }
    if(motor.right_effort==0) { r=0U; }
    for(i=0;i<l;++i) { drv_encoder_on_edges(&f->encoder,1UL<<27,0U,f->f.now_us); }
    for(i=0;i<r;++i) { drv_encoder_on_edges(&f->encoder,1UL<<5,0U,f->f.now_us); }
    car_motion_step(&f->car);
}
static bool line(fixture_t *f,uint32_t id) {
    uint32_t ms=f->f.now_us/1000U;
    motion_continuous_request_t r={id,1U,MOTION_SOURCE_LINE,100,0,100U,ms,ms+150U};
    return car_motion_continuous(&f->car,&r,ms,f->f.now_us);
}
int main(int argc,char **argv) {
    fixture_t f; unsigned i; assert(argc==2); init(&f);
    if(strcmp(argv[1],"normal")==0) {
        assert(!line(&f,1U) && !f.car.executor.status.authorized); fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"bounds")==0) {
        assert(!car_motion_enable_trial(&f.car,0U));
        assert(!car_motion_enable_trial(&f.car,15000001U));
        assert(car_motion_enable_trial(&f.car,15000000U)); fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"pivot_cap")==0 || strcmp(argv[1],"forward_cap")==0) {
        /* Wheels held still: pivots and straight goals both stay at 30%
         * (pivot_max_effort is 0 in the trial profile). */
        bool pivot=strcmp(argv[1],"pivot_cap")==0;
        motion_goal_request_t r={23U,1U,MOTION_SOURCE_JUNCTION,
            pivot?MOTION_GOAL_PIVOT_LEFT:MOTION_GOAL_FORWARD,pivot?0:200,pivot?10000:0,100U,0U};
        int peak=0;
        assert(f.car.executor.config.pivot_max_effort==0U);
        assert(car_motion_enable_trial(&f.car,15000000U)); assert(car_motion_goal(&f.car,&r,0U));
        for(i=0U;i<60U && !f.car.executor.status.terminal_pending;++i) {
            motion_motor_state_t state; int l,rr;
            tick(&f,0U,0U);
            assert(drv_motor_read(&f.motor,&state)==MOTION_OK);
            l=state.left_effort<0?-state.left_effort:state.left_effort;
            rr=state.right_effort<0?-state.right_effort:state.right_effort;
            if(l>peak) { peak=l; }
            if(rr>peak) { peak=rr; }
        }
        assert(peak==300);
        car_motion_cancel(&f.car,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"sync")==0 || strcmp(argv[1],"sync_far")==0) {
        /* Straight 200 mm; the left wheel stops 20 mm (sync) or 50 mm
         * (sync_far) short. Within the 30 mm slack both stop together and the
         * goal completes; beyond it the old per-wheel rule (and its stall
         * fault) still applies. */
        uint64_t short_um=strcmp(argv[1],"sync")==0?20000U:50000U;
        motion_goal_request_t r={23U,1U,MOTION_SOURCE_OBSTACLE,MOTION_GOAL_FORWARD,200,0,100U,0U};
        motion_goal_result_t out;
        assert(f.car.executor.config.linear_sync_slack_um==30000U);
        assert(car_motion_enable_trial(&f.car,15000000U)); assert(car_motion_goal(&f.car,&r,0U));
        for(i=0U;i<800U && !f.car.executor.status.terminal_pending;++i) {
            uint64_t left=f.car.executor.measurement.sample.travel_um[0]-f.car.executor.goal_base_um[0];
            tick(&f,left+short_um<f.car.executor.goal_target_um?6U:0U,6U);
        }
        assert(f.car.executor.status.terminal_pending);
        assert(car_motion_result(&f.car,r.source,23U,&out));
        if(short_um==20000U) {
            assert(out.code==MOTION_RESULT_COMPLETED);
            assert(out.travelled_mm>=180 && out.travelled_mm<=200);  /* average of the two */
        } else {
            assert(out.code==MOTION_RESULT_FAULT);
            assert(f.car.executor.status.detail_reason==MOTION_DETAIL_NO_PROGRESS);
        }
        fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"pivot_sync")==0 || strcmp(argv[1],"pivot_sync_far")==0) {
        /* 60 deg pivot left (~59 mm per wheel); the left wheel stops 15 mm
         * (pivot_sync) or 30 mm (pivot_sync_far) short of its share. Within
         * the 20 mm pivot slack both stop and the turn completes. */
        uint64_t short_um=strcmp(argv[1],"pivot_sync")==0?15000U:30000U;
        motion_goal_request_t r={24U,1U,MOTION_SOURCE_OBSTACLE,MOTION_GOAL_PIVOT_LEFT,0,60000,100U,0U};
        motion_goal_result_t out;
        assert(f.car.executor.config.pivot_sync_slack_um==20000U);
        assert(car_motion_enable_trial(&f.car,15000000U)); assert(car_motion_goal(&f.car,&r,0U));
        for(i=0U;i<800U && !f.car.executor.status.terminal_pending;++i) {
            uint64_t left=f.car.executor.measurement.sample.travel_um[0]-f.car.executor.goal_base_um[0];
            tick(&f,(!f.car.executor.goal_started || left+short_um<f.car.executor.goal_target_um)?6U:0U,6U);
        }
        assert(f.car.executor.status.terminal_pending);
        assert(car_motion_result(&f.car,r.source,24U,&out));
        if(short_um==15000U) { assert(out.code==MOTION_RESULT_COMPLETED); }
        else {
            assert(out.code==MOTION_RESULT_FAULT);
            assert(f.car.executor.status.detail_reason==MOTION_DETAIL_NO_PROGRESS);
        }
        fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"unlimited")==0) {
        assert(car_motion_enable_trial(&f.car,MEASURED_MOTION_NO_RUN_LIMIT));
        for(i=0U;i<1250U;++i) { /* 25 s: past the old 15 s and the 20 s bound */
            if(i%5U==0U) { assert(line(&f,1U+i)); }
            tick(&f,2U,2U);
        }
        assert(f.car.executor.status.authorized && !f.car.executor.config.qualified);
        car_motion_cancel(&f.car,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.f);
    } else if(strcmp(argv[1],"goal")==0 || strcmp(argv[1],"goal_obstacle")==0) {
        motion_goal_request_t r={23U,1U,MOTION_SOURCE_JUNCTION,MOTION_GOAL_PIVOT_LEFT,0,90000,100U,0U};
        motion_goal_result_t out; unsigned left=0U,right=0U; bool counted=false;
        if(strcmp(argv[1],"goal_obstacle")==0) { r.source=MOTION_SOURCE_OBSTACLE; }
        assert(car_motion_enable_trial(&f.car,15000000U)); assert(car_motion_goal(&f.car,&r,0U));
        for(i=0;i<500U && !f.car.executor.status.terminal_pending;++i) {
            unsigned l,rr; motion_motor_state_t state;
            if(f.car.executor.goal_started && !counted) {
                uint64_t target=f.car.executor.goal_target_um;
                left=(unsigned)((target*1271U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
                right=(unsigned)((target*1264U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U)); counted=true;
            }
            assert(drv_motor_read(&f.motor,&state)==MOTION_OK);
            l=state.left_effort==0?0U:(left>13U?13U:left);
            rr=state.right_effort==0?0U:(right>13U?13U:right);
            tick(&f,l,rr); left-=l; right-=rr;
        }
        assert(car_motion_result(&f.car,r.source,23U,&out));
        assert(out.goal_id==23U && out.code==MOTION_RESULT_COMPLETED && out.turned_mdeg>80000);
        assert(!f.car.executor.config.qualified); car_motion_cancel(&f.car,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.f);
    } else {
        bool deadline=strcmp(argv[1],"deadline")==0;
        assert(car_motion_enable_trial(&f.car,deadline?300000U:15000000U));
        assert(line(&f,1U));
        for(i=0;i<10U;++i) { if(i==5U) { assert(line(&f,2U)); } tick(&f,2U,2U); }
        assert(f.car.executor.status.authorized && !f.car.executor.config.qualified);
        if(deadline) {
            assert(line(&f,3U));
            for(i=0;i<10U;++i) { tick(&f,2U,2U); }
            assert(!f.car.executor.status.authorized && f.car.executor.status.last_stop==MOTION_RESULT_TIMEOUT);
            assert(!line(&f,4U)); fake_assert_safe(&f.f);
        } else if(strcmp(argv[1],"stop")==0) {
            f.stop=true; assert(drv_motor_stop(&f.motor)==MOTION_OK); tick(&f,0U,0U);
            assert(!f.car.executor.status.authorized && !line(&f,3U)); fake_assert_safe(&f.f);
        } else {
            assert(strcmp(argv[1],"line")==0);
            for(i=0U;i<30U;++i) {
                if(i%5U==0U) { assert(line(&f,3U+i)); }
                tick(&f,2U,2U);
            }
            assert(f.car.executor.status.target_um_s[0]==100000 && f.car.executor.status.target_um_s[1]==100000);
            car_motion_cancel(&f.car,MOTION_RESULT_CANCELLED); fake_assert_safe(&f.f);
        }
    }
    printf("PASS explicit car trial %s\n",argv[1]); return 0;
}
