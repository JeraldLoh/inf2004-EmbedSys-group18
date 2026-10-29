#include "fake_motion_hal.h"
#include "car_motion.h"
#include "../vehicle_controller/controller_service.h"
#include "../subsystem5/b5_motion.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    fake_motion_hal_t fake;
    drv_motor_t motor;
    drv_encoder_t encoder;
    car_motion_t motion;
    controller_service_t controller;
    b5_motion_t b5;
    uint32_t ms;
    int8_t steer;
    bool pending, armed, events;
    bool route_taken, junction_taken;
} fixture_t;
static fixture_t *current;
static bool pending(void *p) { return ((fixture_t *)p)->pending; }
static bool watchdog(void *p,uint32_t duration)
{ assert(duration==100000U); ((fixture_t *)p)->armed=true; return !pending(p); }
static void cancel_watchdog(void *p) { ((fixture_t *)p)->armed=false; }
static bool stop(void *p) { car_motion_halt(&((fixture_t *)p)->motion); return true; }
static bool drive(void *p,const motion_continuous_request_t *r) {
    fixture_t *f=p; return car_motion_continuous(&f->motion,r,f->ms,f->fake.now_us);
}
static bool line(void *p,controller_line_input_t *r) {
    r->steer=((fixture_t *)p)->steer; r->barcode_active=false; return true;
}
static bool authority(motion_source_t s,authority_generation_t *g)
{ return controller_service_authority(&current->controller,s,g); }
static bool goal(const motion_goal_request_t *r)
{ return car_motion_goal(&current->motion,r,current->fake.now_us); }
static bool b5_drive(const motion_continuous_request_t *r) { return drive(current,r); }
static bool b5_result(goal_id_t id,motion_goal_result_t *r)
{ return car_motion_result(&current->motion,MOTION_SOURCE_OBSTACLE,id,r); }
static void halt(void) { (void)stop(current); }
static bool seen(void) { return true; }
static void reacquire(bool left) { (void)left; }
static void event(fixture_t *f,controller_event_type_t type,system_mode_t mode,bool rejoined) {
    controller_event_t e={type,mode,STOP_REASON_REMOTE,MISSION_IDLE,rejoined};
    assert(controller_service_handle(&f->controller,&e));
}
static void init(fixture_t *f,bool qualified) {
    motion_hal_t hal;
    measured_motion_executor_config_t c;
    measured_motion_executor_io_t io={f,pending,watchdog,cancel_watchdog};
    controller_outputs_t outputs={f,stop,drive,line};
    b5_motion_ops_t b5={authority,goal,b5_drive,b5_result,halt,seen,reacquire};
    b5_motion_config_t b5_config;
    memset(f,0,sizeof(*f)); current=f; fake_motion_init(&f->fake);
    hal=fake_motion_hal(&f->fake);
    assert(drv_motor_init(&f->motor,&hal,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f->encoder,&hal)==MOTION_OK);
    car_motion_candidate_config(&c); assert(!c.qualified && c.pid[0].kp_milli==0U);
    if(qualified) {
        /* Synthetic test gains. This is not hardware calibration evidence. */
        c.qualified=true;
        c.pid[0]=(wheel_pid_config_t){2000U,1000U,0U,4000U,350U,80U,0U,0U};
        c.pid[1]=c.pid[0];
    }
    assert(car_motion_init(&f->motion,&f->motor,&f->encoder,&c,&io));
    assert(controller_service_init(&f->controller,&outputs));
    f->controller.line.cruise_mm_s=30U; f->controller.line.slow_mm_s=15U;
    event(f,CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_AUTO,false);
    b5_motion_default_config(&b5_config); b5_config.goal_speed_mm_s=30U;
    assert(b5_config.goal_timeout_ms==0U);
    b5_motion_init(&f->b5,&b5,&b5_config);
}
static void tick(fixture_t *f,unsigned l,unsigned r) {
    unsigned i;
    f->fake.trace_count=0U; f->fake.now_us+=20000U; f->ms+=20U;
    for(i=0;i<l;++i) { drv_encoder_on_edges(&f->encoder,1UL<<27,0U,f->fake.now_us); }
    for(i=0;i<r;++i) { drv_encoder_on_edges(&f->encoder,1UL<<5,0U,f->fake.now_us); }
    car_motion_generation(&f->motion,f->controller.controller.authority_generation);
    car_motion_step(&f->motion);
    controller_service_tick(&f->controller,f->ms);
    if(f->b5.state==B5_MOVE_SEARCH) { b5_motion_service(&f->b5,f->ms); }
}
static void warm(fixture_t *f) { unsigned i; for(i=0;i<8U;++i) { tick(f,0U,0U); } }
static void finish_goal(fixture_t *f) {
    uint32_t internal=f->motion.executor.goal.goal_id;
    unsigned i,l=0U,r=0U,dl=0U,dr=0U;
    bool paths=false;
    for(i=0U;i<400U;++i) {
        measured_motion_executor_t *e=&f->motion.executor;
        if(e->status.state!=MOTION_EXEC_GOAL || e->goal.goal_id!=internal) { return; }
        if(e->goal_started && !paths) {
            uint64_t target=e->goal_target_um;
            l=(unsigned)((target*1271U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
            r=(unsigned)((target*1264U*1000U+65ULL*3141593U/2U)/(65ULL*3141593U));
            paths=true;
        }
        dl=l>4U?4U:l; dr=r>4U?4U:r;
        tick(f,dl,dr); l-=dl; r-=dr;
    }
    assert(!"goal failed to terminate");
}
static void unqualified(void) {
    fixture_t f;
    init(&f,false); warm(&f);
    assert(!f.motion.executor.status.authorized && f.motion.rejected>0U);
    fake_assert_safe(&f.fake);
}
static void steering(void) {
    fixture_t f;
    unsigned i;
    init(&f,true); f.steer=40; warm(&f);
    for(i=0U;i<10U;++i) { tick(&f,2U,1U); }
    assert(f.motion.executor.continuous.curvature_u1_m==7);
    assert(f.motion.executor.status.target_um_s[0]>f.motion.executor.status.target_um_s[1]);
    f.steer=-40; tick(&f,0U,0U); tick(&f,0U,0U);
    assert(f.motion.executor.continuous.curvature_u1_m==-7);
    assert(f.motion.executor.status.target_um_s[0]<f.motion.executor.status.target_um_s[1]);
    f.steer=100; tick(&f,0U,0U);
    assert(f.motion.executor.continuous.curvature_u1_m==17); /* No wheel reversal. */
}
static void obstacle(void) {
    fixture_t f;
    unsigned i;
    init(&f,true); warm(&f);
    event(&f,CONTROLLER_EVENT_OBSTACLE_DETECTED,SYSTEM_MODE_AUTO,false);
    b5_motion_search(&f.b5,true); b5_motion_service(&f.b5,f.ms);
    assert(f.motion.executor.continuous.source==MOTION_SOURCE_OBSTACLE);
    assert(f.motion.executor.continuous.curvature_u1_m==-3);
    warm(&f);
    for(i=0U;i<35U;++i) { tick(&f,3U,4U); }
    assert(f.motion.executor.status.target_um_s[0]<f.motion.executor.status.target_um_s[1]);
    event(&f,CONTROLLER_EVENT_OBSTACLE_CLEARED,SYSTEM_MODE_AUTO,true);
    tick(&f,0U,0U);
    assert(f.motion.executor.continuous.source==MOTION_SOURCE_LINE);
}
static void b5_goal(void) {
    fixture_t f;
    init(&f,true); warm(&f);
    event(&f,CONTROLLER_EVENT_OBSTACLE_DETECTED,SYSTEM_MODE_AUTO,false);
    b5_motion_straight(&f.b5,100,false,f.ms);
    b5_motion_service(&f.b5,f.ms);
    assert(f.b5.state==B5_MOVE_GOAL);
    finish_goal(&f); b5_motion_service(&f.b5,f.ms);
    assert(b5_motion_done(&f.b5) && f.b5.last_result==MOTION_RESULT_COMPLETED);
    assert(!f.motion.executor.status.terminal_pending);
    b5_motion_turn(&f.b5,-90000,f.ms); b5_motion_service(&f.b5,f.ms);
    assert(f.b5.state==B5_MOVE_GOAL && f.motion.executor.goal.type==MOTION_GOAL_PIVOT_RIGHT);
    finish_goal(&f); b5_motion_service(&f.b5,f.ms);
    assert(b5_motion_done(&f.b5));
}
static bool take_route(void *p,route_instruction_t *r) {
    fixture_t *f=p;
    if(!f->events || f->route_taken) { return false; }
    *r=(route_instruction_t){1U,ROUTE_COMMAND_LEFT,'L',f->ms,f->ms+10000U,true};
    f->route_taken=true; return true;
}
static bool take_junction(void *p,junction_observation_t *j) {
    fixture_t *f=p;
    if(!f->events || f->junction_taken) { return false; }
    *j=(junction_observation_t){true,true,true,true,true,100U,f->ms};
    f->junction_taken=true; return true;
}
static bool no_probe(void *p) { (void)p; return false; }
static bool probe(void *p) { (void)p; return true; }
static bool junction_goal(void *p,const motion_goal_request_t *r)
{ fixture_t *f=p; return car_motion_goal(&f->motion,r,f->fake.now_us); }
static bool junction_result(void *p,goal_id_t id,motion_goal_result_t *r)
{ return car_motion_result(&((fixture_t *)p)->motion,MOTION_SOURCE_JUNCTION,id,r); }
static void junction(void) {
    fixture_t f;
    controller_junction_outputs_t ops={&f,take_route,take_junction,no_probe,probe,probe,junction_goal,junction_result};
    init(&f,true);
    assert(controller_service_bind_junction(&f.controller,&ops));
    f.controller.junction.goal_speed_mm_s=30U;
    assert(f.controller.junction.goal_timeout_ms==0U);
    warm(&f); f.events=true; tick(&f,0U,0U);
    assert(f.controller.junction_phase==CONTROLLER_JUNCTION_APPROACH);
    finish_goal(&f);
    assert(f.controller.junction_phase==CONTROLLER_JUNCTION_TURN);
    assert(f.motion.executor.goal.type==MOTION_GOAL_PIVOT_LEFT);
    assert(f.controller.controller.mission==MISSION_JUNCTION_EXECUTE); /* No early line resume. */
    finish_goal(&f);
    assert(f.controller.junction_phase==CONTROLLER_JUNCTION_IDLE);
    assert(f.controller.status.junctions_done==1U);
    assert(f.controller.controller.mission==MISSION_VERIFY_LINE);
}
static void stop_and_ids(void) {
    fixture_t f;
    motion_goal_request_t g={1U,0U,MOTION_SOURCE_JUNCTION,MOTION_GOAL_FORWARD,100,0,30U,0U};
    motion_goal_result_t result;
    init(&f,true); warm(&f);
    g.authority_generation=f.controller.controller.authority_generation;
    assert(car_motion_goal(&f.motion,&g,f.fake.now_us));
    f.pending=true; tick(&f,0U,0U); fake_assert_safe(&f.fake);
    f.pending=false;
    assert(!car_motion_goal(&f.motion,&g,f.fake.now_us)); /* STOP cannot rearm same scope. */
    event(&f,CONTROLLER_EVENT_OBSTACLE_DETECTED,SYSTEM_MODE_AUTO,false);
    g.source=MOTION_SOURCE_OBSTACLE; g.authority_generation=f.controller.controller.authority_generation;
    assert(car_motion_goal(&f.motion,&g,f.fake.now_us)); /* Reuses external id=1, new source. */
    car_motion_generation(&f.motion,g.authority_generation-1U); /* Delayed cross-core snapshot. */
    assert(f.motion.executor.status.authorized && f.motion.executor.status.state==MOTION_EXEC_GOAL);
    assert(!car_motion_result(&f.motion,MOTION_SOURCE_JUNCTION,1U,&result));
    assert(car_motion_result(&f.motion,MOTION_SOURCE_OBSTACLE,1U,&result));
    assert(result.code==MOTION_RESULT_PROGRESS && result.goal_id==1U);
    car_motion_halt(&f.motion); fake_assert_safe(&f.fake);
    assert(!car_motion_goal(&f.motion,&g,f.fake.now_us)); /* Duplicate goal cannot replay. */
    g.goal_id=2U;
    assert(car_motion_goal(&f.motion,&g,f.fake.now_us)); /* Scoped pause permits explicit next goal. */
}
static void deadline(void) {
    fixture_t f;
    motion_goal_request_t g={1U,0U,MOTION_SOURCE_OBSTACLE,MOTION_GOAL_FORWARD,1000,0,30U,4000U};
    motion_continuous_request_t r={1U,0U,MOTION_SOURCE_LINE,30,0,0U,0U,150U};
    init(&f,true);
    r.authority_generation=f.controller.controller.authority_generation;
    assert(!car_motion_continuous(&f.motion,&r,150U,0U));
    event(&f,CONTROLLER_EVENT_OBSTACLE_DETECTED,SYSTEM_MODE_AUTO,false);
    g.authority_generation=f.controller.controller.authority_generation;
    assert(!car_motion_goal(&f.motion,&g,f.fake.now_us));
    g.timeout_ms=0U;
    assert(car_motion_goal(&f.motion,&g,f.fake.now_us));
    assert(f.motion.executor.goal.timeout_ms>33333U && f.motion.executor.goal.timeout_ms<60000U);
}
int main(int argc,char **argv) {
    assert(argc==2);
    if(strcmp(argv[1],"unqualified")==0) { unqualified(); }
    else if(strcmp(argv[1],"steering")==0) { steering(); }
    else if(strcmp(argv[1],"obstacle")==0) { obstacle(); }
    else if(strcmp(argv[1],"b5_goal")==0) { b5_goal(); }
    else if(strcmp(argv[1],"junction")==0) { junction(); }
    else if(strcmp(argv[1],"stop_ids")==0) { stop_and_ids(); }
    else if(strcmp(argv[1],"deadline")==0) { deadline(); }
    else { assert(false); }
    printf("PASS CAR_RUN %s\n",argv[1]); return 0;
}
