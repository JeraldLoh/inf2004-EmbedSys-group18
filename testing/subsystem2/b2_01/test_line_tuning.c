/* Real digital line logic -> controller -> measured motion adapter. */
#include "fake_motion_hal.h"
#include "car_motion.h"
#include "../diagnostics/b2_move_profile.h"
#include "../subsystem3/line_follow.h"
#include "../vehicle_controller/controller_service.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

typedef struct {
    fake_motion_hal_t fake;
    drv_motor_t motor;
    drv_encoder_t encoder;
    car_motion_t car;
    line_follow_t line;
    controller_service_t controller;
    uint32_t ms, drives;
    bool barcode;
} fixture_t;
static bool pending(void *p) { (void)p; return false; }
static bool arm(void *p, uint32_t us) { (void)p; return us == 100000U; }
static void cancel(void *p) { (void)p; }
static bool halt(void *p) { car_motion_halt(&((fixture_t *)p)->car); return true; }
static bool read_line(void *p, controller_line_input_t *out) {
    fixture_t *f=p;
    out->steer=line_follow_steer(&f->line,f->ms); out->barcode_active=f->barcode;
    return true;
}
static bool drive(void *p, const motion_continuous_request_t *r) {
    fixture_t *f=p;
    assert(r->speed_mm_s==50); ++f->drives;
    return car_motion_continuous(&f->car,r,f->ms,f->ms*1000U);
}
static void sample(fixture_t *f, bool l, bool r, uint32_t ms) {
    f->ms=ms; (void)line_follow_update(&f->line,l,r,ms);
    controller_service_tick(&f->controller,ms);
    assert(f->controller.status.drive_rejected==0U);
}
int main(void) {
    fixture_t f; motion_hal_t hal; measured_motion_executor_config_t config;
    controller_event_t start={CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_AUTO,0,MISSION_IDLE,false};
    controller_event_t stop={CONTROLLER_EVENT_LATCH_STOP,SYSTEM_MODE_STOPPED,STOP_REASON_PHYSICAL,MISSION_IDLE,false};
    measured_motion_executor_io_t io={NULL,pending,arm,cancel};
    controller_outputs_t outputs={&f,halt,drive,read_line};
    uint32_t before;
    memset(&f,0,sizeof(f)); fake_motion_init(&f.fake); hal=fake_motion_hal(&f.fake);
    assert(drv_motor_init(&f.motor,&hal,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&f.encoder,&hal)==MOTION_OK);
    b2_move_candidate_profile(&config);
    assert(car_motion_init(&f.car,&f.motor,&f.encoder,&config,&io));
    assert(car_motion_enable_trial(&f.car,15000000U));
    line_follow_reset(&f.line);
    assert(controller_service_init(&f.controller,&outputs));
    assert(f.controller.line.slow_mm_s<=f.controller.line.cruise_mm_s);
    assert(controller_service_handle(&f.controller,&start));
    sample(&f,false,false,0); assert(f.car.executor.continuous.curvature_u1_m==0);
    sample(&f,true,false,1); assert(f.controller.status.last_steer==0); /* reject one glitch */
    sample(&f,true,false,2); assert(f.controller.status.last_steer==-12);
    sample(&f,true,false,102); assert(f.controller.status.last_steer==-28);
    sample(&f,true,false,302); assert(f.controller.status.last_steer==-62);
    sample(&f,true,false,470); assert(f.controller.status.last_steer==-90);
    sample(&f,true,false,1002); assert(f.controller.status.last_steer==-90);
    assert(f.car.executor.continuous.curvature_u1_m==-11); /* turn left, both forward */
    sample(&f,false,false,1003); sample(&f,false,false,1004);
    assert(f.controller.status.last_steer==-90); /* no sudden straight command on a gap */
    sample(&f,false,false,1104); assert(f.controller.status.last_steer==-45);
    sample(&f,true,false,1105); sample(&f,true,false,1106);
    assert(f.controller.status.last_steer==-44); /* same side resumes, not reset to12 */
    sample(&f,true,false,1166); assert(f.controller.status.last_steer==-54);
    sample(&f,false,false,1167); sample(&f,false,false,1168);
    sample(&f,false,false,1268); assert(f.controller.status.last_steer==-27);
    sample(&f,false,false,1368); assert(f.controller.status.last_steer==0);
    sample(&f,false,false,1568); assert(f.controller.status.last_steer==0); /* never indefinite */
    sample(&f,false,true,1569); sample(&f,false,true,1570);
    assert(f.controller.status.last_steer==12);
    sample(&f,false,true,1670); assert(f.controller.status.last_steer==28);
    sample(&f,false,true,1870); assert(f.controller.status.last_steer==62);
    sample(&f,false,true,2570); assert(f.controller.status.last_steer==90);
    assert(f.car.executor.continuous.curvature_u1_m==11);
    sample(&f,false,false,2571); sample(&f,false,false,2572);
    sample(&f,false,false,2672); assert(f.controller.status.last_steer==45);
    sample(&f,true,false,2673); sample(&f,true,false,2674);
    assert(f.controller.status.last_steer==-12); /* opposite evidence overrides memory */
    f.barcode=true; sample(&f,true,false,2675); /* slow mode never increases speed */
    before=f.drives; assert(controller_service_handle(&f.controller,&stop));
    controller_service_tick(&f.controller,2680); assert(f.drives==before);
    line_follow_reset(&f.line); assert(line_follow_steer(&f.line,2680)==0);
    assert(!f.car.executor.config.qualified); fake_assert_safe(&f.fake);
    puts("50mm/s line candidate: filtering, ramp, cap, fading gap memory, same/opposite side, reset, slow mode and STOP passed");
    return 0;
}
