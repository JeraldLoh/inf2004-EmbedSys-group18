#include "fake_motion_hal.h"
#include "b2_trial.h"
#include <assert.h>
#include <stdio.h>
static bool pressed,armed;
static bool stop(void *p) { (void)p; return pressed; }
static bool cutoff(void *p,uint32_t us) { (void)p; assert(us==250000U); armed=true; return true; }
static void cancel(void *p) { (void)p; armed=false; }
int main(void) {
    fake_motion_hal_t f;
    motion_hal_t hal;
    drv_motor_t m={0}; drv_encoder_t e={0};
    vehicle_controller_t c;
    b2_trial_t t;
    b2_trial_result_t result;
    motion_motor_state_t state;
    b2_trial_io_t io={NULL,stop,stop,cutoff,cancel};
    uint32_t generation;
    unsigned i;
    fake_motion_init(&f); hal=fake_motion_hal(&f);
    assert(drv_motor_init(&m,&hal,MOTION_PWM_WRAP_CANDIDATE)==MOTION_OK);
    assert(drv_encoder_init(&e,&hal)==MOTION_OK); vehicle_controller_init(&c);
    assert(b2_trial_init(&t,&m,&e,&c,&io));
    assert(b2_trial_begin(&t,200,0,250000U)==MOTION_INVALID); /* old bench unchanged */
    assert(!b2_trial_init_limited(&t,&m,&e,&c,&io,301,250000U));
    assert(!b2_trial_init_limited(&t,&m,&e,&c,&io,300,500001U));
    assert(b2_trial_init_limited(&t,&m,&e,&c,&io,300,250000U));
    assert(b2_trial_begin(&t,301,0,250000U)==MOTION_INVALID);
    assert(b2_trial_begin(&t,-301,0,250000U)==MOTION_INVALID);
    assert(b2_trial_begin(&t,300,0,250001U)==MOTION_INVALID);
    pressed=true;
    assert(b2_trial_begin(&t,300,0,250000U)==MOTION_INHIBITED && !armed);
    b2_trial_service(&t,STOP_REASON_PHYSICAL,false); generation=c.authority_generation;
    for(i=0U;i<100U;++i) { f.trace_count=0U; b2_trial_service(&t,STOP_REASON_PHYSICAL,false); }
    assert(c.authority_generation==generation); /* held STOP does not spam new authority */
    pressed=false;
    assert(b2_trial_begin(&t,300,0,250000U)==MOTION_OK && armed);
    assert(drv_motor_read(&m,&state)==MOTION_OK && state.left_effort==300 && state.right_effort==0);
    f.now_us+=250000U; b2_trial_service(&t,0U,true);
    assert(b2_trial_take_result(&t,&result) && result.state==B2_TRIAL_TIME_ENDED && !armed);
    fake_assert_safe(&f);
    puts("PASS original bench cap, diagnostic cap/duration, STOP and finite cutoff"); return 0;
}
