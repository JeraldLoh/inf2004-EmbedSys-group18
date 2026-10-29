#include "b2_move_profile.h"
#if defined(APP_B2_MOVE_TEST) && APP_B2_MOVE_TEST
#include "../subsystem2/car_motion.h"
void b2_move_candidate_profile(measured_motion_executor_config_t *c)
{
    car_motion_candidate_config(c);
    /* UNTUNED starting values, not calibration evidence. Prior branch notes
     * reported effort199 at ~92/74mm/s. 199/92 and 199/74 provide rough FF.
     * Bound all tests to30%; normal CAR_RUN remains unqualified with zero gains. */
    c->pid[0]=(wheel_pid_config_t){1500U,1500U,0U,2163U,300U,100U,300U,500000U};
    c->pid[1]=(wheel_pid_config_t){1500U,1500U,0U,2689U,300U,100U,300U,500000U};
    c->max_speed_mm_s=120U; c->max_goal_mm=1000U;
    c->acceleration_um_s2=200000U;
    c->tolerance_um=2000U; c->overshoot_um=10000U;
    c->settle_us=200000U; c->settle_speed_um_s=3000U;
    c->progress_timeout_us=700000U; c->startup_grace_us=1500000U;
    c->qualified=false;
}
#endif
