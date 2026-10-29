#ifndef GROUP18_B2_MOVE_PROFILE_H
#define GROUP18_B2_MOVE_PROFILE_H
#include "../subsystem2/measured_motion_executor.h"
#if defined(APP_B2_MOVE_TEST) && APP_B2_MOVE_TEST
void b2_move_candidate_profile(measured_motion_executor_config_t *config);
#endif
#endif
