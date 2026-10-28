#ifndef GROUP18_B5_RUN_TARGET_H
#define GROUP18_B5_RUN_TARGET_H
#include "../app.h"

/* B5_RUN=1 only. Starts a short-lived core-0 task that brings up B2's
 * motor/STOP/GPIO-interrupt foundation (motors stay inhibited), the vehicle
 * controller task and the Buddy 5 task: every init first, then every start.
 * Puts the controller in AUTO so obstacle missions are accepted. */
ER b5_run_start(void);

#endif
