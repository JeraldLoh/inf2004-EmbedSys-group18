#ifndef GROUP18_APP_H
#define GROUP18_APP_H

/* Course lifecycle boundary. These declarations are not a running system.
 * See app_main.c and TEAM.md before implementing or starting any task.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>

#ifndef APP_TEST
#define APP_TEST 0
#endif
#ifndef APP_CIRCUIT_TEST
#define APP_CIRCUIT_TEST 0 /* manual circuit diagnostic, never autonomous */
#endif
#ifndef APP_B2_MOVE_TEST
#define APP_B2_MOVE_TEST 0 /* bounded B2-only movement/calibration image */
#endif
#ifndef APP_B5_RUN
#define APP_B5_RUN 0     /* 1 = Buddy 5 obstacle task + controller, motors off */
#endif
#ifndef APP_CAR_RUN
#define APP_CAR_RUN 0    /* 1 = whole car: line following + obstacle avoidance */
#endif

IMPORT ER subsystem1_init(void);
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem3_init(void);
IMPORT ER subsystem4_init(void);
IMPORT ER subsystem5_init(void);
IMPORT ER subsystem1_start(void);
IMPORT ER subsystem2_start(void);
IMPORT ER subsystem3_start(void);
IMPORT ER subsystem4_start(void);
IMPORT ER subsystem5_start(void);

#if APP_TEST
IMPORT void subsystem1_test(void);
IMPORT void subsystem2_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* The controller's future task/lifecycle adapter is separate from these five
 * hooks. It must be included in the all-init-before-any-start sequence later.
 * No task helper, priorities, IDs or thread safety are invented here.
 */
#endif
