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
IMPORT void subsystem3_test(void);
IMPORT void subsystem4_test(void);
IMPORT void subsystem5_test(void);
#endif

/* The controller's future task/lifecycle adapter is separate from these five
 * hooks. It must be included in the all-init-before-any-start sequence later.
 * No task helper, priorities, IDs or thread safety are invented here.
 */
#endif
