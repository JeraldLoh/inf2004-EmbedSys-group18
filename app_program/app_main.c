/*
 * Course entry hook, kept deliberately inactive during the folder migration.
 * Original working examples are in docs/template-reference/app_program/.
 * No application task, GPIO, motor, sensor, ADC or network API is started here.
 * Kernel USB/radio services may already be active before this hook; this is NOT
 * an electrically qualified or hardware-safe image. Do not flash it yet.
 *
 * Later: create ALL controller/subsystem kernel objects, validate creation,
 * THEN start tasks with reviewed failure rollback, ownership and SMP policy.
 */
#include "app.h"

EXPORT INT usermain(void)
{
    tm_printf((UB*)"Group 18: inactive architecture scaffold; no vehicle tasks started.\n");
#if APP_TEST
    tm_printf((UB*)"[TEST] SKIP: target tests not implemented; no PASS results claimed.\n");
#endif
    for (;;)
    {
        /* Returning from usermain() would shut down the kernel. */
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
