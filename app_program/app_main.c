/*
 * Course entry hook, kept deliberately inactive during the folder migration.
 * Original working examples are in docs/template-reference/app_program/.
 * Default: no application tasks. B1_DEMO_RUN=1 enables only Buddy 1 demo.
 * No motor/sensor/controller task is started in either profile.
 * Kernel USB/radio services may already be active before this hook; this is NOT
 * an electrically qualified or hardware-safe image. Do not flash it yet.
 *
 * Later: create ALL controller/subsystem kernel objects, validate creation,
 * THEN start tasks with reviewed failure rollback, ownership and SMP policy.
 */
#include "app.h"
#include "subsystem1/subsystem1.h"

EXPORT INT usermain(void)
{
#if B1_STANDALONE_DEMO
    /* Explicit standalone profile only. All B1 objects exist before task start.
     * Other buddies and the portable controller are deliberately not started. */
    ER er = subsystem1_init();
    if (er == E_OK) er = subsystem1_start();
    if (er < E_OK) {
        tm_printf((UB*)"Buddy 1 startup failed (%d); no vehicle tasks started.\n", er);
    } else {
        tm_printf((UB*)"Buddy 1 TEXT DEMO: simulated data/commands; no motors.\n");
#if APP_TEST
        subsystem1_test();
#endif
    }
#else
    tm_printf((UB*)"Group 18: inactive architecture scaffold; no vehicle tasks started.\n");
#if APP_TEST
    tm_printf((UB*)"[TEST] SKIP: target tests not implemented; no PASS results claimed.\n");
#endif
#endif
    for (;;)
    {
        /* Returning from usermain() would shut down the kernel. */
        (void)tk_slp_tsk(TMO_FEVR);
    }
}
