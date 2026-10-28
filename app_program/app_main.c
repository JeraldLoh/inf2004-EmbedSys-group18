/*
 * Course entry hook, kept deliberately inactive during the folder migration.
 * Original working examples are in docs/template-reference/app_program/.
 * Normal builds start no application task, GPIO, motor, sensor, ADC or network
 * API here. B2_BENCH=1 selects the explicit bounded, manual driver test instead.
 * S3_BENCH=1 (sensors only) or S3_BENCH=2 (line follow) selects Buddy 3's bench.
 * B5_RUN=1 selects the Buddy 5 obstacle task + vehicle controller task instead.
 * Kernel USB/radio services may already be active before this hook; this is NOT
 * an electrically qualified or hardware-safe image. Do not flash it yet.
 *
 * Later: create ALL controller/subsystem kernel objects, validate creation,
 * THEN start tasks with reviewed failure rollback, ownership and SMP policy.
 */
#include "app.h"
#if APP_S3_BENCH
#include "subsystem3/s3_bench.h"
#elif APP_B2_BENCH
#include "subsystem2/b2_bench_target.h"
#endif
#if APP_B5_RUN
#include "subsystem5/b5_run_target.h"
#endif

EXPORT INT usermain(void)
{
#if APP_S3_BENCH
    if (s3_bench_start() != E_OK)
    { tm_printf((UB*)"S3 bench task failed; nothing will move.\n"); }
#elif APP_B2_BENCH
    if (b2_bench_start() != E_OK)
    { tm_printf((UB*)"B2 bench task failed; no trial permitted.\n"); }
#elif APP_B5_RUN
    if (b5_run_start() != E_OK)
    { tm_printf((UB*)"B5 run failed; motors held stopped.\n"); }
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
