/* Buddy 1: target-test placeholder. This is NOT a passing test.
 * No GPIO or motion is performed. Define real tests and obtain relevant
 * electrical clearance before replacing this placeholder.
 */
#include "subsystem1.h"

#if APP_TEST
EXPORT void subsystem1_test(void)
{
    tm_printf((UB*)"[TEST] subsystem1 SKIP: not implemented or run.\n");
}
#endif
