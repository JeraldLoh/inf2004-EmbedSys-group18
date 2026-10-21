/* Buddy 3: target-test placeholder. This is NOT a passing test.
 * No GPIO or motion is performed. Define real tests and obtain relevant
 * electrical clearance before replacing this placeholder.
 */
#include "subsystem3.h"

#if APP_TEST
EXPORT void subsystem3_test(void)
{
    tm_printf((UB*)"[TEST] subsystem3 SKIP: not implemented or run.\n");
}
#endif
