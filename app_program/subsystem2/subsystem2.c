/* Buddy 2: inactive lifecycle placeholders, not implemented functionality.
 * app_main.c deliberately does NOT call these hooks during this migration.
 * Later: init creates owned objects; start starts tasks only after ALL objects
 * (including the shared controller's) have been created successfully.
 */
#include "subsystem2.h"

EXPORT ER subsystem2_init(void)
{
    return E_NOSPT;
}

EXPORT ER subsystem2_start(void)
{
    return E_NOSPT;
}
