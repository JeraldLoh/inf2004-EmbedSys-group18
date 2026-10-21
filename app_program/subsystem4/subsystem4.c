/* Buddy 4: inactive lifecycle placeholders, not implemented functionality.
 * app_main.c deliberately does NOT call these hooks during this migration.
 * Later: init creates owned objects; start starts tasks only after ALL objects
 * (including the shared controller's) have been created successfully.
 */
#include "subsystem4.h"

EXPORT ER subsystem4_init(void)
{
    return E_NOSPT;
}

EXPORT ER subsystem4_start(void)
{
    return E_NOSPT;
}
