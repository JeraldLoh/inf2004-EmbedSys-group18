#ifndef GROUP18_BARCODE_MAP_H
#define GROUP18_BARCODE_MAP_H

/* Buddy 3: which barcode character means which turn.
 *
 * The letters are announced on the day, so the map is set at run time from
 * the dashboard's Barcode routes panel: Buddy 1 receives a binary MAP_SET on
 * /command (see subsystem1/buddy1_protocol.h) and calls barcode_map_set()
 * for all 36 characters. The decoder looks the letter up after every read.
 * Power-on map: A=LEFT B=RIGHT C=STRAIGHT D=UTURN.
 *
 * Portable: no tk_* calls and no hardware, so it builds in the host tests.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

#define BARCODE_MAP_SIZE 36U        /* A-Z then 0-9 */

/* Command for a decoded character, ROUTE_COMMAND_NONE if not mapped. */
route_command_t barcode_map_get(char character);

/* Set one character. Returns false for a character outside A-Z / 0-9. */
bool barcode_map_set(char character, route_command_t command);

void barcode_map_defaults(void);    /* A=LEFT B=RIGHT C=STRAIGHT D=UTURN */
void barcode_map_clear(void);       /* everything ROUTE_COMMAND_NONE */

#endif
