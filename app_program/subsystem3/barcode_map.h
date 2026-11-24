#ifndef GROUP18_BARCODE_MAP_H
#define GROUP18_BARCODE_MAP_H

/* Buddy 3: which barcode character means which turn.
 *
 * The letters are announced on the day, so the map is set at run time from
 * the dashboard's Barcode routes panel: Buddy 1 receives a binary MAP_SET on
 * /command and replaces the entire table. The decoder looks the letter up
 * after every read. The power-on map is empty.
 *
 * Portable: no tk_* calls and no hardware, so it builds in the host tests.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

#define BARCODE_MAP_SIZE 36U        /* A-Z then 0-9 */

/* Command for a decoded character, ROUTE_COMMAND_NONE if not mapped. */
route_command_t barcode_map_get(char character);
uint32_t barcode_map_revision(void); /* zero while a replacement is in flight */
bool barcode_map_replace(const uint8_t commands[BARCODE_MAP_SIZE]);

/* Set one character. Returns false for a character outside A-Z / 0-9. */
bool barcode_map_set(char character, route_command_t command);

void barcode_map_defaults(void);    /* A=LEFT B=RIGHT C=STRAIGHT D=UTURN */
void barcode_map_clear(void);       /* everything ROUTE_COMMAND_NONE */

#endif
