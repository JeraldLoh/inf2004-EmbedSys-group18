#ifndef GROUP18_BARCODE_MAP_H
#define GROUP18_BARCODE_MAP_H

/* Buddy 3: which barcode character means which turn.
 *
 * On demo day the letters are only announced on the day, so the mapping is
 * set at run time over MQTT instead of being fixed in the code. Buddy 1's
 * command handler passes any message that starts with "MAP" to
 * barcode_map_command(); the decoder looks the letter up after every read.
 *
 * MQTT text format (case does not matter; spaces, commas, '=' or ':' all
 * work as separators):
 *
 *   MAP K LEFT Q RIGHT Z STRAIGHT M UTURN   set some letters (others kept)
 *   MAP K=L, Q=R, Z=S, M=U                  same, short command names
 *   MAP CLEAR K L Q R                       forget all, then set these
 *   MAP CLEAR                               no letter means anything
 *   MAP DEFAULT                             back to A=L B=R C=S D=U
 *   MAP SHOW                                change nothing, just reply
 *   MAP K NONE                              K no longer means anything
 *
 * Commands: LEFT/L, RIGHT/R, STRAIGHT/S, UTURN/U, NONE/X.
 * Characters: A-Z and 0-9.
 * A message is applied all-or-nothing: if any pair is invalid, nothing
 * changes and the reply says which word was wrong.
 *
 * Portable: no tk_* calls and no hardware, so it builds in the host tests.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

#define BARCODE_MAP_SIZE 36U        /* A-Z then 0-9 */
#define BARCODE_MAP_REPLY_MAX 160U  /* fits Buddy 1's 192-byte payload */

/* Command for a decoded character, ROUTE_COMMAND_NONE if not mapped. */
route_command_t barcode_map_get(char character);

/* Set one character. Returns false for a character outside A-Z / 0-9. */
bool barcode_map_set(char character, route_command_t command);

void barcode_map_defaults(void);    /* A=LEFT B=RIGHT C=STRAIGHT D=UTURN */
void barcode_map_clear(void);       /* everything ROUTE_COMMAND_NONE */

/* True if the message is a MAP command (so Buddy 1 should hand it here). */
bool barcode_map_is_command(const char *text, unsigned int length);

/* Apply a MAP message. The reply (always NUL-terminated) is the new map,
 * e.g. "OK A=L B=R C=S D=U", or "ERR <reason>". Returns true on success. */
bool barcode_map_command(
    const char *text,
    unsigned int length,
    char *reply,
    unsigned int reply_size);

/* One-letter name for a command: L R S U, or '-' for NONE. */
char barcode_map_command_letter(route_command_t command);

#endif
