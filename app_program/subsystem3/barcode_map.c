#include "barcode_map.h"

/* One byte per character. The comms task (Buddy 1, on a dashboard MAP
 * command) writes entries and the S3 task reads them; a single-byte store
 * cannot be seen half-written, so no lock is needed. A new map is applied
 * entry by entry, so a read during the update sees each letter either old
 * or new. */
static volatile uint8_t map_table[BARCODE_MAP_SIZE] = {
    (uint8_t)ROUTE_COMMAND_LEFT,      /* A */
    (uint8_t)ROUTE_COMMAND_RIGHT,     /* B */
    (uint8_t)ROUTE_COMMAND_STRAIGHT,  /* C */
    (uint8_t)ROUTE_COMMAND_UTURN      /* D, the rest start as NONE (0) */
};

static char upper(char c)
{
    return ((c >= 'a') && (c <= 'z')) ? (char)(c - 'a' + 'A') : c;
}

/* 'A'-'Z' -> 0-25, '0'-'9' -> 26-35, anything else -> -1. */
static int index_of(char c)
{
    c = upper(c);
    if ((c >= 'A') && (c <= 'Z'))
    {
        return c - 'A';
    }
    if ((c >= '0') && (c <= '9'))
    {
        return 26 + (c - '0');
    }
    return -1;
}

route_command_t barcode_map_get(char character)
{
    int index = index_of(character);
    return (index < 0) ? ROUTE_COMMAND_NONE : (route_command_t)map_table[index];
}

bool barcode_map_set(char character, route_command_t command)
{
    int index = index_of(character);

    if ((index < 0) || (command > ROUTE_COMMAND_UTURN))
    {
        return false;
    }
    map_table[index] = (uint8_t)command;
    return true;
}

void barcode_map_clear(void)
{
    unsigned int index;

    for (index = 0U; index < BARCODE_MAP_SIZE; index++)
    {
        map_table[index] = (uint8_t)ROUTE_COMMAND_NONE;
    }
}

void barcode_map_defaults(void)
{
    barcode_map_clear();
    (void)barcode_map_set('A', ROUTE_COMMAND_LEFT);
    (void)barcode_map_set('B', ROUTE_COMMAND_RIGHT);
    (void)barcode_map_set('C', ROUTE_COMMAND_STRAIGHT);
    (void)barcode_map_set('D', ROUTE_COMMAND_UTURN);
}
