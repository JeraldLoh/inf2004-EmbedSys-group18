#include "barcode_map.h"

#include <stdatomic.h>

/* One writer. Readers fail closed during publication without waiting. */
static atomic_uchar map_table[BARCODE_MAP_SIZE];
static atomic_uint_least32_t revision = 2U;

uint32_t barcode_map_revision(void)
{
    uint32_t current = atomic_load(&revision);
    return (current & 1U) ? 0U : current;
}

bool barcode_map_replace(const uint8_t commands[BARCODE_MAP_SIZE])
{
    unsigned used = 0U;
    for (unsigned i = 0; i < BARCODE_MAP_SIZE; ++i)
    {
        unsigned command = commands[i];
        if (command > ROUTE_COMMAND_UTURN) return false;
        if (command == ROUTE_COMMAND_NONE) continue;
        if ((used & (1U << command)) != 0U) return false;
        used |= 1U << command;
    }
    uint32_t prior = atomic_load(&revision);
    atomic_store(&revision, prior + 1U);
    for (unsigned i = 0; i < BARCODE_MAP_SIZE; ++i)
        atomic_store(&map_table[i], commands[i]);
    atomic_store(&revision, prior == UINT32_MAX - 1U ? 2U : prior + 2U);
    return true;
}

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
    uint32_t current = barcode_map_revision();
    if (index < 0 || current == 0U) return ROUTE_COMMAND_NONE;
    route_command_t command = (route_command_t)atomic_load(&map_table[index]);
    return current == barcode_map_revision() ? command : ROUTE_COMMAND_NONE;
}

bool barcode_map_set(char character, route_command_t command)
{
    int index = index_of(character);

    if ((index < 0) || (command > ROUTE_COMMAND_UTURN))
    {
        return false;
    }
    uint8_t replacement[BARCODE_MAP_SIZE];
    for (unsigned i = 0; i < BARCODE_MAP_SIZE; ++i)
    {
        uint8_t old = atomic_load(&map_table[i]);
        replacement[i] = old == command ? ROUTE_COMMAND_NONE : old;
    }
    replacement[index] = (uint8_t)command;
    return barcode_map_replace(replacement);
}

void barcode_map_clear(void)
{
    const uint8_t blank[BARCODE_MAP_SIZE] = {0};
    (void)barcode_map_replace(blank);
}

void barcode_map_defaults(void)
{
    const uint8_t defaults[BARCODE_MAP_SIZE] =
        {ROUTE_COMMAND_LEFT, ROUTE_COMMAND_RIGHT, ROUTE_COMMAND_STRAIGHT, ROUTE_COMMAND_UTURN};
    (void)barcode_map_replace(defaults);
}
