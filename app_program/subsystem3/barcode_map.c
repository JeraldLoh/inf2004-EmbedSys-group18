#include "barcode_map.h"

#include <stdatomic.h>
#include <stddef.h>

/* Four directions, one permitted character per direction. Each field stores
 * index + 1 (1..36), with 0 meaning unassigned. A native atomic word publishes
 * the whole 24-bit map. Keep the version handshake for scan invalidation. */
#define CHARACTER_BITS 6U
#define CHARACTER_MASK ((1U << CHARACTER_BITS) - 1U)
static atomic_uint_least32_t map_slots;
static atomic_uint_least32_t revision = 2U;
_Static_assert(BARCODE_MAP_SIZE <= CHARACTER_MASK, "barcode index must fit");
_Static_assert(ROUTE_COMMAND_UTURN * CHARACTER_BITS <= 32U, "route map must fit");

static unsigned slot_shift(unsigned command)
{
    return (command - 1U) * CHARACTER_BITS;
}

static void publish(uint32_t slots)
{
    uint32_t prior = atomic_load(&revision);
    atomic_store(&revision, prior + 1U);
    atomic_store(&map_slots, slots);
    atomic_store(&revision, prior == UINT32_MAX - 1U ? 2U : prior + 2U);
}

uint32_t barcode_map_revision(void)
{
    uint32_t current = atomic_load(&revision);
    return (current & 1U) ? 0U : current;
}

bool barcode_map_replace(const uint8_t commands[BARCODE_MAP_SIZE])
{
    uint32_t slots = 0U;
    if (commands == NULL) return false;
    for (unsigned i = 0; i < BARCODE_MAP_SIZE; ++i)
    {
        unsigned command = commands[i];
        if (command > ROUTE_COMMAND_UTURN) return false;
        if (command == ROUTE_COMMAND_NONE) continue;
        unsigned shift = slot_shift(command);
        if (((slots >> shift) & CHARACTER_MASK) != 0U) return false;
        slots |= (uint32_t)(i + 1U) << shift;
    }
    publish(slots);
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
    uint32_t slots = atomic_load(&map_slots);
    route_command_t command = ROUTE_COMMAND_NONE;
    for (unsigned i = ROUTE_COMMAND_LEFT; i <= ROUTE_COMMAND_UTURN; ++i)
    {
        if (((slots >> slot_shift(i)) & CHARACTER_MASK) == (unsigned)index + 1U)
        {
            command = (route_command_t)i;
            break;
        }
    }
    return current == barcode_map_revision() ? command : ROUTE_COMMAND_NONE;
}

bool barcode_map_set(char character, route_command_t command)
{
    int index = index_of(character);

    if ((index < 0) || ((unsigned)command > ROUTE_COMMAND_UTURN))
    {
        return false;
    }
    uint32_t slots = atomic_load(&map_slots);
    unsigned character_slot = (unsigned)index + 1U;
    for (unsigned i = ROUTE_COMMAND_LEFT; i <= ROUTE_COMMAND_UTURN; ++i)
    {
        unsigned shift = slot_shift(i);
        if (i == (unsigned)command || ((slots >> shift) & CHARACTER_MASK) == character_slot)
        {
            slots &= ~((uint32_t)CHARACTER_MASK << shift);
        }
    }
    if (command != ROUTE_COMMAND_NONE)
    {
        slots |= (uint32_t)character_slot << slot_shift((unsigned)command);
    }
    publish(slots);
    return true;
}

void barcode_map_clear(void)
{
    publish(0U);
}

void barcode_map_defaults(void)
{
    const uint8_t defaults[BARCODE_MAP_SIZE] =
        {ROUTE_COMMAND_LEFT, ROUTE_COMMAND_RIGHT, ROUTE_COMMAND_STRAIGHT, ROUTE_COMMAND_UTURN};
    (void)barcode_map_replace(defaults);
}
