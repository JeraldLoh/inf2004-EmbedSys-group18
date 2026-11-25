/* Build this harness with either barcode_map.c using the same -O2 compiler. */
#include <assert.h>
#include <stdio.h>
#include <time.h>
#include "../../app_program/subsystem3/barcode_map.h"

int main(void)
{
    static const char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    const unsigned reads = 8000000U, writes = 400000U;
    uint8_t map[BARCODE_MAP_SIZE] = {0};
    map[0] = ROUTE_COMMAND_LEFT;
    map[25] = ROUTE_COMMAND_RIGHT;
    map[26] = ROUTE_COMMAND_STRAIGHT;
    map[35] = ROUTE_COMMAND_UTURN;
    assert(barcode_map_replace(map));
    unsigned sum = 0U;
    clock_t start = clock();
    for (unsigned i = 0U; i < reads; ++i)
    {
        sum += (unsigned)barcode_map_get(alphabet[i % BARCODE_MAP_SIZE]);
    }
    clock_t lookup_ticks = clock() - start;
    start = clock();
    for (unsigned i = 0U; i < writes; ++i)
    {
        map[0] = (i & 1U) ? ROUTE_COMMAND_LEFT : ROUTE_COMMAND_NONE;
        map[1] = (i & 1U) ? ROUTE_COMMAND_NONE : ROUTE_COMMAND_LEFT;
        assert(barcode_map_replace(map));
    }
    clock_t replace_ticks = clock() - start;
    printf("lookup_ns=%.2f replace_ns=%.2f checksum=%u revision=%lu\n",
        (double)lookup_ticks * 1e9 / CLOCKS_PER_SEC / reads,
        (double)replace_ticks * 1e9 / CLOCKS_PER_SEC / writes, sum,
        (unsigned long)barcode_map_revision());
    return 0;
}
