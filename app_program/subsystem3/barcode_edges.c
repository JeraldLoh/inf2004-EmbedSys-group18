#include "barcode_edges.h"

#include <stddef.h>

/* GCC emits a full "dmb" barrier for this on the Cortex-M0+. */
#define EDGES_BARRIER() __sync_synchronize()
#define EDGES_MASK (BARCODE_EDGES_SIZE - 1U)

void barcode_edges_reset(barcode_edges_t *edges)
{
    if (edges == NULL)
    {
        return;
    }
    edges->head = 0U;
    edges->tail = 0U;
    edges->overflow = 0U;
}

bool barcode_edges_push(barcode_edges_t *edges, bool black, uint32_t now_us)
{
    uint8_t head;
    uint8_t next;

    if (edges == NULL)
    {
        return false;
    }
    head = edges->head;
    next = (uint8_t)((head + 1U) & EDGES_MASK);
    if (next == edges->tail)
    {
        edges->overflow = 1U;    /* full: drop this edge, the read will fail */
        return false;
    }
    edges->edge[head] = (now_us & ~1UL) | (black ? 1UL : 0UL);
    EDGES_BARRIER();             /* data first, then publish the index */
    edges->head = next;
    return true;
}

bool barcode_edges_pop(barcode_edges_t *edges, bool *black, uint32_t *time_us)
{
    uint8_t tail;
    uint32_t packed;

    if ((edges == NULL) || (black == NULL) || (time_us == NULL))
    {
        return false;
    }
    tail = edges->tail;
    if (tail == edges->head)
    {
        return false;
    }
    EDGES_BARRIER();             /* see the index, then read the data */
    packed = edges->edge[tail];
    edges->tail = (uint8_t)((tail + 1U) & EDGES_MASK);
    *black = (packed & 1UL) != 0UL;
    *time_us = packed & ~1UL;
    return true;
}
