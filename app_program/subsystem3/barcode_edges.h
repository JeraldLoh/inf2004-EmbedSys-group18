#ifndef GROUP18_BARCODE_EDGES_H
#define GROUP18_BARCODE_EDGES_H

/* Buddy 3: OPTIONAL interrupt-driven barcode timing (hardware independent).
 *
 * Polling the ADC needs a fast kernel tick (CNF_TIMER_PERIOD = 1). If the
 * barcode module's digital output D0 is wired to a GPIO instead, a GPIO edge
 * interrupt can timestamp every black/white change to the microsecond, and
 * the normal 10 ms tick is then enough for the rest.
 *
 * This is a single-producer / single-consumer ring buffer:
 *   producer = the GPIO interrupt (barcode_edges_push)
 *   consumer = the Buddy 3 task   (barcode_edges_pop)
 * Each side only writes its own index, so no lock is needed. A memory
 * barrier makes sure the data is visible before the index that publishes it,
 * which matters because the ISR and the task can run on different cores.
 *
 * Packing (Change Requirement 1): each edge is ONE uint32_t. Bits 31..1 are
 * the microsecond timestamp, bit 0 is the new level (1 = black). Dropping
 * the lowest timestamp bit costs 1 us of resolution; bars last >= 400 us.
 */
#include <stdbool.h>
#include <stdint.h>

#define BARCODE_EDGES_SIZE 32U   /* power of two; one barcode is 29 edges */

typedef struct
{
    uint32_t edge[BARCODE_EDGES_SIZE];
    volatile uint8_t head;       /* written only by the ISR */
    volatile uint8_t tail;       /* written only by the task */
    volatile uint8_t overflow;   /* set by ISR if the task fell behind */
} barcode_edges_t;

void barcode_edges_reset(barcode_edges_t *edges);

/* Call from the GPIO interrupt. Returns false if the buffer was full. */
bool barcode_edges_push(barcode_edges_t *edges, bool black, uint32_t now_us);

/* Call from the task. Returns false when empty. */
bool barcode_edges_pop(barcode_edges_t *edges, bool *black, uint32_t *time_us);

#endif
