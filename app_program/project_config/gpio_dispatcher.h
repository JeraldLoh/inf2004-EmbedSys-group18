#ifndef GROUP18_GPIO_DISPATCHER_H
#define GROUP18_GPIO_DISPATCHER_H
#include <stdbool.h>
#include <stdint.h>

#define GPIO_DISPATCH_SLOTS 4U
typedef void (*gpio_edge_callback_t)(void *user, uint32_t rising,
    uint32_t falling, uint32_t observed_us);
typedef struct
{
    void *context;
    uintptr_t (*enter)(void *context);
    void (*leave)(void *context, uintptr_t key);
    /* Registration is restricted to core-0 task/startup context. IRQ dispatch
     * occurs on core 0 only. This gives detach quiescence without holding a
     * lock over client callbacks; other-core registration MUST be rejected. */
    bool (*registration_allowed)(void *context);
    bool (*enable_edges)(void *context, uint32_t mask, bool enabled);
    void (*ack_edges)(void *context, uint32_t rising, uint32_t falling);
} gpio_dispatch_io_t;
typedef struct
{
    uint32_t mask;
    gpio_edge_callback_t callback;
    void *user;
    bool urgent;
} gpio_dispatch_slot_t;
typedef struct
{
    gpio_dispatch_io_t io;
    gpio_dispatch_slot_t slots[GPIO_DISPATCH_SLOTS];
    bool initialized;
    bool dispatching;
} gpio_dispatcher_t;

bool gpio_dispatcher_init(gpio_dispatcher_t *dispatcher,
    const gpio_dispatch_io_t *io);
bool gpio_dispatcher_attach(gpio_dispatcher_t *dispatcher, uint32_t pins,
    gpio_edge_callback_t callback, void *user, bool urgent);
bool gpio_dispatcher_detach(gpio_dispatcher_t *dispatcher, uint32_t pins);
/* ACK only registered edges, preserve other sources; urgent clients first.
 * Callback is invoked OUTSIDE registry lock, allowing motion to take its lock. */
void gpio_dispatcher_dispatch(gpio_dispatcher_t *dispatcher, uint32_t rising,
    uint32_t falling, uint32_t observed_us);
#endif
