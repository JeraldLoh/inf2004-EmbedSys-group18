#include "gpio_dispatcher.h"
#include <stddef.h>

bool gpio_dispatcher_init(gpio_dispatcher_t *dispatcher,
    const gpio_dispatch_io_t *io)
{
    if ((dispatcher == NULL) || (io == NULL) || dispatcher->initialized ||
        (io->enter == NULL) || (io->leave == NULL) ||
        (io->registration_allowed == NULL) || (io->enable_edges == NULL) ||
        (io->ack_edges == NULL) || !io->registration_allowed(io->context))
    {
        return false;
    }
    dispatcher->io = *io;
    dispatcher->initialized = true;
    return true;
}

bool gpio_dispatcher_attach(gpio_dispatcher_t *dispatcher, uint32_t pins,
    gpio_edge_callback_t callback, void *user, bool urgent)
{
    uintptr_t key;
    unsigned i;
    unsigned empty = GPIO_DISPATCH_SLOTS;
    bool ok = false;
    if ((dispatcher == NULL) || !dispatcher->initialized || (pins == 0U) ||
        ((pins & 0xC0000000UL) != 0U) || (callback == NULL) ||
        !dispatcher->io.registration_allowed(dispatcher->io.context))
    {
        return false;
    }
    key = dispatcher->io.enter(dispatcher->io.context);
    if (!dispatcher->dispatching)
    {
        for (i = 0U; i < GPIO_DISPATCH_SLOTS; ++i)
        {
            if ((dispatcher->slots[i].mask & pins) != 0U)
            {
                break;
            }
            if (dispatcher->slots[i].mask == 0U) { empty = i; }
        }
        if ((i == GPIO_DISPATCH_SLOTS) && (empty != GPIO_DISPATCH_SLOTS))
        {
            /* Discard pre-subscription edges BEFORE enabling this client. */
            dispatcher->io.ack_edges(dispatcher->io.context, pins, pins);
            if (dispatcher->io.enable_edges(dispatcher->io.context, pins, true))
            {
                dispatcher->slots[empty] = (gpio_dispatch_slot_t)
                    {pins, callback, user, urgent};
                ok = true;
            }
        }
    }
    dispatcher->io.leave(dispatcher->io.context, key);
    return ok;
}

bool gpio_dispatcher_detach(gpio_dispatcher_t *dispatcher, uint32_t pins)
{
    uintptr_t key;
    unsigned i;
    bool ok = false;
    if ((dispatcher == NULL) || !dispatcher->initialized || (pins == 0U) ||
        !dispatcher->io.registration_allowed(dispatcher->io.context))
    {
        return false;
    }
    key = dispatcher->io.enter(dispatcher->io.context);
    if (!dispatcher->dispatching)
    {
        for (i = 0U; i < GPIO_DISPATCH_SLOTS; ++i)
        {
            if (dispatcher->slots[i].mask == pins)
            {
                if (dispatcher->io.enable_edges(dispatcher->io.context, pins, false))
                {
                    dispatcher->io.ack_edges(dispatcher->io.context, pins, pins);
                    dispatcher->slots[i] = (gpio_dispatch_slot_t){0};
                    ok = true;
                }
                break;
            }
        }
    }
    dispatcher->io.leave(dispatcher->io.context, key);
    return ok;
}

void gpio_dispatcher_dispatch(gpio_dispatcher_t *dispatcher, uint32_t rising,
    uint32_t falling, uint32_t observed_us)
{
    gpio_dispatch_slot_t copy[GPIO_DISPATCH_SLOTS];
    uint32_t owned = 0U;
    uintptr_t key;
    unsigned i;
    unsigned pass;
    if ((dispatcher == NULL) || !dispatcher->initialized) { return; }
    key = dispatcher->io.enter(dispatcher->io.context);
    if (dispatcher->dispatching)
    {
        dispatcher->io.leave(dispatcher->io.context, key);
        return;
    }
    dispatcher->dispatching = true;
    for (i = 0U; i < GPIO_DISPATCH_SLOTS; ++i)
    {
        copy[i] = dispatcher->slots[i];
        owned |= copy[i].mask;
    }
    dispatcher->io.ack_edges(dispatcher->io.context, rising & owned, falling & owned);
    dispatcher->io.leave(dispatcher->io.context, key);
    for (pass = 0U; pass < 2U; ++pass)
    {
        for (i = 0U; i < GPIO_DISPATCH_SLOTS; ++i)
        {
            if ((copy[i].mask & (rising | falling)) != 0U &&
                (copy[i].urgent == (pass == 0U)))
            {
                copy[i].callback(copy[i].user, rising & copy[i].mask,
                    falling & copy[i].mask, observed_us);
            }
        }
    }
    key = dispatcher->io.enter(dispatcher->io.context);
    dispatcher->dispatching = false;
    dispatcher->io.leave(dispatcher->io.context, key);
}
