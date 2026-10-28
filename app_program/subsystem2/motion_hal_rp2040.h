#ifndef GROUP18_MOTION_HAL_RP2040_H
#define GROUP18_MOTION_HAL_RP2040_H

#include "motion_hal.h"

/* Integration lead supplies reviewed synchronization, clock and the sole GPIO
 * dispatcher. This adapter deliberately has NO default interrupt/spinlock/SDK
 * callback installation. Port storage must outlive the bound HAL/drivers. */
typedef struct
{
    void *context;
    motion_lock_key_t (*enter)(void *context);
    void (*leave)(void *context, motion_lock_key_t key);
    bool (*clock_us)(void *context, uint32_t *now_us);
    bool (*attach_encoders)(void *context, motion_edge_handler_t handler,
        void *user);
    bool (*detach_encoders)(void *context);
} motion_rp2040_port_t;

/* Invalid/missing port returns false and does not touch GPIO/PWM. */
bool motion_hal_rp2040_create(motion_hal_t *hal, motion_rp2040_port_t *port);

#endif
