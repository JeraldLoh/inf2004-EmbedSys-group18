#ifndef GROUP18_MOTION_HAL_H
#define GROUP18_MOTION_HAL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MOTION_OK = 0,
    MOTION_INVALID,
    MOTION_STATE,
    MOTION_INHIBITED,
    MOTION_STALE,
    MOTION_IO_ERROR,
    MOTION_UNAVAILABLE,
    MOTION_REVERSAL_REQUIRED
} motion_status_t;

typedef uintptr_t motion_lock_key_t;
typedef void (*motion_edge_handler_t)(void *user, uint32_t rising_pins,
    uint32_t falling_pins, uint32_t observed_us);

/* REQUIRED platform contract, not an implementation of an SMP/ISR lock:
 * enter/leave serialize ALL motor, encoder and STOP callers across cores and
 * prevent same-core ISR preemption while held (mask before acquiring). They
 * cannot fail, sleep, allocate, or reenter a driver. No owner may be preempted
 * by an ISR that waits for this lock. Every other HAL operation is bounded,
 * nonblocking and called with this lock held, except attach/detach below.
 * Callbacks/context must outlive the driver objects. No other PWM writer.
 * A fake lock proves software ordering only; target proof remains required.
 */
typedef struct
{
    void *context;
    motion_lock_key_t (*enter)(void *context);
    void (*leave)(void *context, motion_lock_key_t key);
    bool (*gpio_write)(void *context, uint8_t pin, bool high);
    bool (*gpio_output)(void *context, uint8_t pin);
    bool (*gpio_input)(void *context, uint8_t pin);
    bool (*pwm_function)(void *context, uint8_t pin);
    bool (*pwm_compare)(void *context, uint8_t pin, uint16_t compare);
    bool (*pwm_wrap)(void *context, uint8_t first_pin, uint16_t wrap);
    bool (*pwm_enable)(void *context, uint8_t first_pin, bool enabled);
    bool (*clock_us)(void *context, uint32_t *now_us);
    /* Shared dispatcher owns registration/IRQ acknowledgement. Only GP27/5
     * BOTH edges subscribe here; sonar/STOP have independent routes. Attach
     * false must leave no subscription. Detach quiesces in-flight callbacks.
     * Neither operation runs with the driver lock held. */
    bool (*attach_encoders)(void *context, motion_edge_handler_t handler,
        void *user);
    bool (*detach_encoders)(void *context);
} motion_hal_t;

bool motion_hal_is_valid(const motion_hal_t *hal);

#endif
