#ifndef GROUP18_B2_PLATFORM_RP2040_H
#define GROUP18_B2_PLATFORM_RP2040_H
#include "motion_hal.h"
#include "b2_trial.h"
#include "measured_motion_executor.h"
#include "../project_config/gpio_dispatcher.h"

typedef struct
{
    stop_reason_mask_t reasons;
    bool cutoff_fired;
    uint32_t confirmed_us;
    uint32_t output_command_us;
    uint32_t stop_events;
    uint32_t max_lock_hold_us;
    uint32_t max_lock_wait_us;
} b2_platform_events_t;

/* Core-0 task/startup only. Reserves IO_BANK0 IRQ13 and TIMER alarm0/IRQ0.
 * SMP=1 kernel ISpinLock provides shared exclusion with local IRQ masking.
 * Does not start motion. Existing enabled IRQ ownership causes refusal. */
bool b2_platform_init(motion_hal_t *hal, b2_trial_io_t *trial_io);
void b2_platform_take_events(b2_platform_events_t *events);
/* Executor uses peek + conditional acknowledgement AFTER controller latches
 * the reason. A queued/full controller transport must not clear an inhibit. */
void b2_platform_peek_events(b2_platform_events_t *events);
bool b2_platform_ack_events(uint32_t evidence_id);
bool b2_platform_executor_io(measured_motion_executor_io_t *io);
/* Startup-only optional auxiliary cutoff (circuit-test servo). Called under
 * the common IRQ/core lock for physical STOP and alarm cutoff. Callback must
 * be bounded MMIO only: no locks, task/kernel calls or logging. */
bool b2_platform_bind_aux_stop(void (*callback)(void *), void *context);
gpio_dispatcher_t *b2_platform_dispatcher(void);
bool b2_platform_start_pressed(void);
bool b2_platform_stop_pressed(void);
uint32_t b2_platform_now_us(void);

#endif
