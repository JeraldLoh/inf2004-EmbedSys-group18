#ifndef GROUP18_DRV_SONAR_H
#define GROUP18_DRV_SONAR_H

/* Buddy 5: HC-SR04 on the kernel image (target only).
 *
 * TRIG is a plain output. ECHO edges are timestamped by the shared GPIO
 * dispatcher's interrupt (registration from core 0 task/startup context
 * only); the reading task sleeps with tk_dly_tsk() while it waits, so it
 * never spins for the length of an echo. Pulse width -> distance uses the
 * portable range_from_echo_us().
 */
#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../project_config/gpio_dispatcher.h"
#include "range.h"

typedef struct
{
    UINT trig_pin;
    UINT echo_pin;
    float cm_per_us;
    float min_cm;
    float max_cm;
    gpio_dispatcher_t *dispatcher;
    bool attached;
    /* Written by the GPIO interrupt, read by the sonar task. Each timestamp
     * is written before its flag, and the task reads the flag first. */
    volatile uint32_t rise_us;
    volatile uint32_t fall_us;
    volatile bool rose;
    volatile bool fell;
    volatile bool both_in_one; /* both edges in one interrupt: width unknown */
} drv_sonar_t;

/* Fails (E_OBJ) if the dispatcher is not initialised or ECHO is taken. */
ER drv_sonar_init(drv_sonar_t *sonar, gpio_dispatcher_t *dispatcher,
    UINT trig_pin, UINT echo_pin);
ER drv_sonar_deinit(drv_sonar_t *sonar);

/* Median of `samples` pings, spacing_ms apart. Blocks the calling task for
 * at most about samples * 110 ms. */
range_reading_t drv_sonar_read(drv_sonar_t *sonar, size_t samples,
    UINT spacing_ms);

#endif
