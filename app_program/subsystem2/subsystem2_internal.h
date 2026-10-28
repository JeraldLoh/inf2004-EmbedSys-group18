#ifndef GROUP18_SUBSYSTEM2_INTERNAL_H
#define GROUP18_SUBSYSTEM2_INTERNAL_H
#include "subsystem2.h"

/* Buddy 2 only; never a whole-system command/PWM API. The bench must run only
 * under APP_B2_BENCH and cannot coexist with a normal executor/diagnostic. */
#if APP_B2_BENCH
drv_motor_t *subsystem2_bench_motor(void);
drv_encoder_t *subsystem2_bench_encoder(void);
#endif
#endif
