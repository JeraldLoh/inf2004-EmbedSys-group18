#ifndef B2_PLATFORM_STUBS_H
#define B2_PLATFORM_STUBS_H
#include "fake_motion_hal.h"
#include <tk/tkernel.h>
extern fake_motion_hal_t *bsp_fake;
extern uint32_t mock_core;
extern uint32_t mock_exception;
extern uint32_t mock_timer_alarm;
extern uint32_t mock_timer_enabled;
extern bool mock_stop_during_arm;
extern uint32_t mock_alarm_setup_delay;
extern bool mock_gpio_owned;
extern unsigned mock_fail_irq;
extern bool mock_clock_step;
extern unsigned mock_lock_entries;
void mock_gpio_edges(uint32_t rising, uint32_t falling);
void mock_fire_irq(unsigned irq);
#endif
