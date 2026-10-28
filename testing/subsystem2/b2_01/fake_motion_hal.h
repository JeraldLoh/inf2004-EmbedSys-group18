#ifndef B2_FAKE_MOTION_HAL_H
#define B2_FAKE_MOTION_HAL_H

#include "drv_encoder.h"

#include <stddef.h>

#define FAKE_TRACE_CAPACITY 1024U
typedef struct
{
    char operation;
    uint8_t pin;
    uint32_t value;
    bool success;
} fake_trace_t;

typedef struct
{
    bool locked;
    bool pin_pwm[32];
    bool output[32];
    bool latch[32];
    bool inputs[32];
    bool frozen_high[32];
    bool enabled[16];
    uint16_t compare[32];
    uint16_t wrap[16];
    uint32_t now_us;
    bool clock_valid;
    unsigned operations;
    unsigned fail_at;
    unsigned fail_from;
    fake_trace_t trace[FAKE_TRACE_CAPACITY];
    size_t trace_count;
    unsigned inject_at;
    bool stop_before_enter;
    bool pending_stop;
    drv_motor_t *stop_motor;
    bool pending_edges;
    uint32_t rising;
    uint32_t falling;
    uint32_t edge_us;
    motion_edge_handler_t handler;
    void *handler_user;
    bool subscribed;
    unsigned detach_calls;
    unsigned unrelated_sonar_events;
} fake_motion_hal_t;

void fake_motion_init(fake_motion_hal_t *fake);
motion_hal_t fake_motion_hal(fake_motion_hal_t *fake);
bool fake_pin_high(const fake_motion_hal_t *fake, uint8_t pin);
void fake_assert_safe(const fake_motion_hal_t *fake);
void fake_arm(drv_motor_t *motor, uint32_t generation);
void fake_dispatch(fake_motion_hal_t *fake, uint32_t rising,
    uint32_t falling, uint32_t when_us);

#endif
