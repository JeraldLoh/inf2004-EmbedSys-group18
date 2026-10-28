#include "fake_motion_hal.h"
#include "b2_trial.h"
#include "b2_gesture.h"
#include "../project_config/gpio_dispatcher.h"
#include <assert.h>
#include <stdio.h>

typedef struct
{
    fake_motion_hal_t fake;
    bool allowed;
    bool enable_ok;
    uint32_t enabled;
    uint32_t acked;
    unsigned order[4];
    unsigned calls;
    gpio_dispatcher_t *dispatcher;
} dispatch_test_t;
static uintptr_t guard_enter(void *p)
{ dispatch_test_t *d = p; motion_hal_t h = fake_motion_hal(&d->fake); return h.enter(h.context); }
static void guard_leave(void *p, uintptr_t k)
{ dispatch_test_t *d = p; motion_hal_t h = fake_motion_hal(&d->fake); h.leave(h.context, k); }
static bool allow(void *p) { return ((dispatch_test_t*)p)->allowed; }
static bool edges_enable(void *p, uint32_t mask, bool enabled)
{
    dispatch_test_t *d = p;
    assert(d->fake.locked);
    if (!d->enable_ok) { return false; }
    if (enabled) { d->enabled |= mask; } else { d->enabled &= ~mask; }
    return true;
}
static void edges_ack(void *p, uint32_t r, uint32_t f)
{ dispatch_test_t *d = p; assert(d->fake.locked); d->acked |= r | f; }
static void urgent(void *p, uint32_t r, uint32_t f, uint32_t time)
{
    dispatch_test_t *d = p;
    assert(!d->fake.locked && r == 0U && f == 2U && time == 12U);
    d->order[d->calls++] = 1U;
    assert(!gpio_dispatcher_detach(d->dispatcher, 2U));
}
static void ordinary(void *p, uint32_t r, uint32_t f, uint32_t time)
{
    dispatch_test_t *d = p;
    assert(!d->fake.locked && r == 1U && f == 0U && time == 12U);
    d->order[d->calls++] = 2U;
}
static void test_dispatcher(void)
{
    dispatch_test_t d = {0};
    gpio_dispatcher_t dispatcher = {0};
    gpio_dispatch_io_t io = {&d, guard_enter, guard_leave, allow, edges_enable, edges_ack};
    fake_motion_init(&d.fake);
    d.allowed = true; d.enable_ok = true; d.dispatcher = &dispatcher;
    assert(gpio_dispatcher_init(&dispatcher, &io));
    assert(gpio_dispatcher_attach(&dispatcher, 1U, ordinary, &d, false));
    assert(gpio_dispatcher_attach(&dispatcher, 2U, urgent, &d, true));
    assert(!gpio_dispatcher_attach(&dispatcher, 1U, ordinary, &d, false));
    d.acked = 0U;
    gpio_dispatcher_dispatch(&dispatcher, 1U | 8U, 2U | 16U, 12U);
    assert(d.calls == 2U && d.order[0] == 1U && d.order[1] == 2U);
    assert(d.acked == 3U); /* Unregistered events remain untouched. */
    d.allowed = false;
    assert(!gpio_dispatcher_detach(&dispatcher, 1U));
    d.allowed = true;
    d.enable_ok = false;
    assert(!gpio_dispatcher_detach(&dispatcher, 1U));
    assert(d.enabled == 3U);
    assert(!gpio_dispatcher_attach(&dispatcher, 4U, ordinary, &d, false));
    d.enable_ok = true;
    assert(gpio_dispatcher_detach(&dispatcher, 1U));
    assert(d.enabled == 2U);
}

typedef struct { bool pressed; bool pending; bool armed; bool fail; bool race; drv_motor_t *motor; } trial_io_t;
static bool pressed(void *p) { return ((trial_io_t*)p)->pressed; }
static bool pending(void *p) { return ((trial_io_t*)p)->pending; }
static bool cutoff(void *p, uint32_t duration)
{
    trial_io_t *io = p;
    assert(duration <= B2_TRIAL_MAX_US);
    if (io->fail) { return false; }
    io->armed = true;
    if (io->race) { assert(drv_motor_stop(io->motor) == MOTION_OK); }
    return true;
}
static void cancel(void *p) { ((trial_io_t*)p)->armed = false; }
static void test_trials(void)
{
    fake_motion_hal_t fake;
    drv_motor_t motor = {0};
    drv_encoder_t encoder = {0};
    motion_hal_t hal;
    vehicle_controller_t controller;
    b2_trial_t trial;
    b2_trial_result_t result;
    trial_io_t port = {0};
    b2_trial_io_t io = {&port, pressed, pending, cutoff, cancel};
    fake_motion_init(&fake); hal = fake_motion_hal(&fake); port.motor = &motor;
    assert(drv_motor_init(&motor, &hal, 6249U) == MOTION_OK);
    assert(drv_encoder_init(&encoder, &hal) == MOTION_OK);
    vehicle_controller_init(&controller);
    assert(b2_trial_init(&trial, &motor, &encoder, &controller, &io));
    assert(b2_trial_begin(&trial, 151, 0, 250000U) == MOTION_INVALID);
    assert(b2_trial_begin(&trial, 150, 0, 500001U) == MOTION_INVALID);
    port.pressed = true;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    port.pressed = false; port.fail = true;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    fake_assert_safe(&fake);
    port.fail = false;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_OK);
    assert(port.armed);
    assert(b2_trial_begin(&trial, 0, 150, 250000U) == MOTION_STATE);
    drv_encoder_on_edges(&encoder, (uint32_t)1U << 27, 0U, 1U);
    fake.now_us = 250000U;
    b2_trial_service(&trial, STOP_REASON_NONE, false); /* Task deadline fallback. */
    fake_assert_safe(&fake);
    assert(!port.armed);
    assert(b2_trial_begin(&trial, 0, 150, 250000U) == MOTION_STATE);
    assert(b2_trial_take_result(&trial, &result));
    assert(result.state == B2_TRIAL_TIME_ENDED && result.left_edges == 1U);
    assert(!b2_trial_take_result(&trial, &result));
    assert(b2_trial_begin(&trial, -150, 150, 250000U) == MOTION_OK);
    b2_trial_service(&trial, STOP_REASON_PHYSICAL, true); /* Physical wins. */
    assert(b2_trial_take_result(&trial, &result));
    assert(result.state == B2_TRIAL_STOPPED);
    assert(controller.safety.active_reasons & STOP_REASON_PHYSICAL);
    (void)vehicle_controller_latch_stop(&controller, STOP_REASON_SENSOR_CRITICAL);
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    assert(controller.safety.active_reasons & STOP_REASON_SENSOR_CRITICAL);
    (void)vehicle_controller_clear_stop(&controller, STOP_REASON_SENSOR_CRITICAL);
    port.race = true;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_INHIBITED);
    fake_assert_safe(&fake);
    port.race = false;
    fake.now_us = UINT32_MAX - 100000U;
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_OK);
    fake.now_us = 149999U;
    b2_trial_service(&trial, STOP_REASON_NONE, false);
    assert(b2_trial_take_result(&trial, &result));
    assert(result.state == B2_TRIAL_TIME_ENDED);
    assert(b2_trial_begin(&trial, 150, 0, 250000U) == MOTION_OK);
    fake.clock_valid = false;
    b2_trial_service(&trial, STOP_REASON_NONE, false);
    assert(b2_trial_take_result(&trial, &result));
    assert(result.state == B2_TRIAL_STOPPED && !result.counts_valid);
    fake_assert_safe(&fake);
}

static void test_gestures(void)
{
    b2_gesture_t g = {0};
    assert(b2_gesture_update(&g, 0U, true, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 2000000U, true, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 2000001U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 2030001U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 2040000U, true, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 3040000U, true, false, false) == B2_UI_CONFIRM);
    assert(b2_gesture_update(&g, 3040001U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 3070001U, false, false, false) == B2_UI_RUN);
    assert(b2_gesture_update(&g, 3080001U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 3100001U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 3110001U, true, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 4110001U, true, false, false) == B2_UI_CONFIRM);
    assert(b2_gesture_update(&g, 4120001U, false, true, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 4160001U, false, false, false) == B2_UI_NONE);
    assert(g.phase == 1U); /* STOP discarded the confirmed action. */
    assert(b2_gesture_update(&g, 4170001U, true, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 5170001U, true, false, false) == B2_UI_CONFIRM);
    assert(b2_gesture_update(&g, 9170001U, true, false, false) == B2_UI_NONE);
    assert(g.phase == 0U); /* Confirmation expires after 5 s. */
    assert(b2_gesture_update(&g, 9170002U, false, false, false) == B2_UI_NONE);
    assert(b2_gesture_update(&g, 9200002U, false, false, false) == B2_UI_NONE);
}
int main(void)
{
    test_dispatcher(); test_trials(); test_gestures();
    (void)puts("GPIO ordering/ownership, bounded trials/controller/results, manual gesture checks passed");
    return 0;
}
