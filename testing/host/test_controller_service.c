#include <assert.h>
#include <stdio.h>
#include <string.h>

#include "../../app_program/vehicle_controller/controller_service.h"

typedef struct
{
    int stops;
    bool fail_stop;
} fake_motion_t;

static bool fake_stop(void *context)
{
    fake_motion_t *motion = context;
    motion->stops++;
    return !motion->fail_stop;
}

static void start(controller_service_t *service, fake_motion_t *motion)
{
    controller_outputs_t outputs = {motion, fake_stop, NULL, NULL};
    controller_event_t auto_mode = {CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO,
        STOP_REASON_NONE, MISSION_IDLE, false};

    assert(controller_service_init(service, &outputs));
    assert(controller_service_handle(service, &auto_mode));
    assert(service->status.mission == MISSION_TRACK_LINE);
}

static controller_event_t ev(controller_event_type_t type)
{
    controller_event_t e = {type, SYSTEM_MODE_IDLE, STOP_REASON_NONE,
        MISSION_IDLE, false};
    return e;
}

static controller_event_t progress(mission_state_t mission)
{
    controller_event_t e = ev(CONTROLLER_EVENT_OBSTACLE_PROGRESS);
    e.mission = mission;
    return e;
}

static void test_init_requires_stop_output(void)
{
    controller_service_t service;
    controller_outputs_t none = {NULL, NULL, NULL, NULL};
    controller_event_t e = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);

    assert(!controller_service_init(&service, &none));
    assert(!controller_service_init(&service, NULL));
    assert(!controller_service_init(NULL, &none));
    assert(!controller_service_handle(NULL, &e));
}

static void test_obstacle_bypass_runs_stop_once(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_event_t cleared = ev(CONTROLLER_EVENT_OBSTACLE_CLEARED);
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);
    controller_event_t e;

    start(&service, &motion);
    motion.stops = 0; /* entering AUTO stops first, by design */

    assert(controller_service_handle(&service, &detected));
    assert(motion.stops == 1);
    assert(service.status.mission == MISSION_OBSTACLE_STOP);
    assert(service.status.stop_reasons == STOP_REASON_NONE);

    e = progress(MISSION_SCAN);
    assert(controller_service_handle(&service, &e));
    assert(service.status.last_action == CONTROLLER_ACTION_START_SCAN);
    e = progress(MISSION_PLAN_BYPASS);
    assert(controller_service_handle(&service, &e));
    e = progress(MISSION_BYPASS);
    assert(controller_service_handle(&service, &e));
    e = progress(MISSION_RECOVERY);
    assert(controller_service_handle(&service, &e));
    assert(service.status.last_action == CONTROLLER_ACTION_ENTER_RECOVERY);

    cleared.rejoined_line = true;
    assert(controller_service_handle(&service, &cleared));
    assert(service.status.mission == MISSION_VERIFY_LINE);
    assert(service.status.last_action == CONTROLLER_ACTION_START_LINE_TRACKING);
    assert(motion.stops == 1); /* only the detection stopped the car */
    assert(service.status.events_rejected == 0U);
}

static void test_invalid_events_are_counted_not_applied(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_event_t e;
    uint32_t stops;

    start(&service, &motion);
    stops = service.status.stop_commands;

    e = progress(MISSION_BYPASS); /* not in an obstacle mission */
    assert(!controller_service_handle(&service, &e));
    e = ev(CONTROLLER_EVENT_OBSTACLE_CLEARED);
    assert(!controller_service_handle(&service, &e));
    e = ev(CONTROLLER_EVENT_LATCH_STOP); /* no reason given */
    assert(!controller_service_handle(&service, &e));
    e = ev(CONTROLLER_EVENT_CLEAR_STOP);
    assert(!controller_service_handle(&service, &e));
    e = ev(CONTROLLER_EVENT_SET_MODE);
    e.mode = (system_mode_t)99;
    assert(!controller_service_handle(&service, &e));
    e = ev(CONTROLLER_EVENT_COUNT);
    assert(!controller_service_handle(&service, &e));
    assert(!controller_service_handle(&service, NULL));

    assert(service.status.events_rejected == 7U);
    assert(service.status.stop_commands == stops);
    assert(service.status.mission == MISSION_TRACK_LINE);
}

static void test_obstacle_ignored_when_idle(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_outputs_t outputs = {&motion, fake_stop, NULL, NULL};
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);

    assert(controller_service_init(&service, &outputs));
    assert(!controller_service_handle(&service, &detected));
    assert(motion.stops == 0);
    assert(service.status.mission == MISSION_IDLE);
}

static void test_failed_avoidance_latches_and_stops(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);
    controller_event_t failed = ev(CONTROLLER_EVENT_OBSTACLE_FAILED);
    controller_event_t clear = ev(CONTROLLER_EVENT_CLEAR_STOP);
    controller_event_t auto_mode = ev(CONTROLLER_EVENT_SET_MODE);
    controller_event_t e;

    start(&service, &motion);
    assert(controller_service_handle(&service, &detected));
    e = progress(MISSION_SCAN);
    assert(controller_service_handle(&service, &e));
    motion.stops = 0;

    failed.reasons = STOP_REASON_SENSOR_CRITICAL;
    assert(controller_service_handle(&service, &failed));
    assert(motion.stops == 1);
    assert(service.status.mission == MISSION_SAFE_STOP);
    assert(service.status.stop_reasons == STOP_REASON_SENSOR_CRITICAL);

    /* No reason given: defaults to recovery failed. */
    failed.reasons = STOP_REASON_NONE;
    assert(controller_service_handle(&service, &failed));
    assert((service.status.stop_reasons & STOP_REASON_RECOVERY_FAILED) != 0U);

    /* Restart only after clearing and asking for AUTO again. */
    assert(!controller_service_handle(&service, &detected));
    clear.reasons = STOP_REASON_SENSOR_CRITICAL | STOP_REASON_RECOVERY_FAILED;
    assert(controller_service_handle(&service, &clear));
    auto_mode.mode = SYSTEM_MODE_AUTO;
    assert(controller_service_handle(&service, &auto_mode));
    assert(service.status.mission == MISSION_TRACK_LINE);
    assert(controller_service_handle(&service, &detected));
}

static void test_stop_output_failure_latches_motion_fault(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);

    start(&service, &motion);
    motion.fail_stop = true;
    motion.stops = 0;
    assert(controller_service_handle(&service, &detected));
    assert(motion.stops == 1); /* attempted once, not retried in a loop */
    assert(service.status.stop_failures == 1U);
    assert((service.status.stop_reasons & STOP_REASON_MOTION_FAULT) != 0U);
    assert(service.status.mission == MISSION_SAFE_STOP);
    assert(!vehicle_controller_motion_allowed(&service.controller,
        MOTION_SOURCE_OBSTACLE));
}

static void test_status_copy(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};
    controller_status_t status;

    start(&service, &motion);
    controller_service_status(&service, &status);
    assert(status.mode == SYSTEM_MODE_AUTO);
    assert(status.mission == MISSION_TRACK_LINE);
    assert(status.events_applied == 1U);
    assert(status.hardware_faults == 0U);
    controller_service_status(&service, NULL); /* must not crash */
}

/* ---- Line tracking (controller_service_tick) ---------------------------- */

typedef struct
{
    fake_motion_t motion;
    controller_line_input_t line;
    motion_continuous_request_t last;
    int drives;
    bool reject;
} fake_car_t;

static bool fake_car_stop(void *context)
{
    return fake_stop(&((fake_car_t *)context)->motion);
}

static bool fake_drive(void *context, const motion_continuous_request_t *request)
{
    fake_car_t *car = context;
    car->drives++;
    car->last = *request;
    return !car->reject;
}

static bool fake_read_line(void *context, controller_line_input_t *line)
{
    *line = ((fake_car_t *)context)->line;
    return true;
}

static void start_car(controller_service_t *service, fake_car_t *car)
{
    controller_outputs_t outputs = {car, fake_car_stop, fake_drive, fake_read_line};
    controller_event_t auto_mode = {CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO,
        STOP_REASON_NONE, MISSION_IDLE, false};

    memset(car, 0, sizeof(*car));
    assert(controller_service_init(service, &outputs));
    assert(controller_service_handle(service, &auto_mode));
}

static void test_drive_and_read_line_come_together(void)
{
    controller_service_t service;
    fake_car_t car;
    controller_outputs_t half = {&car, fake_car_stop, fake_drive, NULL};
    controller_outputs_t other = {&car, fake_car_stop, NULL, fake_read_line};

    assert(!controller_service_init(&service, &half));
    assert(!controller_service_init(&service, &other));
}

static void test_tick_without_drive_is_a_no_op(void)
{
    controller_service_t service;
    fake_motion_t motion = {0, false};

    start(&service, &motion);
    controller_service_tick(&service, 100U);
    assert(service.status.drive_requests == 0U);
    controller_service_tick(NULL, 100U); /* must not crash */
}

static void test_tick_turns_steering_into_requests(void)
{
    controller_service_t service;
    fake_car_t car;

    start_car(&service, &car);
    car.line.steer = 40;   /* Buddy 3: turn right */
    controller_service_tick(&service, 1000U);
    assert(car.drives == 1 && service.status.drive_requests == 1U);
    assert(car.last.source == MOTION_SOURCE_LINE);
    assert(car.last.authority_generation == service.controller.authority_generation);
    assert(car.last.speed_mm_s == (int32_t)service.line.cruise_mm_s);
    assert(car.last.curvature_u1_m == -40 * service.line.curvature_per_steer_u1_m);
    assert(car.last.created_ms == 1000U);
    assert(car.last.expires_ms == 1000U + service.line.request_life_ms);
    assert(service.status.last_steer == 40);

    /* Command ids advance; a barcode read slows the car. */
    car.line.barcode_active = true;
    controller_service_tick(&service, 1010U);
    assert(car.last.command_id == 2U);
    assert(car.last.speed_mm_s == (int32_t)service.line.slow_mm_s);

    car.reject = true;
    controller_service_tick(&service, 1020U);
    assert(service.status.drive_rejected == 1U && service.status.drive_requests == 2U);
}

static void test_tick_sends_nothing_unless_following_the_line(void)
{
    controller_service_t service;
    fake_car_t car;
    controller_outputs_t outputs = {&car, fake_car_stop, fake_drive, fake_read_line};
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);
    controller_event_t cleared = ev(CONTROLLER_EVENT_OBSTACLE_CLEARED);
    controller_event_t latch = ev(CONTROLLER_EVENT_LATCH_STOP);

    /* IDLE after init. */
    memset(&car, 0, sizeof(car));
    assert(controller_service_init(&service, &outputs));
    controller_service_tick(&service, 0U);
    assert(car.drives == 0);

    /* Obstacle mission: Buddy 5 owns the motion. */
    start_car(&service, &car);
    assert(controller_service_handle(&service, &detected));
    controller_service_tick(&service, 0U);
    assert(car.drives == 0);

    /* Back on the line with fresh authority. */
    assert(controller_service_handle(&service, &cleared));
    controller_service_tick(&service, 10U);
    assert(car.drives == 1);
    assert(car.last.authority_generation == service.controller.authority_generation);

    /* Latched stop. */
    latch.reasons = STOP_REASON_PHYSICAL;
    assert(controller_service_handle(&service, &latch));
    controller_service_tick(&service, 20U);
    assert(car.drives == 1);

    /* AUTO asked again without clearing the stop: still no line motion. */
    {
        controller_event_t auto_mode = ev(CONTROLLER_EVENT_SET_MODE);
        auto_mode.mode = SYSTEM_MODE_AUTO;
        assert(controller_service_handle(&service, &auto_mode));
        assert(service.controller.mission == MISSION_TRACK_LINE);
        controller_service_tick(&service, 30U);
        assert(car.drives == 1);
    }
}

static void test_authority_gate(void)
{
    controller_service_t service;
    fake_car_t car;
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);
    authority_generation_t gen = 0U;
    controller_outputs_t outputs = {&car, fake_car_stop, fake_drive, fake_read_line};

    memset(&car, 0, sizeof(car));
    assert(controller_service_init(&service, &outputs));
    assert(!controller_service_authority(&service, MOTION_SOURCE_LINE, &gen));

    start_car(&service, &car);
    assert(controller_service_authority(&service, MOTION_SOURCE_LINE, &gen));
    assert(gen == service.controller.authority_generation);
    assert(!controller_service_authority(&service, MOTION_SOURCE_OBSTACLE, &gen));

    assert(controller_service_handle(&service, &detected));
    assert(controller_service_authority(&service, MOTION_SOURCE_OBSTACLE, &gen));
    assert(gen == service.controller.authority_generation);
    assert(!controller_service_authority(&service, MOTION_SOURCE_LINE, &gen));
    assert(!controller_service_authority(&service, MOTION_SOURCE_OBSTACLE, NULL));
    assert(!controller_service_authority(NULL, MOTION_SOURCE_OBSTACLE, &gen));
}

int main(void)
{
    test_drive_and_read_line_come_together();
    test_tick_without_drive_is_a_no_op();
    test_tick_turns_steering_into_requests();
    test_tick_sends_nothing_unless_following_the_line();
    test_authority_gate();
    test_init_requires_stop_output();
    test_obstacle_bypass_runs_stop_once();
    test_invalid_events_are_counted_not_applied();
    test_obstacle_ignored_when_idle();
    test_failed_avoidance_latches_and_stops();
    test_stop_output_failure_latches_motion_fault();
    test_status_copy();
    (void)puts("controller_service host tests passed");
    return 0;
}
