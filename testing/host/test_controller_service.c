#include <assert.h>
#include <stdio.h>

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
    controller_outputs_t outputs = {motion, fake_stop};
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
    controller_outputs_t none = {NULL, NULL};
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
    controller_outputs_t outputs = {&motion, fake_stop};
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

int main(void)
{
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
