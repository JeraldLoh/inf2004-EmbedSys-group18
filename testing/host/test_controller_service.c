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

/* ---- Junctions (controller_service_bind_junction) ----------------------- */

typedef struct
{
    fake_car_t car;
    route_instruction_t route;
    bool route_ready;
    junction_observation_t junction;
    bool junction_ready;
    bool probe_requested;
    int probe_begins;
    int probe_ends;
    motion_goal_request_t goals[8];
    int goal_count;
    bool reject_goal;
    motion_result_code_t goal_code;   /* what goal_result reports */
    bool publish_after_first_take;    /* Buddy 3 publishes between the takes */
} fake_junction_t;

static void race_publish(fake_junction_t *f)
{
    if (f->publish_after_first_take)
    {
        f->publish_after_first_take = false;
        f->route_ready = true;      /* route first, as Buddy 3 does */
        f->junction_ready = true;
    }
}

static bool fj_take_route(void *context, route_instruction_t *route)
{
    fake_junction_t *f = context;
    bool ready = f->route_ready;
    if (ready) { *route = f->route; f->route_ready = false; }
    race_publish(f);
    return ready;
}

static bool fj_take_junction(void *context, junction_observation_t *junction)
{
    fake_junction_t *f = context;
    bool ready = f->junction_ready;
    if (ready) { *junction = f->junction; f->junction_ready = false; }
    race_publish(f);
    return ready;
}

static bool fj_probe_requested(void *context)
{
    return ((fake_junction_t *)context)->probe_requested;
}

static bool fj_probe_begin(void *context)
{
    fake_junction_t *f = context;
    if (!f->probe_requested) { return false; }
    f->probe_requested = false;
    f->probe_begins++;
    return true;
}

static bool fj_probe_end(void *context)
{
    ((fake_junction_t *)context)->probe_ends++;
    return true;
}

static bool fj_goal(void *context, const motion_goal_request_t *goal)
{
    fake_junction_t *f = context;
    if (f->reject_goal || (f->goal_count >= 8)) { return false; }
    f->goals[f->goal_count++] = *goal;
    f->goal_code = MOTION_RESULT_PROGRESS;
    return true;
}

static bool fj_goal_result(void *context, goal_id_t goal_id,
    motion_goal_result_t *result)
{
    fake_junction_t *f = context;
    const motion_goal_request_t *last;
    if (f->goal_count == 0) { return false; }
    last = &f->goals[f->goal_count - 1];
    if (last->goal_id != goal_id) { return false; }
    memset(result, 0, sizeof(*result));
    result->goal_id = goal_id;
    result->authority_generation = last->authority_generation;
    result->code = f->goal_code;
    return true;
}

static bool fj_stop(void *context)
{
    return fake_stop(&((fake_junction_t *)context)->car.motion);
}

static bool fj_drive(void *context, const motion_continuous_request_t *request)
{
    return fake_drive(&((fake_junction_t *)context)->car, request);
}

static bool fj_read_line(void *context, controller_line_input_t *line)
{
    return fake_read_line(&((fake_junction_t *)context)->car, line);
}

static const controller_junction_outputs_t *fj_outputs(fake_junction_t *f)
{
    static controller_junction_outputs_t outputs;
    controller_junction_outputs_t o = {NULL, fj_take_route, fj_take_junction,
        fj_probe_requested, fj_probe_begin, fj_probe_end, fj_goal, fj_goal_result};
    o.context = f;
    outputs = o;
    return &outputs;
}

static void start_junctions(controller_service_t *service, fake_junction_t *f)
{
    controller_outputs_t outputs = {f, fj_stop, fj_drive, fj_read_line};
    controller_event_t auto_mode = {CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO,
        STOP_REASON_NONE, MISSION_IDLE, false};

    memset(f, 0, sizeof(*f));
    assert(controller_service_init(service, &outputs));
    assert(controller_service_bind_junction(service, fj_outputs(f)));
    assert(controller_service_handle(service, &auto_mode));
}

/* Buddy 3 stamps on its own clock, far from the controller's. */
static void publish_route(fake_junction_t *f, route_command_t command)
{
    memset(&f->route, 0, sizeof(f->route));
    f->route.command_id = 7U;
    f->route.command = command;
    f->route.decoded_character = 'A';
    f->route.decoded_ms = 900000U;
    f->route.expires_ms = 900000U + 10000U;
    f->route.valid = true;
    f->route_ready = true;
}

static void publish_junction(fake_junction_t *f, bool confirmed, bool straight)
{
    memset(&f->junction, 0, sizeof(f->junction));
    f->junction.candidate = true;
    f->junction.confirmed = confirmed;
    f->junction.left_available = true;
    f->junction.right_available = true;
    f->junction.straight_available = straight;
    f->junction_ready = true;
}

static void test_bind_junction_needs_every_output(void)
{
    controller_service_t service;
    fake_junction_t f;
    fake_motion_t motion = {0, false};
    controller_junction_outputs_t partial;

    start(&service, &motion);   /* no drive/read_line */
    assert(!controller_service_bind_junction(&service, fj_outputs(&f)));

    start_junctions(&service, &f);
    assert(!controller_service_bind_junction(&service, fj_outputs(&f)));  /* twice */

    start_car(&service, &f.car);
    partial = *fj_outputs(&f);
    partial.goal_result = NULL;
    assert(!controller_service_bind_junction(&service, &partial));
    assert(!controller_service_bind_junction(&service, NULL));
    assert(!controller_service_bind_junction(NULL, fj_outputs(&f)));
}

static void test_barcode_left_turn(void)
{
    controller_service_t service;
    fake_junction_t f;
    authority_generation_t gen;
    int drives;

    start_junctions(&service, &f);
    gen = service.controller.authority_generation;
    publish_route(&f, ROUTE_COMMAND_LEFT);
    controller_service_tick(&service, 1000U);
    assert(service.status.routes_accepted == 1U);
    assert(service.controller.route_lifecycle == ROUTE_LIFECYCLE_PENDING);
    assert(service.controller.route.expires_ms == 1000U + 10000U);   /* restamped */
    assert(f.car.drives == 1);   /* still following the line */

    /* Junction (with its route already taken): forward to the centre. */
    publish_junction(&f, true, true);
    controller_service_tick(&service, 2000U);
    assert(service.controller.mission == MISSION_JUNCTION_EXECUTE);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_APPROACH);
    assert(f.goal_count == 1);
    assert(f.goals[0].type == MOTION_GOAL_FORWARD);
    assert(f.goals[0].target_mm == service.junction.approach_mm);
    assert(f.goals[0].source == MOTION_SOURCE_JUNCTION);
    assert(f.goals[0].authority_generation == gen);
    assert(service.controller.active_goal &&
        service.controller.active_goal_id == f.goals[0].goal_id);
    drives = f.car.drives;
    assert(drives == 1);   /* no line request while the goal runs */

    controller_service_tick(&service, 2010U);   /* still running */
    assert(f.goal_count == 1 && f.car.drives == drives);

    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 2500U);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_TURN);
    assert(f.goal_count == 2);
    assert(f.goals[1].type == MOTION_GOAL_PIVOT_LEFT);
    assert(f.goals[1].target_mdeg == service.junction.turn_mdeg);
    assert(f.goals[1].goal_id != f.goals[0].goal_id);

    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 3500U);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_IDLE);
    assert(service.controller.mission == MISSION_VERIFY_LINE);
    assert(service.controller.route_lifecycle == ROUTE_LIFECYCLE_CONSUMED);
    assert(service.status.junctions_done == 1U);
    assert(f.car.drives == drives + 1);   /* line following again, same tick */
    assert(f.car.last.authority_generation == gen);
}

static void test_straight_and_right_and_uturn(void)
{
    static const route_command_t commands[3] = {ROUTE_COMMAND_STRAIGHT,
        ROUTE_COMMAND_RIGHT, ROUTE_COMMAND_UTURN};
    static const motion_goal_type_t turns[3] = {MOTION_GOAL_FORWARD,
        MOTION_GOAL_PIVOT_RIGHT, MOTION_GOAL_PIVOT_LEFT};
    int i;

    for (i = 0; i < 3; ++i)
    {
        controller_service_t service;
        fake_junction_t f;

        start_junctions(&service, &f);
        publish_route(&f, commands[i]);
        publish_junction(&f, true, true);
        controller_service_tick(&service, 100U);   /* route then junction */
        assert(f.goal_count == 1);
        f.goal_code = MOTION_RESULT_COMPLETED;
        controller_service_tick(&service, 200U);
        if (commands[i] == ROUTE_COMMAND_STRAIGHT)
        {
            assert(f.goal_count == 1);
            assert(service.controller.mission == MISSION_VERIFY_LINE);
            continue;
        }
        assert(f.goal_count == 2 && f.goals[1].type == turns[i]);
        assert(f.goals[1].target_mdeg == ((commands[i] == ROUTE_COMMAND_UTURN)
            ? service.junction.uturn_mdeg : service.junction.turn_mdeg));
        f.goal_code = MOTION_RESULT_COMPLETED;
        controller_service_tick(&service, 300U);
        assert(service.controller.mission == MISSION_VERIFY_LINE);
    }
}

/* Buddy 3 publishes route + junction while the tick is between its two
 * takes: the junction must never be applied without its route. */
static void test_route_published_between_takes(void)
{
    controller_service_t service;
    fake_junction_t f;

    start_junctions(&service, &f);
    publish_route(&f, ROUTE_COMMAND_LEFT);
    publish_junction(&f, true, true);
    f.route_ready = false;
    f.junction_ready = false;
    f.publish_after_first_take = true;
    controller_service_tick(&service, 100U);
    controller_service_tick(&service, 110U);
    assert(service.controller.mode == SYSTEM_MODE_AUTO);
    assert(service.controller.mission == MISSION_JUNCTION_EXECUTE);
    assert(f.goal_count == 1 && f.goals[0].type == MOTION_GOAL_FORWARD);
}

static void test_junction_without_route_stops(void)
{
    controller_service_t service;
    fake_junction_t f;

    start_junctions(&service, &f);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    assert(f.goal_count == 0);
    assert(f.car.motion.stops == 2);   /* AUTO entry + the latched stop */
    assert(service.controller.mode == SYSTEM_MODE_STOPPED);
    assert((service.status.stop_reasons & STOP_REASON_ROUTE_REQUIRED) != 0U);
    assert(f.car.drives == 0);
}

static void test_probe_then_default_route(void)
{
    controller_service_t service;
    fake_junction_t f;
    int drives;

    start_junctions(&service, &f);

    /* No barcode: unconfirmed junction plus a probe request. */
    publish_junction(&f, false, false);
    f.probe_requested = true;
    controller_service_tick(&service, 1000U);
    assert(service.controller.mission == MISSION_JUNCTION_APPROACH);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_PROBE_SETTLE);
    drives = f.car.drives;
    assert(drives == 0);   /* settling: no line request */

    controller_service_tick(&service, 1000U + service.junction.settle_ms - 10U);
    assert(f.probe_begins == 0 && f.car.drives == drives);

    controller_service_tick(&service, 1000U + service.junction.settle_ms);
    assert(f.probe_begins == 1);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_PROBE_OUT);
    assert(f.goal_count == 1 && f.goals[0].type == MOTION_GOAL_PIVOT_LEFT);
    assert(f.goals[0].target_mdeg == service.junction.probe_mdeg);

    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 1500U);
    assert(f.goal_count == 2 && f.goals[1].type == MOTION_GOAL_PIVOT_RIGHT);
    assert(f.goals[1].target_mdeg == service.junction.probe_mdeg);
    assert(service.controller.mission == MISSION_JUNCTION_APPROACH);

    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 2000U);
    assert(f.probe_ends == 1);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_PROBE_RESULT);
    assert(f.car.drives == drives);

    /* Buddy 3: it is a T, so RIGHT, then the confirmed junction. */
    publish_route(&f, ROUTE_COMMAND_RIGHT);
    publish_junction(&f, true, false);
    controller_service_tick(&service, 2010U);
    assert(service.controller.mission == MISSION_JUNCTION_EXECUTE);
    assert(f.goal_count == 3 && f.goals[2].type == MOTION_GOAL_FORWARD);
    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 2500U);
    assert(f.goal_count == 4 && f.goals[3].type == MOTION_GOAL_PIVOT_RIGHT);
    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 3000U);
    assert(service.controller.mission == MISSION_VERIFY_LINE);
    assert(f.car.drives == drives + 1);
}

static void test_probe_result_never_comes(void)
{
    controller_service_t service;
    fake_junction_t f;
    uint32_t t;

    start_junctions(&service, &f);
    publish_junction(&f, false, false);
    f.probe_requested = true;
    controller_service_tick(&service, 0U);
    t = service.junction.settle_ms;
    controller_service_tick(&service, t);
    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, t + 10U);
    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, t + 20U);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_PROBE_RESULT);
    controller_service_tick(&service, t + 20U + service.junction.settle_ms);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_IDLE);
    assert((service.status.stop_reasons & STOP_REASON_ROUTE_REQUIRED) != 0U);
}

static void test_junctions_ignored_after_a_turn(void)
{
    controller_service_t service;
    fake_junction_t f;

    start_junctions(&service, &f);
    publish_route(&f, ROUTE_COMMAND_STRAIGHT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    f.goal_code = MOTION_RESULT_COMPLETED;
    controller_service_tick(&service, 200U);
    assert(service.controller.mission == MISSION_VERIFY_LINE);

    /* Re-detected while still on the crossbar: ignored, no stop. */
    publish_junction(&f, true, true);
    controller_service_tick(&service, 200U + service.junction.holdoff_ms - 10U);
    assert(service.status.junctions_ignored == 1U);
    assert(service.controller.mode == SYSTEM_MODE_AUTO);

    /* After the holdoff a junction counts again (no route: stop). */
    publish_junction(&f, true, true);
    controller_service_tick(&service, 200U + service.junction.holdoff_ms);
    assert(service.status.junctions_seen == 2U);
    assert((service.status.stop_reasons & STOP_REASON_ROUTE_REQUIRED) != 0U);
}

static void test_goal_failures(void)
{
    controller_service_t service;
    fake_junction_t f;

    /* Wheels stalled: a motion fault. */
    start_junctions(&service, &f);
    publish_route(&f, ROUTE_COMMAND_LEFT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    f.goal_code = MOTION_RESULT_TIMEOUT;
    controller_service_tick(&service, 200U);
    assert((service.status.stop_reasons & STOP_REASON_MOTION_FAULT) != 0U);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_IDLE);

    /* Goal refused: an operator-clearable stop. */
    start_junctions(&service, &f);
    f.reject_goal = true;
    publish_route(&f, ROUTE_COMMAND_LEFT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    assert(service.status.stop_reasons == STOP_REASON_ROUTE_REQUIRED);

    /* Cancelled mid-probe: probe released, clearable stop. */
    start_junctions(&service, &f);
    publish_junction(&f, false, false);
    f.probe_requested = true;
    controller_service_tick(&service, 0U);
    controller_service_tick(&service, service.junction.settle_ms);
    f.goal_code = MOTION_RESULT_CANCELLED;
    controller_service_tick(&service, service.junction.settle_ms + 10U);
    assert(f.probe_ends == 1);
    assert(service.status.stop_reasons == STOP_REASON_ROUTE_REQUIRED);
}

static void test_stop_or_obstacle_abandons_the_step(void)
{
    controller_service_t service;
    fake_junction_t f;
    controller_event_t latch = ev(CONTROLLER_EVENT_LATCH_STOP);
    controller_event_t detected = ev(CONTROLLER_EVENT_OBSTACLE_DETECTED);

    /* STOP mid-probe: probe released, no extra stop reason. */
    start_junctions(&service, &f);
    publish_junction(&f, false, false);
    f.probe_requested = true;
    controller_service_tick(&service, 0U);
    controller_service_tick(&service, service.junction.settle_ms);
    assert(f.probe_begins == 1);
    latch.reasons = STOP_REASON_PHYSICAL;
    assert(controller_service_handle(&service, &latch));
    controller_service_tick(&service, service.junction.settle_ms + 10U);
    assert(f.probe_ends == 1);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_IDLE);
    assert(service.status.stop_reasons == STOP_REASON_PHYSICAL);

    /* Obstacle mid-turn: Buddy 5 takes over; events dropped meanwhile. */
    start_junctions(&service, &f);
    publish_route(&f, ROUTE_COMMAND_LEFT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    assert(controller_service_handle(&service, &detected));
    controller_service_tick(&service, 110U);
    assert(service.status.junction_phase == CONTROLLER_JUNCTION_IDLE);
    assert(service.controller.route_lifecycle == ROUTE_LIFECYCLE_CANCELLED);
    publish_route(&f, ROUTE_COMMAND_RIGHT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 120U);
    assert(!f.route_ready && !f.junction_ready);   /* taken and dropped */
    assert(service.status.routes_accepted == 1U);
    assert(service.controller.mode == SYSTEM_MODE_AUTO);
    assert(f.goal_count == 1);
}

static void test_events_dropped_outside_auto(void)
{
    controller_service_t service;
    fake_junction_t f;
    controller_outputs_t outputs = {&f, fj_stop, fj_drive, fj_read_line};
    controller_event_t auto_mode = ev(CONTROLLER_EVENT_SET_MODE);

    memset(&f, 0, sizeof(f));
    assert(controller_service_init(&service, &outputs));
    assert(controller_service_bind_junction(&service, fj_outputs(&f)));
    publish_route(&f, ROUTE_COMMAND_LEFT);
    publish_junction(&f, true, true);
    controller_service_tick(&service, 100U);
    assert(!f.route_ready && !f.junction_ready);
    assert(service.controller.route_lifecycle == ROUTE_LIFECYCLE_NONE);

    auto_mode.mode = SYSTEM_MODE_AUTO;
    assert(controller_service_handle(&service, &auto_mode));
    controller_service_tick(&service, 110U);
    assert(f.goal_count == 0 && service.controller.mode == SYSTEM_MODE_AUTO);
}

int main(void)
{
    test_bind_junction_needs_every_output();
    test_barcode_left_turn();
    test_straight_and_right_and_uturn();
    test_route_published_between_takes();
    test_junction_without_route_stops();
    test_probe_then_default_route();
    test_probe_result_never_comes();
    test_junctions_ignored_after_a_turn();
    test_goal_failures();
    test_stop_or_obstacle_abandons_the_step();
    test_events_dropped_outside_auto();
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
