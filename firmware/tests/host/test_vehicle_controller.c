#include <assert.h>
#include <stdio.h>

#include "../../app/vehicle_controller.h"

static void test_no_command_does_not_stop_normal_tracking(void)
{
    vehicle_controller_t controller;
    junction_observation_t observation = {0};
    controller_action_t action;

    vehicle_controller_init(&controller);
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);

    action = vehicle_controller_on_junction(&controller, &observation, 10U);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mode == SYSTEM_MODE_AUTO);
    assert(controller.mission == MISSION_TRACK_LINE);
}

static void test_confirmed_junction_without_command_stops(void)
{
    vehicle_controller_t controller;
    junction_observation_t junction = {0};
    controller_action_t action;

    vehicle_controller_init(&controller);
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);
    junction.confirmed = true;
    junction.left_available = true;

    action = vehicle_controller_on_junction(&controller, &junction, 20U);
    assert(action.type == CONTROLLER_ACTION_STOP_MOTION);
    assert((action.stop_reasons & STOP_REASON_ROUTE_REQUIRED) != 0U);
    assert(controller.mode == SYSTEM_MODE_STOPPED);
}

static void test_valid_route_is_reserved_once(void)
{
    vehicle_controller_t controller;
    route_instruction_t route = {0};
    junction_observation_t junction = {0};
    controller_action_t action;

    vehicle_controller_init(&controller);
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);
    route.command_id = 7U;
    route.command = ROUTE_COMMAND_LEFT;
    route.decoded_character = 'A';
    route.decoded_ms = 100U;
    route.expires_ms = 1000U;
    route.valid = true;
    assert(vehicle_controller_receive_route(&controller, &route, 100U));
    assert(!vehicle_controller_receive_route(&controller, &route, 101U));

    junction.confirmed = true;
    junction.left_available = true;
    action = vehicle_controller_on_junction(&controller, &junction, 200U);
    assert(action.type == CONTROLLER_ACTION_EXECUTE_ROUTE);
    assert(action.route_command == ROUTE_COMMAND_LEFT);
    assert(controller.route_lifecycle == ROUTE_LIFECYCLE_RESERVED);
}

static void test_stop_invalidates_old_goal_result(void)
{
    vehicle_controller_t controller;
    motion_goal_result_t result = {0};
    controller_action_t action;
    authority_generation_t old_generation;

    vehicle_controller_init(&controller);
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);
    controller.active_goal = true;
    controller.active_goal_id = 9U;
    old_generation = controller.authority_generation;

    (void)vehicle_controller_latch_stop(
        &controller,
        STOP_REASON_PHYSICAL);

    result.goal_id = 9U;
    result.authority_generation = old_generation;
    result.code = MOTION_RESULT_COMPLETED;
    action = vehicle_controller_on_motion_result(&controller, &result);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_SAFE_STOP);
}

static void test_terrain_cap_and_stop_compose(void)
{
    vehicle_controller_t controller;
    terrain_report_t terrain = {0};

    vehicle_controller_init(&controller);
    terrain.state = TERRAIN_ASCENDING;
    terrain.speed_cap_mm_s = 120U;
    vehicle_controller_update_terrain(&controller, &terrain);
    assert(vehicle_controller_apply_speed_caps(&controller, 200U) == 120U);

    (void)vehicle_controller_latch_stop(
        &controller,
        STOP_REASON_OBSTACLE);
    assert(vehicle_controller_apply_speed_caps(&controller, 200U) == 0U);
}

int main(void)
{
    test_no_command_does_not_stop_normal_tracking();
    test_confirmed_junction_without_command_stops();
    test_valid_route_is_reserved_once();
    test_stop_invalidates_old_goal_result();
    test_terrain_cap_and_stop_compose();
    (void)puts("vehicle_controller host tests passed");
    return 0;
}
