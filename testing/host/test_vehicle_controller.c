#include <assert.h>
#include <stdio.h>

#include "../../app_program/vehicle_controller/vehicle_controller.h"

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

static void start_auto(vehicle_controller_t *controller)
{
    vehicle_controller_init(controller);
    (void)vehicle_controller_set_mode(controller, SYSTEM_MODE_AUTO);
}

static void test_obstacle_mission_full_bypass(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    start_auto(&controller);
    assert(vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_LINE));
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));

    action = vehicle_controller_on_obstacle(&controller);
    assert(action.type == CONTROLLER_ACTION_STOP_MOTION);
    assert(action.stop_reasons == STOP_REASON_NONE); /* nothing latched */
    assert(controller.mode == SYSTEM_MODE_AUTO);
    assert(controller.mission == MISSION_OBSTACLE_STOP);
    assert(vehicle_controller_in_obstacle_mission(&controller));

    /* Buddy 5 owns motion; line following is locked out, but not by a stop. */
    assert(vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_LINE));
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_JUNCTION));
    assert(vehicle_controller_apply_speed_caps(&controller, 200U) == 200U);

    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_START_SCAN);
    action = vehicle_controller_obstacle_progress(&controller, MISSION_PLAN_BYPASS);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_PLAN_BYPASS);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_BYPASS);
    assert(controller.mission == MISSION_BYPASS);
    action = vehicle_controller_obstacle_progress(&controller, MISSION_RECOVERY);
    assert(action.type == CONTROLLER_ACTION_ENTER_RECOVERY);

    action = vehicle_controller_obstacle_cleared(&controller, true);
    assert(action.type == CONTROLLER_ACTION_START_LINE_TRACKING);
    assert(controller.mission == MISSION_VERIFY_LINE);
    assert(!vehicle_controller_in_obstacle_mission(&controller));
    assert(vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_LINE));
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));
}

static void test_obstacle_removed_resumes_tracking(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);

    action = vehicle_controller_obstacle_cleared(&controller, false);
    assert(action.type == CONTROLLER_ACTION_START_LINE_TRACKING);
    assert(controller.mission == MISSION_TRACK_LINE);
}

static void test_obstacle_ignored_when_not_driving(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    vehicle_controller_init(&controller); /* STARTUP, not AUTO */
    action = vehicle_controller_on_obstacle(&controller);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_IDLE);

    start_auto(&controller);
    (void)vehicle_controller_latch_stop(&controller, STOP_REASON_PHYSICAL);
    action = vehicle_controller_on_obstacle(&controller);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_SAFE_STOP);

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    action = vehicle_controller_on_obstacle(&controller); /* already handling one */
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_SCAN);
}

static void test_obstacle_invalid_transitions_rejected(void)
{
    vehicle_controller_t controller;
    controller_action_t action;
    authority_generation_t generation;

    start_auto(&controller);
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_NONE); /* not in an obstacle mission */
    assert(controller.mission == MISSION_TRACK_LINE);
    action = vehicle_controller_obstacle_cleared(&controller, true);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_TRACK_LINE);

    (void)vehicle_controller_on_obstacle(&controller);
    generation = controller.authority_generation;
    action = vehicle_controller_obstacle_progress(&controller, MISSION_BYPASS);
    assert(action.type == CONTROLLER_ACTION_NONE); /* must scan first */
    assert(controller.mission == MISSION_OBSTACLE_STOP);
    assert(controller.authority_generation == generation);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_TRACK_LINE);
    assert(controller.mission == MISSION_OBSTACLE_STOP);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SAFE_STOP);
    assert(controller.mission == MISSION_OBSTACLE_STOP);
}

static void test_obstacle_rescan_and_replan_paths(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_PLAN_BYPASS);

    /* No clear side: rescan (or reverse, then rescan). */
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_START_SCAN);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_PLAN_BYPASS);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_BYPASS);

    /* Emergency stop mid-bypass: re-plan from where the car is. */
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_START_SCAN);

    /* Obstacle gone, but the car already left the line: straight to recovery. */
    action = vehicle_controller_obstacle_progress(&controller, MISSION_RECOVERY);
    assert(action.type == CONTROLLER_ACTION_ENTER_RECOVERY);
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_START_SCAN);
}

static void test_junction_ignored_during_obstacle_mission(void)
{
    vehicle_controller_t controller;
    junction_observation_t junction = {0};
    controller_action_t action;

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_PLAN_BYPASS);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_BYPASS);

    junction.confirmed = true; /* no route: would normally latch a stop */
    junction.left_available = true;
    action = vehicle_controller_on_junction(&controller, &junction, 50U);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mode == SYSTEM_MODE_AUTO);
    assert(controller.mission == MISSION_BYPASS);
    assert(controller.safety.active_reasons == STOP_REASON_NONE);
}

static void test_obstacle_goal_results(void)
{
    vehicle_controller_t controller;
    motion_goal_result_t result = {0};
    controller_action_t action;

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_PLAN_BYPASS);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_BYPASS);

    /* A finished bypass leg must not hand the car back to line following. */
    controller.active_goal = true;
    controller.active_goal_id = 3U;
    result.goal_id = 3U;
    result.authority_generation = controller.authority_generation;
    result.code = MOTION_RESULT_COMPLETED;
    action = vehicle_controller_on_motion_result(&controller, &result);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_BYPASS);
    assert(!controller.active_goal);

    /* A leg that times out is a motion fault: latched safe stop. */
    controller.active_goal = true;
    controller.active_goal_id = 4U;
    result.goal_id = 4U;
    result.code = MOTION_RESULT_TIMEOUT;
    action = vehicle_controller_on_motion_result(&controller, &result);
    assert(action.type == CONTROLLER_ACTION_STOP_MOTION);
    assert((action.stop_reasons & STOP_REASON_MOTION_FAULT) != 0U);
    assert(controller.mission == MISSION_SAFE_STOP);
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));
}

static void test_obstacle_invalidates_older_goals_and_route(void)
{
    vehicle_controller_t controller;
    route_instruction_t route = {0};
    junction_observation_t junction = {0};
    motion_goal_result_t result = {0};
    controller_action_t action;
    authority_generation_t junction_generation;

    start_auto(&controller);
    route.command = ROUTE_COMMAND_LEFT;
    route.expires_ms = 1000U;
    route.valid = true;
    assert(vehicle_controller_receive_route(&controller, &route, 100U));
    junction.confirmed = true;
    junction.left_available = true;
    (void)vehicle_controller_on_junction(&controller, &junction, 200U);
    assert(controller.route_lifecycle == ROUTE_LIFECYCLE_RESERVED);
    controller.active_goal = true;
    controller.active_goal_id = 8U;
    junction_generation = controller.authority_generation;

    (void)vehicle_controller_on_obstacle(&controller);
    assert(controller.route_lifecycle == ROUTE_LIFECYCLE_CANCELLED);

    /* The junction turn's late result is ignored. */
    result.goal_id = 8U;
    result.authority_generation = junction_generation;
    result.code = MOTION_RESULT_COMPLETED;
    action = vehicle_controller_on_motion_result(&controller, &result);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_OBSTACLE_STOP);

    /* Even if an obstacle goal reuses the id, the old generation is stale:
     * a late junction-turn timeout must not latch a stop. */
    controller.active_goal = true;
    controller.active_goal_id = 8U;
    result.code = MOTION_RESULT_TIMEOUT;
    action = vehicle_controller_on_motion_result(&controller, &result);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_OBSTACLE_STOP);
    assert(controller.safety.active_reasons == STOP_REASON_NONE);
}

static void test_latched_stop_blocks_obstacle_calls(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    /* AUTO requested again without clearing the stop: still blocked. */
    start_auto(&controller);
    (void)vehicle_controller_latch_stop(&controller, STOP_REASON_REMOTE);
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);
    action = vehicle_controller_on_obstacle(&controller);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_TRACK_LINE);

    /* A reason latched directly through safety.h mid-mission. */
    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    safety_latch(&controller.safety, STOP_REASON_SENSOR_CRITICAL);
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_NONE);
    action = vehicle_controller_obstacle_cleared(&controller, false);
    assert(action.type == CONTROLLER_ACTION_NONE);
    assert(controller.mission == MISSION_OBSTACLE_STOP);
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));
}

static void test_failed_avoidance_latches_safe_stop(void)
{
    vehicle_controller_t controller;
    controller_action_t action;

    start_auto(&controller);
    (void)vehicle_controller_on_obstacle(&controller);
    (void)vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);

    action = vehicle_controller_latch_stop(&controller, STOP_REASON_RECOVERY_FAILED);
    assert(action.type == CONTROLLER_ACTION_STOP_MOTION);
    assert(controller.mission == MISSION_SAFE_STOP);
    assert(!vehicle_controller_motion_allowed(&controller, MOTION_SOURCE_OBSTACLE));
    assert(vehicle_controller_apply_speed_caps(&controller, 200U) == 0U);
    action = vehicle_controller_obstacle_cleared(&controller, true);
    assert(action.type == CONTROLLER_ACTION_NONE); /* only a reset clears it */
    action = vehicle_controller_obstacle_progress(&controller, MISSION_SCAN);
    assert(action.type == CONTROLLER_ACTION_NONE);

    assert(vehicle_controller_clear_stop(&controller, STOP_REASON_RECOVERY_FAILED));
    (void)vehicle_controller_set_mode(&controller, SYSTEM_MODE_AUTO);
    assert(controller.mission == MISSION_TRACK_LINE);
}

int main(void)
{
    test_no_command_does_not_stop_normal_tracking();
    test_confirmed_junction_without_command_stops();
    test_valid_route_is_reserved_once();
    test_stop_invalidates_old_goal_result();
    test_terrain_cap_and_stop_compose();
    test_obstacle_mission_full_bypass();
    test_obstacle_removed_resumes_tracking();
    test_obstacle_ignored_when_not_driving();
    test_obstacle_invalid_transitions_rejected();
    test_obstacle_rescan_and_replan_paths();
    test_junction_ignored_during_obstacle_mission();
    test_obstacle_goal_results();
    test_obstacle_invalidates_older_goals_and_route();
    test_latched_stop_blocks_obstacle_calls();
    test_failed_avoidance_latches_safe_stop();
    (void)puts("vehicle_controller host tests passed");
    return 0;
}
