#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../contracts/control_messages.h"
#include "safety.h"

typedef enum
{
    CONTROLLER_ACTION_NONE = 0,
    CONTROLLER_ACTION_STOP_MOTION,
    CONTROLLER_ACTION_START_LINE_TRACKING,
    CONTROLLER_ACTION_EXECUTE_ROUTE,
    CONTROLLER_ACTION_START_SCAN,
    CONTROLLER_ACTION_ENTER_RECOVERY,
    CONTROLLER_ACTION_REPORT_SAFE_STOP
} controller_action_type_t;

typedef struct
{
    controller_action_type_t type;
    authority_generation_t authority_generation;
    route_command_t route_command;
    stop_reason_mask_t stop_reasons;
} controller_action_t;

typedef struct
{
    system_mode_t mode;
    mission_state_t mission;
    terrain_state_t terrain;
    authority_generation_t authority_generation;
    safety_state_t safety;
    route_instruction_t route;
    route_lifecycle_t route_lifecycle;
    goal_id_t active_goal_id;
    bool active_goal;
    uint32_t terrain_speed_cap_mm_s;
} vehicle_controller_t;

void vehicle_controller_init(vehicle_controller_t *controller);
controller_action_t vehicle_controller_set_mode(
    vehicle_controller_t *controller,
    system_mode_t requested_mode);
controller_action_t vehicle_controller_latch_stop(
    vehicle_controller_t *controller,
    stop_reason_mask_t reasons);
bool vehicle_controller_clear_stop(
    vehicle_controller_t *controller,
    stop_reason_mask_t reasons);
bool vehicle_controller_receive_route(
    vehicle_controller_t *controller,
    const route_instruction_t *instruction,
    system_time_ms_t now_ms);
controller_action_t vehicle_controller_on_junction(
    vehicle_controller_t *controller,
    const junction_observation_t *junction,
    system_time_ms_t now_ms);
controller_action_t vehicle_controller_on_motion_result(
    vehicle_controller_t *controller,
    const motion_goal_result_t *result);
void vehicle_controller_update_terrain(
    vehicle_controller_t *controller,
    const terrain_report_t *report);
uint32_t vehicle_controller_apply_speed_caps(
    const vehicle_controller_t *controller,
    uint32_t requested_speed_mm_s);

/* Obstacle handling (Buddy 5). An obstacle is a mission, not a latched stop:
 * the car stops, then motion belongs to MOTION_SOURCE_OBSTACLE while Buddy 5
 * scans and drives around it, and line following resumes when Buddy 5 reports
 * the path clear. No stop reason is latched, so obstacle goals can still move
 * the car. If avoidance fails, latch STOP_REASON_RECOVERY_FAILED (or
 * STOP_REASON_SENSOR_CRITICAL) with vehicle_controller_latch_stop() instead.
 *
 * Missions: OBSTACLE_STOP -> SCAN -> PLAN_BYPASS -> BYPASS -> RECOVERY.
 * Also allowed: PLAN_BYPASS -> SCAN (rescan or reverse), BYPASS -> SCAN and
 * RECOVERY -> SCAN (re-plan after an emergency stop), SCAN -> RECOVERY
 * (obstacle gone after the car had already left the line). */
bool vehicle_controller_in_obstacle_mission(
    const vehicle_controller_t *controller);
/* Accepted only while driving in AUTO with no latched stop and not already in
 * an obstacle mission; returns STOP_MOTION. Otherwise returns NONE. */
controller_action_t vehicle_controller_on_obstacle(
    vehicle_controller_t *controller);
/* Moves to the next obstacle mission. START_SCAN on entering SCAN,
 * ENTER_RECOVERY on entering RECOVERY, otherwise NONE. A transition not
 * listed above is rejected: mission unchanged, NONE returned. */
controller_action_t vehicle_controller_obstacle_progress(
    vehicle_controller_t *controller,
    mission_state_t next);
/* Ends the obstacle mission and returns START_LINE_TRACKING. rejoined_line:
 * the car left the line and has found it again (VERIFY_LINE); false: the
 * obstacle went away and the car never left the line (TRACK_LINE). */
controller_action_t vehicle_controller_obstacle_cleared(
    vehicle_controller_t *controller,
    bool rejoined_line);
/* Who may drive now: during an obstacle mission only MOTION_SOURCE_OBSTACLE,
 * otherwise anyone except MOTION_SOURCE_OBSTACLE. Always false while a stop
 * reason is latched. */
bool vehicle_controller_motion_allowed(
    const vehicle_controller_t *controller,
    motion_source_t source);

#endif
