#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../contracts/control_messages.h"
#include "../contracts/hardware_faults.h"
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
    hardware_fault_state_t hardware_faults;
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
bool vehicle_controller_latch_hardware_faults(
    vehicle_controller_t *controller,
    hardware_fault_mask_t faults);
bool vehicle_controller_clear_hardware_faults(
    vehicle_controller_t *controller,
    hardware_fault_mask_t faults);
hardware_fault_mask_t vehicle_controller_hardware_faults(
    const vehicle_controller_t *controller);
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

#endif
