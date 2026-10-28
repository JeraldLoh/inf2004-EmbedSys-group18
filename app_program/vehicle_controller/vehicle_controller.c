#include "vehicle_controller.h"

#include <stddef.h>
#include <string.h>

static controller_action_t make_action(
    const vehicle_controller_t *controller,
    controller_action_type_t type)
{
    controller_action_t action = {0};

    action.type = type;
    if (controller != NULL)
    {
        action.authority_generation = controller->authority_generation;
        action.stop_reasons = controller->safety.active_reasons;
    }
    return action;
}

static bool route_is_available(
    route_command_t command,
    const junction_observation_t *junction)
{
    bool available = false;

    if (junction != NULL)
    {
        switch (command)
        {
            case ROUTE_COMMAND_LEFT:
                available = junction->left_available;
                break;
            case ROUTE_COMMAND_RIGHT:
                available = junction->right_available;
                break;
            case ROUTE_COMMAND_STRAIGHT:
                available = junction->straight_available;
                break;
            case ROUTE_COMMAND_UTURN:
                available = true;
                break;
            default:
                available = false;
                break;
        }
    }
    return available;
}

void vehicle_controller_init(vehicle_controller_t *controller)
{
    if (controller != NULL)
    {
        (void)memset(controller, 0, sizeof(*controller));
        controller->mode = SYSTEM_MODE_STARTUP;
        controller->mission = MISSION_IDLE;
        controller->terrain = TERRAIN_UNKNOWN;
        controller->authority_generation = 1U;
        controller->route_lifecycle = ROUTE_LIFECYCLE_NONE;
        safety_init(&controller->safety);
        hardware_fault_state_init(&controller->hardware_faults);
    }
}

controller_action_t vehicle_controller_set_mode(
    vehicle_controller_t *controller,
    system_mode_t requested_mode)
{
    controller_action_t action = {0};

    if (controller == NULL)
    {
        return action;
    }

    controller->authority_generation++;
    controller->active_goal = false;
    controller->active_goal_id = 0U;

    if (controller->route_lifecycle == ROUTE_LIFECYCLE_RESERVED)
    {
        controller->route_lifecycle = ROUTE_LIFECYCLE_CANCELLED;
    }

    controller->mode = requested_mode;
    controller->mission = (requested_mode == SYSTEM_MODE_AUTO)
        ? MISSION_TRACK_LINE
        : MISSION_IDLE;

    action = make_action(controller, CONTROLLER_ACTION_STOP_MOTION);
    return action;
}

controller_action_t vehicle_controller_latch_stop(
    vehicle_controller_t *controller,
    stop_reason_mask_t reasons)
{
    controller_action_t action = {0};

    if (controller == NULL)
    {
        return action;
    }

    safety_latch(&controller->safety, reasons);
    controller->authority_generation++;
    controller->active_goal = false;
    controller->active_goal_id = 0U;
    controller->mode = SYSTEM_MODE_STOPPED;
    controller->mission = MISSION_SAFE_STOP;

    if (controller->route_lifecycle == ROUTE_LIFECYCLE_RESERVED)
    {
        controller->route_lifecycle = ROUTE_LIFECYCLE_CANCELLED;
    }

    return make_action(controller, CONTROLLER_ACTION_STOP_MOTION);
}

bool vehicle_controller_clear_stop(
    vehicle_controller_t *controller,
    stop_reason_mask_t reasons)
{
    if (controller == NULL)
    {
        return false;
    }

    safety_clear(&controller->safety, reasons);
    return safety_motion_allowed(&controller->safety);
}

bool vehicle_controller_latch_hardware_faults(
    vehicle_controller_t *controller,
    hardware_fault_mask_t faults)
{
    if ((controller == NULL) || !hardware_fault_mask_is_valid(faults))
    {
        return false;
    }
    hardware_fault_state_latch(&controller->hardware_faults, faults);
    return true;
}

bool vehicle_controller_clear_hardware_faults(
    vehicle_controller_t *controller,
    hardware_fault_mask_t faults)
{
    if ((controller == NULL) || !hardware_fault_mask_is_valid(faults))
    {
        return false;
    }
    hardware_fault_state_clear(&controller->hardware_faults, faults);
    return true;
}

hardware_fault_mask_t vehicle_controller_hardware_faults(
    const vehicle_controller_t *controller)
{
    return (controller == NULL)
        ? 0U
        : hardware_fault_state_read(&controller->hardware_faults);
}

bool vehicle_controller_receive_route(
    vehicle_controller_t *controller,
    const route_instruction_t *instruction,
    system_time_ms_t now_ms)
{
    if ((controller == NULL) || (instruction == NULL) ||
        !instruction->valid ||
        (instruction->command == ROUTE_COMMAND_NONE) ||
        (now_ms > instruction->expires_ms))
    {
        return false;
    }

    if ((controller->route_lifecycle == ROUTE_LIFECYCLE_PENDING) ||
        (controller->route_lifecycle == ROUTE_LIFECYCLE_RESERVED))
    {
        return false;
    }

    controller->route = *instruction;
    controller->route_lifecycle = ROUTE_LIFECYCLE_PENDING;
    return true;
}

controller_action_t vehicle_controller_on_junction(
    vehicle_controller_t *controller,
    const junction_observation_t *junction,
    system_time_ms_t now_ms)
{
    controller_action_t action = {0};

    if ((controller == NULL) || (junction == NULL) ||
        (controller->mode != SYSTEM_MODE_AUTO))
    {
        return action;
    }

    if (!junction->confirmed)
    {
        if (junction->candidate)
        {
            controller->mission = MISSION_JUNCTION_APPROACH;
        }
        return action;
    }

    if ((controller->route_lifecycle != ROUTE_LIFECYCLE_PENDING) ||
        (now_ms > controller->route.expires_ms) ||
        !route_is_available(controller->route.command, junction))
    {
        if ((controller->route_lifecycle == ROUTE_LIFECYCLE_PENDING) &&
            (now_ms > controller->route.expires_ms))
        {
            controller->route_lifecycle = ROUTE_LIFECYCLE_EXPIRED;
        }
        return vehicle_controller_latch_stop(
            controller,
            STOP_REASON_ROUTE_REQUIRED);
    }

    controller->route_lifecycle = ROUTE_LIFECYCLE_RESERVED;
    controller->mission = MISSION_JUNCTION_EXECUTE;
    action = make_action(controller, CONTROLLER_ACTION_EXECUTE_ROUTE);
    action.route_command = controller->route.command;
    return action;
}

controller_action_t vehicle_controller_on_motion_result(
    vehicle_controller_t *controller,
    const motion_goal_result_t *result)
{
    controller_action_t action = {0};

    if ((controller == NULL) || (result == NULL) ||
        !controller->active_goal ||
        (result->goal_id != controller->active_goal_id) ||
        (result->authority_generation != controller->authority_generation))
    {
        return action;
    }

    if (result->code == MOTION_RESULT_COMPLETED)
    {
        controller->active_goal = false;
        controller->active_goal_id = 0U;
        if (controller->route_lifecycle == ROUTE_LIFECYCLE_RESERVED)
        {
            controller->route_lifecycle = ROUTE_LIFECYCLE_CONSUMED;
        }
        controller->mission = MISSION_VERIFY_LINE;
        return make_action(controller, CONTROLLER_ACTION_START_LINE_TRACKING);
    }

    if ((result->code == MOTION_RESULT_TIMEOUT) ||
        (result->code == MOTION_RESULT_ABORTED) ||
        (result->code == MOTION_RESULT_FAULT))
    {
        return vehicle_controller_latch_stop(
            controller,
            STOP_REASON_MOTION_FAULT);
    }

    return action;
}

void vehicle_controller_update_terrain(
    vehicle_controller_t *controller,
    const terrain_report_t *report)
{
    if ((controller != NULL) && (report != NULL))
    {
        controller->terrain = report->state;
        controller->terrain_speed_cap_mm_s = report->speed_cap_mm_s;
    }
}

uint32_t vehicle_controller_apply_speed_caps(
    const vehicle_controller_t *controller,
    uint32_t requested_speed_mm_s)
{
    uint32_t permitted_speed = requested_speed_mm_s;

    if ((controller != NULL) &&
        (controller->terrain_speed_cap_mm_s > 0U) &&
        (controller->terrain_speed_cap_mm_s < permitted_speed))
    {
        permitted_speed = controller->terrain_speed_cap_mm_s;
    }

    if ((controller == NULL) || !safety_motion_allowed(&controller->safety))
    {
        permitted_speed = 0U;
    }
    return permitted_speed;
}
