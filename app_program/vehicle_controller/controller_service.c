#include "controller_service.h"

#include <stddef.h>
#include <string.h>

static void refresh_status(controller_service_t *service)
{
    controller_status_t *status = &service->status;

    status->mode = service->controller.mode;
    status->mission = service->controller.mission;
    status->stop_reasons = service->controller.safety.active_reasons;
    status->hardware_faults =
        vehicle_controller_hardware_faults(&service->controller);
    status->authority_generation = service->controller.authority_generation;
}

void controller_line_default_config(controller_line_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    /* 150 mm/s and steer 100 -> inner wheel ~0 reproduce Buddy 3's tuned
     * line-follow bench (base effort 200, gain 2) through Buddy 2's draft
     * executor defaults. */
    config->cruise_mm_s = 150U;
    config->slow_mm_s = 100U;
    config->curvature_per_steer_u1_m = 150000;
    config->request_life_ms = 150U;
}

static void execute(
    controller_service_t *service,
    const controller_action_t *action)
{
    if (action->type == CONTROLLER_ACTION_NONE)
    {
        return;
    }
    service->status.last_action = action->type;

    if (action->type == CONTROLLER_ACTION_STOP_MOTION)
    {
        service->status.stop_commands++;
        if (!service->outputs.stop_motion(service->outputs.context))
        {
            /* The stop was not applied: hold every mover off with a latched
             * motion fault. Not retried here, so a broken output cannot loop. */
            service->status.stop_failures++;
            (void)vehicle_controller_latch_stop(
                &service->controller,
                STOP_REASON_MOTION_FAULT);
        }
    }
}

bool controller_service_init(
    controller_service_t *service,
    const controller_outputs_t *outputs)
{
    if ((service == NULL) || (outputs == NULL) ||
        (outputs->stop_motion == NULL) ||
        ((outputs->drive == NULL) != (outputs->read_line == NULL)))
    {
        return false;
    }
    (void)memset(service, 0, sizeof(*service));
    vehicle_controller_init(&service->controller);
    service->outputs = *outputs;
    controller_line_default_config(&service->line);
    service->initialized = true;
    refresh_status(service);
    return true;
}

static bool apply(
    controller_service_t *service,
    const controller_event_t *event,
    controller_action_t *action)
{
    vehicle_controller_t *controller = &service->controller;

    switch (event->type)
    {
        case CONTROLLER_EVENT_SET_MODE:
            if ((unsigned)event->mode > (unsigned)SYSTEM_MODE_FAULT)
            {
                return false;
            }
            *action = vehicle_controller_set_mode(controller, event->mode);
            return true;

        case CONTROLLER_EVENT_LATCH_STOP:
            if (event->reasons == STOP_REASON_NONE)
            {
                return false;
            }
            *action = vehicle_controller_latch_stop(controller, event->reasons);
            return true;

        case CONTROLLER_EVENT_CLEAR_STOP:
            if (event->reasons == STOP_REASON_NONE)
            {
                return false;
            }
            (void)vehicle_controller_clear_stop(controller, event->reasons);
            return true;

        case CONTROLLER_EVENT_OBSTACLE_DETECTED:
            *action = vehicle_controller_on_obstacle(controller);
            return action->type == CONTROLLER_ACTION_STOP_MOTION;

        case CONTROLLER_EVENT_OBSTACLE_PROGRESS:
            *action = vehicle_controller_obstacle_progress(controller,
                event->mission);
            return controller->mission == event->mission;

        case CONTROLLER_EVENT_OBSTACLE_CLEARED:
            *action = vehicle_controller_obstacle_cleared(controller,
                event->rejoined_line);
            return action->type == CONTROLLER_ACTION_START_LINE_TRACKING;

        case CONTROLLER_EVENT_OBSTACLE_FAILED:
            /* Avoidance gave up: always a latched safe stop, in any mode. */
            *action = vehicle_controller_latch_stop(controller,
                (event->reasons != STOP_REASON_NONE)
                    ? event->reasons
                    : STOP_REASON_RECOVERY_FAILED);
            return true;

        case CONTROLLER_EVENT_COUNT:
        default:
            return false;
    }
}

bool controller_service_handle(
    controller_service_t *service,
    const controller_event_t *event)
{
    controller_action_t action = {0};
    bool applied;

    if ((service == NULL) || !service->initialized)
    {
        return false;
    }
    applied = (event != NULL) && apply(service, event, &action);
    if (applied)
    {
        service->status.events_applied++;
        execute(service, &action);
    }
    else
    {
        service->status.events_rejected++;
    }
    refresh_status(service);
    return applied;
}

void controller_service_status(
    const controller_service_t *service,
    controller_status_t *status)
{
    if ((service != NULL) && (status != NULL))
    {
        *status = service->status;
    }
}

static bool following_line(mission_state_t mission)
{
    return (mission == MISSION_TRACK_LINE) || (mission == MISSION_VERIFY_LINE) ||
        (mission == MISSION_JUNCTION_APPROACH);
}

void controller_service_tick(
    controller_service_t *service,
    system_time_ms_t now_ms)
{
    const vehicle_controller_t *controller;
    controller_line_input_t line = {0, false};
    motion_continuous_request_t request;
    uint32_t speed;

    if ((service == NULL) || !service->initialized ||
        (service->outputs.drive == NULL))
    {
        return;
    }
    controller = &service->controller;
    if ((controller->mode != SYSTEM_MODE_AUTO) ||
        !following_line(controller->mission) ||
        !vehicle_controller_motion_allowed(controller, MOTION_SOURCE_LINE) ||
        !service->outputs.read_line(service->outputs.context, &line))
    {
        return;   /* nothing sent: Buddy 2 stops when the last one expires */
    }

    speed = (line.barcode_active ||
        (controller->mission == MISSION_JUNCTION_APPROACH))
        ? service->line.slow_mm_s : service->line.cruise_mm_s;
    speed = vehicle_controller_apply_speed_caps(controller, speed);

    (void)memset(&request, 0, sizeof(request));
    request.command_id = ++service->next_command_id;
    request.authority_generation = controller->authority_generation;
    request.source = MOTION_SOURCE_LINE;
    request.speed_mm_s = (int32_t)speed;
    /* Buddy 3: steer > 0 = turn right. Curvature > 0 = turn left. */
    request.curvature_u1_m = -(int32_t)line.steer *
        service->line.curvature_per_steer_u1_m;
    request.created_ms = now_ms;
    request.expires_ms = now_ms + service->line.request_life_ms;

    service->status.last_steer = line.steer;
    if (service->outputs.drive(service->outputs.context, &request))
    {
        service->status.drive_requests++;
    }
    else
    {
        service->status.drive_rejected++;
    }
}

bool controller_service_authority(
    const controller_service_t *service,
    motion_source_t source,
    authority_generation_t *generation)
{
    if ((service == NULL) || !service->initialized || (generation == NULL) ||
        (service->controller.mode != SYSTEM_MODE_AUTO) ||
        !vehicle_controller_motion_allowed(&service->controller, source))
    {
        return false;
    }
    *generation = service->controller.authority_generation;
    return true;
}
