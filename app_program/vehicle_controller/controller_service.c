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
        (outputs->stop_motion == NULL))
    {
        return false;
    }
    (void)memset(service, 0, sizeof(*service));
    vehicle_controller_init(&service->controller);
    service->outputs = *outputs;
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
