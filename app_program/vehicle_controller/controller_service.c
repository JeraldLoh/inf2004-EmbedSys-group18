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

/* Line-follow cruise speed; CAR_RUN may lower it with -D for floor tuning. */
#ifndef CONTROLLER_LINE_CRUISE_MM_S
#define CONTROLLER_LINE_CRUISE_MM_S 150U
#endif

void controller_line_default_config(controller_line_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    /* 150 mm/s and steer 100 -> inner wheel ~0 reproduce Buddy 3's tuned
     * (since removed) line-follow bench (base effort 200, gain 2) through Buddy 2's draft
     * executor defaults: curvature 2 / track (112 mm) = 17.8 1/m at 100. */
    config->cruise_mm_s = CONTROLLER_LINE_CRUISE_MM_S;
    config->slow_mm_s = 100U;
    config->curvature_per_steer_u1_m = 178000;
    config->request_life_ms = 150U;
}

void controller_junction_default_config(controller_junction_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    /* approach_mm: distance from the line sensors back to the wheel axle, so
     * the car pivots about the junction centre. A guess: measure it. The
     * probe angle is Buddy 3's (~10 deg). settle_ms covers the last line
     * request expiring (150 ms) plus the car coming to rest. */
    config->approach_mm = 50;
    config->turn_mdeg = 90000;
    config->uturn_mdeg = 180000;
    config->probe_mdeg = 10000;
    config->goal_speed_mm_s = 100U;
    config->goal_timeout_ms = 4000U;
    config->settle_ms = 250U;
    config->holdoff_ms = 500U;
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
    controller_junction_default_config(&service->junction);
    /* Junction goal ids sit in their own range, clear of Buddy 5's. */
    service->next_goal_id = 0x80000000UL;
    service->initialized = true;
    refresh_status(service);
    return true;
}

bool controller_service_bind_junction(
    controller_service_t *service,
    const controller_junction_outputs_t *outputs)
{
    if ((service == NULL) || !service->initialized || (outputs == NULL) ||
        service->junction_bound || (service->outputs.drive == NULL) ||
        (outputs->take_route == NULL) || (outputs->take_junction == NULL) ||
        (outputs->probe_requested == NULL) || (outputs->probe_begin == NULL) ||
        (outputs->probe_end == NULL) || (outputs->goal == NULL) ||
        (outputs->goal_result == NULL))
    {
        return false;
    }
    service->junction_out = *outputs;
    service->junction_bound = true;
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

/* ---- Junctions ---------------------------------------------------------- */

typedef enum
{
    GOAL_RUNNING = 0,
    GOAL_DONE,
    GOAL_FAILED
} goal_state_t;

static bool time_reached(system_time_ms_t now, system_time_ms_t deadline)
{
    return (int32_t)(now - deadline) >= 0;
}

static void set_phase(
    controller_service_t *service,
    controller_junction_phase_t phase,
    system_time_ms_t now_ms)
{
    service->junction_phase = phase;
    service->phase_since_ms = now_ms;
    service->status.junction_phase = phase;
}

static bool probing(const controller_service_t *service)
{
    return (service->junction_phase == CONTROLLER_JUNCTION_PROBE_OUT) ||
        (service->junction_phase == CONTROLLER_JUNCTION_PROBE_BACK);
}

/* The step still belongs to the mission it was started in. Anything else
 * (STOP, obstacle, mode change) has taken the car away. */
static bool phase_still_valid(const controller_service_t *service)
{
    const vehicle_controller_t *controller = &service->controller;

    if ((controller->mode != SYSTEM_MODE_AUTO) ||
        !vehicle_controller_motion_allowed(controller, MOTION_SOURCE_JUNCTION))
    {
        return false;
    }
    switch (service->junction_phase)
    {
        case CONTROLLER_JUNCTION_PROBE_SETTLE:
        case CONTROLLER_JUNCTION_PROBE_OUT:
        case CONTROLLER_JUNCTION_PROBE_BACK:
        case CONTROLLER_JUNCTION_PROBE_RESULT:
            return controller->mission == MISSION_JUNCTION_APPROACH;
        case CONTROLLER_JUNCTION_APPROACH:
        case CONTROLLER_JUNCTION_TURN:
            return controller->mission == MISSION_JUNCTION_EXECUTE;
        case CONTROLLER_JUNCTION_IDLE:
        default:
            return true;
    }
}

/* Leave the step without touching the controller (it already moved on). */
static void junction_abort(controller_service_t *service, system_time_ms_t now_ms)
{
    if (probing(service))
    {
        (void)service->junction_out.probe_end(service->junction_out.context);
    }
    set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);
}

static void junction_fail(
    controller_service_t *service,
    stop_reason_mask_t reasons,
    system_time_ms_t now_ms)
{
    controller_action_t action;

    junction_abort(service, now_ms);
    action = vehicle_controller_latch_stop(&service->controller, reasons);
    execute(service, &action);
}

static bool submit_goal(
    controller_service_t *service,
    motion_goal_type_t type,
    int32_t mm,
    int32_t mdeg)
{
    motion_goal_request_t *goal = &service->junction_goal;

    (void)memset(goal, 0, sizeof(*goal));
    goal->goal_id = ++service->next_goal_id;
    if (goal->goal_id == 0U)
    {
        goal->goal_id = ++service->next_goal_id;   /* 0 is "no goal" */
    }
    goal->authority_generation = service->controller.authority_generation;
    goal->source = MOTION_SOURCE_JUNCTION;
    goal->type = type;
    goal->target_mm = mm;
    goal->target_mdeg = mdeg;
    goal->max_speed_mm_s = service->junction.goal_speed_mm_s;
    goal->timeout_ms = service->junction.goal_timeout_ms;
    if (!service->junction_out.goal(service->junction_out.context, goal))
    {
        return false;
    }
    service->controller.active_goal = true;
    service->controller.active_goal_id = goal->goal_id;
    return true;
}

static goal_state_t poll_goal(
    controller_service_t *service,
    motion_goal_result_t *result)
{
    goal_id_t id = service->junction_goal.goal_id;

    if (!service->junction_out.goal_result(service->junction_out.context, id,
            result) || (result->goal_id != id))
    {
        result->code = MOTION_RESULT_FAULT;   /* the executor lost it */
        return GOAL_FAILED;
    }
    if ((result->code == MOTION_RESULT_ACCEPTED) ||
        (result->code == MOTION_RESULT_PROGRESS))
    {
        return GOAL_RUNNING;
    }
    return (result->code == MOTION_RESULT_COMPLETED) ? GOAL_DONE : GOAL_FAILED;
}

static void goal_failed(
    controller_service_t *service,
    const motion_goal_result_t *result,
    system_time_ms_t now_ms)
{
    /* The wheels could not do it: a fault. Otherwise the goal was stopped
     * from outside (STOP racing this tick): a stop the operator can clear. */
    junction_fail(service,
        ((result->code == MOTION_RESULT_TIMEOUT) ||
         (result->code == MOTION_RESULT_FAULT))
            ? STOP_REASON_MOTION_FAULT : STOP_REASON_ROUTE_REQUIRED,
        now_ms);
}

/* An intermediate goal finished: the controller only sees the last one. */
static void goal_step_done(controller_service_t *service)
{
    service->controller.active_goal = false;
    service->controller.active_goal_id = 0U;
}

static void start_turn(controller_service_t *service, system_time_ms_t now_ms)
{
    motion_goal_result_t done = {0};
    route_command_t command = service->junction_route;
    int32_t mdeg = 0;

    if ((command == ROUTE_COMMAND_LEFT) || (command == ROUTE_COMMAND_RIGHT))
    {
        mdeg = service->junction.turn_mdeg;
    }
    else if (command == ROUTE_COMMAND_UTURN)
    {
        mdeg = service->junction.uturn_mdeg;
    }
    if (mdeg == 0)
    {
        return;   /* STRAIGHT: the approach was the whole route */
    }
    goal_step_done(service);
    if (submit_goal(service,
            (command == ROUTE_COMMAND_RIGHT) ? MOTION_GOAL_PIVOT_RIGHT
                                             : MOTION_GOAL_PIVOT_LEFT,
            0, mdeg))
    {
        set_phase(service, CONTROLLER_JUNCTION_TURN, now_ms);
    }
    else
    {
        done.code = MOTION_RESULT_REJECTED;
        goal_failed(service, &done, now_ms);
    }
}

static void finish_route(
    controller_service_t *service,
    const motion_goal_result_t *result,
    system_time_ms_t now_ms)
{
    controller_action_t action;

    action = vehicle_controller_on_motion_result(&service->controller, result);
    execute(service, &action);
    service->status.junctions_done++;
    service->holdoff = true;
    service->holdoff_until_ms = now_ms + service->junction.holdoff_ms;
    set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);
}

static void start_route(
    controller_service_t *service,
    route_command_t command,
    system_time_ms_t now_ms)
{
    motion_goal_result_t rejected = {0};

    service->junction_route = command;
    if (submit_goal(service, MOTION_GOAL_FORWARD, service->junction.approach_mm, 0))
    {
        set_phase(service, CONTROLLER_JUNCTION_APPROACH, now_ms);
    }
    else
    {
        rejected.code = MOTION_RESULT_REJECTED;
        goal_failed(service, &rejected, now_ms);
    }
}

/* Take Buddy 3's latest junction and route and hand them to the controller. */
static void junction_poll(controller_service_t *service, system_time_ms_t now_ms)
{
    const controller_junction_outputs_t *out = &service->junction_out;
    vehicle_controller_t *controller = &service->controller;
    junction_observation_t junction;
    route_instruction_t route;
    controller_action_t action;
    bool have_junction;
    bool have_route;

    /* Junction first: its route was published before it. */
    have_junction = out->take_junction(out->context, &junction);
    have_route = out->take_route(out->context, &route);

    if ((controller->mode != SYSTEM_MODE_AUTO) ||
        vehicle_controller_in_obstacle_mission(controller))
    {
        return;   /* dropped: stale by the time line following resumes */
    }

    if (have_route)
    {
        /* Buddy 3 stamps routes on its own clock: keep the lifetime only. */
        route.expires_ms = now_ms + (route.expires_ms - route.decoded_ms);
        route.decoded_ms = now_ms;
        if (vehicle_controller_receive_route(controller, &route, now_ms))
        {
            service->status.routes_accepted++;
            service->status.last_route = route.command;
        }
        else
        {
            service->status.routes_rejected++;
        }
    }

    if (have_junction)
    {
        if (service->holdoff && !time_reached(now_ms, service->holdoff_until_ms))
        {
            service->status.junctions_ignored++;
        }
        else
        {
            service->holdoff = false;
            service->status.junctions_seen++;
            action = vehicle_controller_on_junction(controller, &junction, now_ms);
            execute(service, &action);
            if (action.type == CONTROLLER_ACTION_EXECUTE_ROUTE)
            {
                start_route(service, action.route_command, now_ms);
            }
        }
    }

    if ((service->junction_phase == CONTROLLER_JUNCTION_IDLE) &&
        (controller->mission == MISSION_JUNCTION_APPROACH) &&
        out->probe_requested(out->context))
    {
        set_phase(service, CONTROLLER_JUNCTION_PROBE_SETTLE, now_ms);
    }
}

static void junction_tick(controller_service_t *service, system_time_ms_t now_ms)
{
    const controller_junction_outputs_t *out = &service->junction_out;
    motion_goal_result_t result = {0};
    goal_state_t state = GOAL_RUNNING;

    if ((service->junction_phase != CONTROLLER_JUNCTION_IDLE) &&
        !phase_still_valid(service))
    {
        junction_abort(service, now_ms);
    }

    switch (service->junction_phase)
    {
        case CONTROLLER_JUNCTION_IDLE:
            junction_poll(service, now_ms);
            break;

        case CONTROLLER_JUNCTION_PROBE_SETTLE:
            if (!time_reached(now_ms,
                    service->phase_since_ms + service->junction.settle_ms))
            {
                break;
            }
            if (!out->probe_begin(out->context))
            {
                set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);  /* withdrawn */
            }
            else if (submit_goal(service, MOTION_GOAL_PIVOT_LEFT, 0,
                         service->junction.probe_mdeg))
            {
                set_phase(service, CONTROLLER_JUNCTION_PROBE_OUT, now_ms);
            }
            else
            {
                (void)out->probe_end(out->context);
                junction_fail(service, STOP_REASON_ROUTE_REQUIRED, now_ms);
            }
            break;

        case CONTROLLER_JUNCTION_PROBE_OUT:
            state = poll_goal(service, &result);
            if (state == GOAL_DONE)
            {
                goal_step_done(service);
                if (submit_goal(service, MOTION_GOAL_PIVOT_RIGHT, 0,
                        service->junction.probe_mdeg))
                {
                    set_phase(service, CONTROLLER_JUNCTION_PROBE_BACK, now_ms);
                }
                else
                {
                    junction_fail(service, STOP_REASON_ROUTE_REQUIRED, now_ms);
                }
            }
            break;

        case CONTROLLER_JUNCTION_PROBE_BACK:
            state = poll_goal(service, &result);
            if (state == GOAL_DONE)
            {
                goal_step_done(service);
                (void)out->probe_end(out->context);
                set_phase(service, CONTROLLER_JUNCTION_PROBE_RESULT, now_ms);
            }
            break;

        case CONTROLLER_JUNCTION_PROBE_RESULT:
            junction_poll(service, now_ms);
            if (service->junction_phase != CONTROLLER_JUNCTION_PROBE_RESULT)
            {
                break;   /* the route started */
            }
            if (!phase_still_valid(service))
            {
                set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);  /* stopped */
            }
            else if (time_reached(now_ms,
                         service->phase_since_ms + service->junction.settle_ms))
            {
                junction_fail(service, STOP_REASON_ROUTE_REQUIRED, now_ms);
            }
            break;

        case CONTROLLER_JUNCTION_APPROACH:
            state = poll_goal(service, &result);
            if (state == GOAL_DONE)
            {
                start_turn(service, now_ms);
                if (service->junction_phase == CONTROLLER_JUNCTION_APPROACH)
                {
                    finish_route(service, &result, now_ms);   /* STRAIGHT */
                }
            }
            break;

        case CONTROLLER_JUNCTION_TURN:
            state = poll_goal(service, &result);
            if (state == GOAL_DONE)
            {
                finish_route(service, &result, now_ms);
            }
            break;

        default:
            set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);
            break;
    }

    if (state == GOAL_FAILED)
    {
        goal_failed(service, &result, now_ms);
    }
    refresh_status(service);
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
    if (service->junction_bound)
    {
        junction_tick(service, now_ms);
        if (service->junction_phase != CONTROLLER_JUNCTION_IDLE)
        {
            return;   /* a junction step owns the motion */
        }
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

void controller_service_motion_result(controller_service_t *service,
    const motion_goal_result_t *result)
{
    controller_action_t action;
    if (service == NULL || !service->initialized || result == NULL) { return; }
    action = vehicle_controller_on_motion_result(&service->controller, result);
    execute(service, &action);
    refresh_status(service);
}

/* Called while the controller task owns its guard. No separately queued CLEAR
 * can outlive this request and erase a newer STOP. The task re-drains urgent
 * ingress after this operation and before issuing motion on its next tick. */
bool controller_service_remote_start(controller_service_t *s,
    uint32_t requested, uint32_t current, bool pending)
{
    controller_status_t status;
    controller_event_t clear={CONTROLLER_EVENT_CLEAR_STOP,SYSTEM_MODE_IDLE,
        STOP_REASON_REMOTE,MISSION_IDLE,false};
    controller_event_t start={CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_AUTO,
        STOP_REASON_NONE,MISSION_IDLE,false};
    if(s==NULL || !s->initialized || requested!=current || current==UINT32_MAX || pending)
        return false;
    controller_service_status(s,&status);
    if((status.mode!=SYSTEM_MODE_STARTUP && status.mode!=SYSTEM_MODE_IDLE && status.mode!=SYSTEM_MODE_STOPPED) ||
       status.hardware_faults!=0U ||
       (status.stop_reasons & ~((uint32_t)STOP_REASON_REMOTE))!=0U) return false;
    if(status.stop_reasons!=0U && !controller_service_handle(s,&clear)) return false;
    return controller_service_handle(s,&start);
}
