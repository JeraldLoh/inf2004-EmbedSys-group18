#include "controller_service.h"
#include "../project_config/car_trial_tuning.h"

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

/* While a barcode is being captured, scale the steering request to this
 * percent. A snap correction mid-barcode changes speed and heading across the
 * bars and breaks the read. 100 = unchanged (main). Corrections still happen
 * for a big deviation. make BARCODE_STEER_PERCENT=40 */
#ifndef CONTROLLER_BARCODE_STEER_PERCENT
#define CONTROLLER_BARCODE_STEER_PERCENT 100
#endif

/* IMU heading assist (make HDG_ASSIST=1). While the line sensors are both
 * white the car is not being steered by the tape, so it drifts across the
 * line and weaves. The compass reference is captured when the run starts and
 * a small, capped counter-curvature holds the heading within about +/-2 deg.
 * The tape always wins: while a sensor sees tape the reference simply follows
 * the car, so curves, junctions and recovery are untouched. */
#ifndef CONTROLLER_HDG_ASSIST
#define CONTROLLER_HDG_ASSIST 0
#endif
#if CONTROLLER_HDG_ASSIST
#define HDG_DEADBAND_DDEG 20          /* no correction inside +/-2.0 deg */
#define HDG_LINE_STEER_MIN 25         /* |steer| above the idle hold = tape seen */
#define HDG_MAX_TURN_DPS 150          /* faster than this: compass untrusted */
#define HDG_LOST_DDEG 250             /* >25 deg off: reference is stale */
#define HDG_BIAS_BASE_U1_M 3600000    /* just above Buddy 2's 3.5/m deadband */
#define HDG_BIAS_PER_DEG_U1_M 500000  /* +0.5/m per degree beyond the deadband */
#define HDG_BIAS_MAX_U1_M 8000000     /* cap: 8/m */
#define HDG_REF_SLEW_MS 25            /* reference follows at 1 deci-deg / 25 ms */
bool s4_heading_read(int32_t *heading_ddeg, int32_t *turn_dps);
void s4_heading_report(int state, int32_t ref_ddeg, int32_t err_ddeg, int32_t bias_u1_m);

static bool hdg_ref_set;
static int32_t hdg_ref_ddeg;
static uint32_t hdg_last_ms;

static int32_t hdg_wrap(int32_t d)
{
    while (d > 1800) { d -= 3600; }
    while (d < -1800) { d += 3600; }
    return d;
}

static void heading_assist_idle(void)
{
    hdg_ref_set = false;
}

/* Extra curvature (1e-6 /m units, + = left) from the compass. */
static int32_t heading_assist_bias(int32_t steer, uint32_t now_ms)
{
    int32_t hdg;
    int32_t turn;
    int32_t err;
    int32_t mag;
    int32_t bias;
    int32_t abs_steer = steer < 0 ? -steer : steer;

    if (!s4_heading_read(&hdg, &turn))
    {
        hdg_ref_set = false;
        s4_heading_report(0, 0, 0, 0);          /* still calibrating */
        return 0;
    }
    if (!hdg_ref_set)
    {
        hdg_ref_set = true;
        hdg_ref_ddeg = hdg;
        hdg_last_ms = now_ms;
    }
    err = hdg_wrap(hdg - hdg_ref_ddeg);
    mag = err < 0 ? -err : err;
    if ((abs_steer >= HDG_LINE_STEER_MIN) || (mag > HDG_LOST_DDEG) ||
        (turn > HDG_MAX_TURN_DPS) || (turn < -HDG_MAX_TURN_DPS))
    {
        hdg_ref_ddeg = hdg;                     /* the tape (or a turn) leads */
        hdg_last_ms = now_ms;
        s4_heading_report(2, hdg_ref_ddeg, 0, 0);
        return 0;
    }
    if ((now_ms - hdg_last_ms) >= HDG_REF_SLEW_MS)
    {
        hdg_last_ms = now_ms;                   /* let gentle bends move the reference */
        hdg_ref_ddeg += err > 0 ? 1 : (err < 0 ? -1 : 0);
        err = hdg_wrap(hdg - hdg_ref_ddeg);
        mag = err < 0 ? -err : err;
    }
    if (mag <= HDG_DEADBAND_DDEG)
    {
        s4_heading_report(1, hdg_ref_ddeg, err, 0);
        return 0;
    }
    bias = HDG_BIAS_BASE_U1_M +
        ((mag - HDG_DEADBAND_DDEG) * (HDG_BIAS_PER_DEG_U1_M / 10));
    if (bias > HDG_BIAS_MAX_U1_M) { bias = HDG_BIAS_MAX_U1_M; }
    bias = err > 0 ? bias : -bias;              /* heading right of ref: steer left */
    s4_heading_report(1, hdg_ref_ddeg, err, bias);
    return bias;
}
#endif

/* Junction pause (trial policy): stand still this long at a junction. A barcode
 * route that is live then runs; with none, the car drives straight on.
 * make JUNCTION_PAUSE_MS=1500 */
#ifndef CONTROLLER_JUNCTION_PAUSE_MS
#define CONTROLLER_JUNCTION_PAUSE_MS 1000U
#endif

/* Turn tuning. The wheels pivot less than they are told (150-160 deg for a
 * 180 on the car), so the pivot goal is scaled up; the line search then finds
 * the line. The car also drives further into the junction before pivoting.
 * make JUNCTION_APPROACH_MM=100 JUNCTION_TURN_COMP_PERCENT=116 */
#ifndef CONTROLLER_JUNCTION_APPROACH_MM
#define CONTROLLER_JUNCTION_APPROACH_MM 50
#endif
#ifndef CONTROLLER_JUNCTION_TURN_COMP_PERCENT
#define CONTROLLER_JUNCTION_TURN_COMP_PERCENT 100
#endif

/* Line-follow cruise speed; CAR_RUN may lower it with -D for floor tuning. */
#ifndef CONTROLLER_LINE_CRUISE_MM_S
#define CONTROLLER_LINE_CRUISE_MM_S 150U
#endif
#ifndef CONTROLLER_LINE_CURVATURE_PER_STEER
#define CONTROLLER_LINE_CURVATURE_PER_STEER 178000
#endif
#ifndef CONTROLLER_LINE_LOST_MS
#define CONTROLLER_LINE_LOST_MS 0U   /* off: see controller_line_config_t */
#endif

void controller_line_default_config(controller_line_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    /* Retain the team's steering proposal: full steer approaches 2/track
     * (17.8 1/m for 112 mm). CAR_RUN's measured adapter explicitly converts
     * this legacy micro-1/m, positive-left value to the measured units/sign.
     * Cruise is a speed target, not an actuator effort or tuned PID gain. */
    config->cruise_mm_s = CONTROLLER_LINE_CRUISE_MM_S;
    config->slow_mm_s = CONTROLLER_LINE_CRUISE_MM_S < 100U ? CONTROLLER_LINE_CRUISE_MM_S : 100U;
    config->curvature_per_steer_u1_m = CONTROLLER_LINE_CURVATURE_PER_STEER;
    config->lost_ms = CONTROLLER_LINE_LOST_MS;
    config->search_step_mdeg = 15000;   /* sensors ~5 cm ahead sweep ~1.3 cm */
    config->search_max_mdeg = 60000;
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
    config->approach_mm = CONTROLLER_JUNCTION_APPROACH_MM;
    config->turn_mdeg = 90000 * CONTROLLER_JUNCTION_TURN_COMP_PERCENT / 100;
    config->uturn_mdeg = 180000 * CONTROLLER_JUNCTION_TURN_COMP_PERCENT / 100;
    config->probe_mdeg = 10000;
    config->goal_speed_mm_s = 100U;
    config->goal_timeout_ms = 0U; /* B2 derives finite distance/speed + ramp/settle bound. */
    config->settle_ms = 250U;
    config->holdoff_ms = 500U;
    config->pause_ms = CONTROLLER_JUNCTION_PAUSE_MS;
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

    if (service->junction_phase == CONTROLLER_JUNCTION_PAUSE)
    {
        /* Standing still on the line mission: valid until STOP, a mode change
         * or an obstacle takes the car away. */
        return (controller->mode == SYSTEM_MODE_AUTO) &&
            following_line(controller->mission) &&
            vehicle_controller_motion_allowed(controller, MOTION_SOURCE_LINE);
    }
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
#if defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL
            /* Trial policy: stand still at the junction, then follow the live
             * barcode route if there is one, otherwise drive straight on. The
             * pause uses the normal zero-output stop (never clears a STOP);
             * junction_tick() makes the choice when it has lasted pause_ms. */
            if (junction.candidate && following_line(controller->mission) &&
                safety_motion_allowed(&controller->safety) &&
                vehicle_controller_hardware_faults(controller) == 0U)
            {
                controller_action_t pause_stop = {0};
                pause_stop.type = CONTROLLER_ACTION_STOP_MOTION;
                service->pause_junction = junction;
                execute(service, &pause_stop);
                set_phase(service, CONTROLLER_JUNCTION_PAUSE, now_ms);
                return;
            }
#endif
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
        /* A deliberate manoeuvre pause must retire the live line command.
         * Merely withholding refresh leaves its 150ms lease running during
         * the 250ms probe settle, which the measured executor treats as a
         * timeout fault. Use the normal zero-output pause; never clear STOP. */
        controller_action_t pause = {0};
        pause.type = CONTROLLER_ACTION_STOP_MOTION;
        set_phase(service, CONTROLLER_JUNCTION_PROBE_SETTLE, now_ms);
        execute(service, &pause);
    }
}

static void search_start(controller_service_t *service,
    const controller_line_input_t *line, system_time_ms_t now_ms);

/* After a pivot the car sits wherever the wheels say it turned to, usually a
 * little short of the line. If neither sensor sees black, hand over to the
 * line search, which keeps turning the same way first. */
static void find_line_after_turn(
    controller_service_t *service,
    route_command_t command,
    system_time_ms_t now_ms)
{
    controller_line_input_t line = {0};

    if ((command != ROUTE_COMMAND_LEFT) && (command != ROUTE_COMMAND_RIGHT) &&
        (command != ROUTE_COMMAND_UTURN))
    {
        return;   /* STRAIGHT: still on the line */
    }
    if ((service->outputs.read_line == NULL) ||
        (service->controller.mode != SYSTEM_MODE_AUTO))
    {
        return;
    }
    if (service->outputs.read_line(service->outputs.context, &line) && line.on_line)
    {
        return;   /* already on the line */
    }
    line.steer = (command == ROUTE_COMMAND_RIGHT) ? 1 : -1;   /* same way as the pivot */
    search_start(service, &line, now_ms);
}

/* The pause is over: run the live barcode route, or carry straight on. */
static void pause_finish(controller_service_t *service, system_time_ms_t now_ms)
{
    vehicle_controller_t *controller = &service->controller;
    controller_action_t action;
    bool live = (controller->route_lifecycle == ROUTE_LIFECYCLE_PENDING) &&
        (now_ms <= controller->route.expires_ms);

    if (live)
    {
        action = vehicle_controller_on_junction(controller,
            &service->pause_junction, now_ms);
        execute(service, &action);
        if (action.type == CONTROLLER_ACTION_EXECUTE_ROUTE)
        {
            start_route(service, action.route_command, now_ms);
        }
        else
        {
            set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);
        }
        return;
    }
    if (controller->route_lifecycle == ROUTE_LIFECYCLE_PENDING)
    {
        controller->route_lifecycle = ROUTE_LIFECYCLE_EXPIRED;
    }
    /* No command: carry on straight. The pause retired the old LINE
     * authority, so resume under a fresh generation (as after a turn). */
    controller->authority_generation++;
    if (controller->authority_generation == 0U)
    {
        controller->authority_generation = 1U;
    }
    controller->mission = MISSION_TRACK_LINE;
    service->status.last_route = ROUTE_COMMAND_STRAIGHT;
    service->status.junctions_done++;
    service->holdoff = true;
    service->holdoff_until_ms = now_ms + service->junction.holdoff_ms;
    set_phase(service, CONTROLLER_JUNCTION_IDLE, now_ms);
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

        case CONTROLLER_JUNCTION_PAUSE:
        {
            route_instruction_t pause_route;

            /* A barcode decoded while we wait still counts. */
            if (out->take_route(out->context, &pause_route))
            {
                pause_route.expires_ms = now_ms +
                    (pause_route.expires_ms - pause_route.decoded_ms);
                pause_route.decoded_ms = now_ms;
                if (vehicle_controller_receive_route(&service->controller,
                        &pause_route, now_ms))
                {
                    service->status.routes_accepted++;
                    service->status.last_route = pause_route.command;
                }
                else
                {
                    service->status.routes_rejected++;
                }
            }
            if (time_reached(now_ms,
                    service->phase_since_ms + service->junction.pause_ms))
            {
                pause_finish(service, now_ms);
            }
            break;
        }

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
                route_command_t turned = service->junction_route;

                finish_route(service, &result, now_ms);
                find_line_after_turn(service, turned, now_ms);
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

/* ---- Line search --------------------------------------------------------
 * Car trial 2026-11-02: with the no-black stop off, a car that had lost the
 * line drove on along its last steering and left the course. The sensors
 * straddle the line, so both white is normal when centred; only a longer
 * time with no black at all (lost_ms) counts as lost. Then the car stops and
 * pivots, a step at a time, toward the side the line was last seen, then the
 * other way, and carries on following as soon as a sensor sees black. */

static void search_end(controller_service_t *service, system_time_ms_t now_ms)
{
    route_instruction_t route;
    junction_observation_t junction;

    service->searching = false;
    service->status.line_searching = false;
    service->line_seen_ms = now_ms;
    /* Pivoting over the line can look like a crossbar: drop those events. */
    (void)service->junction_out.take_route(service->junction_out.context, &route);
    (void)service->junction_out.take_junction(service->junction_out.context, &junction);
}

static void search_fail(controller_service_t *service, system_time_ms_t now_ms)
{
    controller_action_t action;

    search_end(service, now_ms);
    goal_step_done(service);
    service->status.line_search_fails++;
    action = vehicle_controller_latch_stop(&service->controller,
        STOP_REASON_RECOVERY_FAILED);
    execute(service, &action);
}

static void search_next(controller_service_t *service, system_time_ms_t now_ms)
{
    int32_t step = service->line.search_step_mdeg;
    int32_t max = service->line.search_max_mdeg;
    int32_t mdeg = step;
    int8_t dir = service->search_dir;

    if ((service->search_pass == 0) && (service->search_mdeg < max))
    {
        service->search_mdeg += step;
    }
    else if (service->search_pass == 0)
    {
        service->search_pass = 1;   /* swing back across to the other side */
        dir = (int8_t)-dir;
        mdeg = service->search_mdeg + step;
        service->search_mdeg = -step;
    }
    else if (service->search_mdeg > -max)
    {
        dir = (int8_t)-dir;
        service->search_mdeg -= step;
    }
    else
    {
        search_fail(service, now_ms);
        return;
    }
    service->search_seen = false;
    if (!submit_goal(service,
            (dir > 0) ? MOTION_GOAL_PIVOT_RIGHT : MOTION_GOAL_PIVOT_LEFT, 0, mdeg))
    {
        search_fail(service, now_ms);
    }
}

static void search_start(controller_service_t *service,
    const controller_line_input_t *line, system_time_ms_t now_ms)
{
    service->searching = true;
    service->status.line_searching = true;
    service->status.line_searches++;
    /* Buddy 3 keeps steering toward the side last seen: steer > 0 = right. */
    service->search_dir = (line->steer < 0) ? -1 : 1;
    service->search_pass = 0;
    service->search_mdeg = 0;
    search_next(service, now_ms);
}

static void search_tick(controller_service_t *service, system_time_ms_t now_ms)
{
    const vehicle_controller_t *controller = &service->controller;
    controller_line_input_t line = {0};
    motion_goal_result_t result = {0};

    if ((controller->mode != SYSTEM_MODE_AUTO) ||
        !following_line(controller->mission) ||
        !vehicle_controller_motion_allowed(controller, MOTION_SOURCE_JUNCTION))
    {
        search_end(service, now_ms);   /* STOP, obstacle or mode change */
        return;
    }
    if (service->outputs.read_line(service->outputs.context, &line) && line.on_line)
    {
        service->search_seen = true;
    }
    switch (poll_goal(service, &result))
    {
        case GOAL_RUNNING:
            break;
        case GOAL_DONE:
            goal_step_done(service);
            if (service->search_seen)
            {
                search_end(service, now_ms);   /* line following takes over */
            }
            else
            {
                search_next(service, now_ms);
            }
            break;
        default:
            search_end(service, now_ms);
            goal_failed(service, &result, now_ms);
            break;
    }
    refresh_status(service);
}

void controller_service_tick(
    controller_service_t *service,
    system_time_ms_t now_ms)
{
    const vehicle_controller_t *controller;
    controller_line_input_t line = {0};
    motion_continuous_request_t request;
    uint32_t speed;

    if ((service == NULL) || !service->initialized ||
        (service->outputs.drive == NULL))
    {
        return;
    }
    if (service->searching)
    {
#if CONTROLLER_HDG_ASSIST
        heading_assist_idle();
#endif
        search_tick(service, now_ms);
        return;   /* the line search owns the motion */
    }
    if (service->junction_bound)
    {
        junction_tick(service, now_ms);
        if (service->junction_phase != CONTROLLER_JUNCTION_IDLE)
        {
#if CONTROLLER_HDG_ASSIST
            heading_assist_idle();
#endif
            service->line_seen_ms = now_ms;
            return;   /* a junction step owns the motion */
        }
    }
    controller = &service->controller;
    if ((controller->mode != SYSTEM_MODE_AUTO) ||
        !following_line(controller->mission) ||
        !vehicle_controller_motion_allowed(controller, MOTION_SOURCE_LINE) ||
        !service->outputs.read_line(service->outputs.context, &line))
    {
#if CONTROLLER_HDG_ASSIST
        heading_assist_idle();
#endif
        service->line_seen_ms = now_ms;   /* the lost timer runs only while following */
        return;   /* nothing sent: Buddy 2 stops when the last one expires */
    }
    if (line.on_line)
    {
        service->line_seen_ms = now_ms;
    }
    else if ((service->line.lost_ms != 0U) && service->junction_bound &&
        (controller->mission != MISSION_JUNCTION_APPROACH) &&
        ((now_ms - service->line_seen_ms) >= service->line.lost_ms))
    {
        search_start(service, &line, now_ms);
        return;
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
    if (line.barcode_active && (CONTROLLER_BARCODE_STEER_PERCENT != 100))
    {
        /* Gentle steering while the barcode is read; see the define above. */
        request.curvature_u1_m = (int32_t)((int64_t)request.curvature_u1_m *
            CONTROLLER_BARCODE_STEER_PERCENT / 100);
    }
#if CONTROLLER_HDG_ASSIST
    request.curvature_u1_m += heading_assist_bias(line.steer, now_ms);
#endif
    request.analogue_steering = line.analogue_steering;
    request.created_ms = now_ms;
    request.expires_ms = now_ms + service->line.request_life_ms;

    service->status.last_steer = line.steer;
    service->status.last_analogue_steering = line.analogue_steering;
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
