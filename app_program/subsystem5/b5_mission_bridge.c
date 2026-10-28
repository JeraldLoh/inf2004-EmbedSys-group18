#include "b5_mission_bridge.h"

static controller_event_t event_of(controller_event_type_t type)
{
    controller_event_t e = {type, SYSTEM_MODE_IDLE, STOP_REASON_NONE,
        MISSION_IDLE, false};
    return e;
}

static void push(controller_event_t out[B5_BRIDGE_MAX_EVENTS], size_t *n,
    controller_event_t e)
{
    if (*n < B5_BRIDGE_MAX_EVENTS)
    {
        out[(*n)++] = e;
    }
}

static void push_progress(b5_bridge_t *bridge,
    controller_event_t out[B5_BRIDGE_MAX_EVENTS], size_t *n,
    mission_state_t mission)
{
    controller_event_t e = event_of(CONTROLLER_EVENT_OBSTACLE_PROGRESS);
    e.mission = mission;
    push(out, n, e);
    bridge->mission = mission;
}

/* Mission for an avoidance state that is part of an obstacle encounter. */
static mission_state_t mission_of(av_state_t state)
{
    switch (state)
    {
        case AV_STATE_DECIDE: return MISSION_PLAN_BYPASS;
        case AV_STATE_BYPASS: return MISSION_BYPASS;
        case AV_STATE_REACQUIRE: return MISSION_RECOVERY;
        default: return MISSION_SCAN; /* scans, rescan wait, reverse */
    }
}

void b5_bridge_init(b5_bridge_t *bridge)
{
    if (bridge != NULL)
    {
        bridge->active = false;
        bridge->failure_reported = false;
        bridge->mission = MISSION_IDLE;
    }
}

size_t b5_bridge_update(
    b5_bridge_t *bridge,
    av_state_t state,
    bool sensor_fault,
    controller_event_t out[B5_BRIDGE_MAX_EVENTS])
{
    size_t n = 0U;
    mission_state_t target;

    if ((bridge == NULL) || (out == NULL))
    {
        return 0U;
    }

    if (state == AV_STATE_FAILED)
    {
        if (!bridge->failure_reported)
        {
            controller_event_t e = event_of(CONTROLLER_EVENT_OBSTACLE_FAILED);
            e.reasons = sensor_fault ? STOP_REASON_SENSOR_CRITICAL
                                     : STOP_REASON_RECOVERY_FAILED;
            push(out, &n, e);
            bridge->failure_reported = true;
            bridge->active = false;
            bridge->mission = MISSION_SAFE_STOP;
        }
        return n;
    }

    if (state == AV_STATE_MONITOR)
    {
        if (bridge->active)
        {
            controller_event_t e = event_of(CONTROLLER_EVENT_OBSTACLE_CLEARED);
            e.rejoined_line = bridge->mission == MISSION_RECOVERY;
            push(out, &n, e);
        }
        /* After av_reset() from FAILED the controller still holds its latched
         * stop; clearing it is the operator's decision, not Buddy 5's. */
        b5_bridge_init(bridge);
        return n;
    }

    if (bridge->failure_reported)
    {
        return 0U; /* never restart a mission without passing MONITOR */
    }

    if (!bridge->active)
    {
        push(out, &n, event_of(CONTROLLER_EVENT_OBSTACLE_DETECTED));
        bridge->active = true;
        bridge->mission = MISSION_OBSTACLE_STOP;
    }

    target = mission_of(state);
    if (target == bridge->mission)
    {
        return n;
    }
    /* The controller only accepts listed steps; fill in the step Buddy 5 can
     * pass through within one tick. */
    if ((bridge->mission == MISSION_OBSTACLE_STOP) && (target != MISSION_SCAN))
    {
        push_progress(bridge, out, &n, MISSION_SCAN);
    }
    if ((bridge->mission == MISSION_SCAN) && (target == MISSION_BYPASS))
    {
        push_progress(bridge, out, &n, MISSION_PLAN_BYPASS);
    }
    if (target != bridge->mission)
    {
        push_progress(bridge, out, &n, target);
    }
    return n;
}
