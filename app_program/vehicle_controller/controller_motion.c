#include "controller_motion.h"
#include <stddef.h>
#include <string.h>

static bool moving_mode(system_mode_t mode)
{ return mode == SYSTEM_MODE_AUTO || mode == SYSTEM_MODE_MANUAL || mode == SYSTEM_MODE_CALIBRATION; }
static bool fresh(uint32_t now, uint32_t created, uint32_t expires)
{
    uint32_t lease = expires-created;
    return lease > 0U && lease <= 1000U && (now-created) < lease;
}

bool controller_motion_init(controller_motion_t *m, controller_service_t *service,
    const controller_motion_outputs_t *o, uint32_t session,
    uint32_t caution, uint32_t cruise)
{
    if (m == NULL || service == NULL || !service->initialized || o == NULL ||
        o->authorize == NULL || o->continuous == NULL || o->goal == NULL ||
        o->cancel == NULL || o->acknowledge_stop == NULL || session == 0U ||
        o->session != session || caution == 0U || caution > cruise || cruise > 500U) { return false; }
    (void)memset(m, 0, sizeof(*m));
    m->service = service; m->outputs = *o;
    m->status.session = session; m->status.caution_mm_s = caution;
    m->status.authority_generation = service->controller.authority_generation;
    m->status.cruise_mm_s = cruise; m->status.selected_profile = MOTION_PROFILE_CRUISE;
    m->bound = true; return true;
}

void controller_motion_sync(controller_motion_t *m)
{
    vehicle_controller_t *c;
    if (m == NULL || !m->bound) { return; }
    c = &m->service->controller;
    m->status.authority_generation = c->authority_generation;
    if (!moving_mode(c->mode) || c->safety.active_reasons != STOP_REASON_NONE ||
        vehicle_controller_hardware_faults(c) != 0U)
    {
        m->status.enabled = false;
        (void)m->outputs.cancel(m->outputs.context);
        m->executor_generation = 0U;
    }
    else if (m->executor_generation != 0U && m->executor_generation != c->authority_generation)
    {
        /* Onboard mission arbitration may issue a new scoped generation;
         * cancel old work. A subsequent explicit permitted request authorizes
         * the new generation; it does not clear a global STOP. */
        (void)m->outputs.cancel(m->outputs.context);
        m->executor_generation = 0U;
    }
}

static bool allowed_source(const vehicle_controller_t *c, motion_source_t source)
{
    if ((unsigned)source < MOTION_SOURCE_LINE || (unsigned)source > MOTION_SOURCE_CALIBRATION)
    { return false; }
    if (c->mode == SYSTEM_MODE_MANUAL && source != MOTION_SOURCE_MANUAL) { return false; }
    if (c->mode == SYSTEM_MODE_CALIBRATION && source != MOTION_SOURCE_CALIBRATION) { return false; }
    if (c->mode == SYSTEM_MODE_AUTO &&
        (source == MOTION_SOURCE_MANUAL || source == MOTION_SOURCE_CALIBRATION)) { return false; }
    return vehicle_controller_motion_allowed(c, source);
}

bool controller_motion_handle(controller_motion_t *m,
    const controller_motion_request_t *r, uint32_t now)
{
    vehicle_controller_t *c;
    uint32_t old, speed;
    motion_status_t result = MOTION_INVALID;
    if (m == NULL || !m->bound || r == NULL) { return false; }
    c = &m->service->controller;
    if (r->session != m->status.session || r->generation != c->authority_generation ||
        !fresh(now, r->created_ms, r->expires_ms) || r->id == 0U ||
        (unsigned)r->source > MOTION_SOURCE_CALIBRATION || r->source == MOTION_SOURCE_NONE)
    { ++m->status.rejected; return false; }
    old = m->last_id[r->source];
    if (old != 0U && ((r->id-old) == 0U || (r->id-old) >= 0x80000000UL))
    { ++m->status.rejected; return false; }
    m->last_id[r->source] = r->id; /* Rejected duplicates cannot renew a lease. */
    if (r->kind == CONTROLLER_MOTION_PROFILE)
    {
        if (r->value.profile == MOTION_PROFILE_STOP)
        {
            controller_event_t stop = {CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED,
                STOP_REASON_REMOTE, MISSION_IDLE, false};
            (void)controller_service_handle(m->service, &stop);
            controller_motion_sync(m);
            result = MOTION_OK; /* Preserve selected moving profile. */
        }
        else if (r->value.profile == MOTION_PROFILE_CAUTION || r->value.profile == MOTION_PROFILE_CRUISE)
        { m->status.selected_profile = r->value.profile; result = MOTION_OK; }
    }
    else if (moving_mode(c->mode) && c->safety.active_reasons == STOP_REASON_NONE &&
        vehicle_controller_hardware_faults(c) == 0U && allowed_source(c, r->source))
    {
        if (r->kind == CONTROLLER_MOTION_START && !c->active_goal)
        {
            (void)m->outputs.cancel(m->outputs.context);
            m->status.enabled = false;
            if (c->authority_generation != UINT32_MAX)
            {
                ++c->authority_generation; /* Fresh START, output still zero. */
                result = m->outputs.authorize(m->outputs.context, c->authority_generation);
                if (result == MOTION_OK)
                { m->executor_generation = c->authority_generation; m->status.enabled = true; }
            }
        }
        else if (m->status.enabled && !c->active_goal)
        {
            if (m->executor_generation != c->authority_generation)
            {
                result = m->outputs.authorize(m->outputs.context, c->authority_generation);
                if (result != MOTION_OK) { ++m->status.rejected; return false; }
                m->executor_generation = c->authority_generation;
            }
            speed = m->status.selected_profile == MOTION_PROFILE_CAUTION
                ? m->status.caution_mm_s : m->status.cruise_mm_s;
            speed = vehicle_controller_apply_speed_caps(c, speed);
            if (r->kind == CONTROLLER_MOTION_CONTINUOUS)
            {
                motion_continuous_request_t request = {0};
                request.command_id = r->id; request.authority_generation = c->authority_generation;
                request.source = r->source; request.speed_mm_s = r->value.continuous.speed_mm_s;
                if (request.speed_mm_s > (int32_t)speed) { request.speed_mm_s = (int32_t)speed; }
                request.curvature_u1_m = r->value.continuous.curvature_u1_m;
                request.speed_cap_mm_s = speed;
                if (r->value.continuous.cap_mm_s != 0U && r->value.continuous.cap_mm_s < speed)
                { request.speed_cap_mm_s = r->value.continuous.cap_mm_s; }
                request.created_ms = r->created_ms; request.expires_ms = r->expires_ms;
                result = m->outputs.continuous(m->outputs.context, &request, (r->expires_ms-now)*1000U);
            }
            else if (r->kind == CONTROLLER_MOTION_GOAL && speed != 0U)
            {
                motion_goal_request_t request = {0};
                request.goal_id = r->id; request.authority_generation = c->authority_generation;
                request.source = r->source; request.type = r->value.goal.type;
                request.target_mm = r->value.goal.mm; request.target_mdeg = r->value.goal.mdeg;
                request.max_speed_mm_s = r->value.goal.speed_mm_s == 0U ? speed : r->value.goal.speed_mm_s;
                if (request.max_speed_mm_s > speed) { request.max_speed_mm_s = speed; }
                request.timeout_ms = r->value.goal.timeout_ms;
                result = m->outputs.goal(m->outputs.context, &request, (r->expires_ms-now)*1000U);
                if (result == MOTION_OK) { c->active_goal = true; c->active_goal_id = request.goal_id; }
            }
        }
    }
    m->status.authority_generation = c->authority_generation;
    if (result == MOTION_OK) { ++m->status.accepted; return true; }
    ++m->status.rejected; return false;
}

void controller_motion_result(controller_motion_t *m, const motion_goal_result_t *r)
{
    if (m == NULL || !m->bound || r == NULL ||
        (r->code < MOTION_RESULT_COMPLETED && r->code != MOTION_RESULT_REJECTED) ||
        r->code > MOTION_RESULT_FAULT) { return; }
    m->status.last_terminal = *r; m->status.terminal_valid = true;
    if (m->status.terminal_sequence != UINT32_MAX) { ++m->status.terminal_sequence; }
    controller_service_motion_result(m->service, r);
    controller_motion_sync(m);
}

void controller_motion_stop_evidence(controller_motion_t *m,
    stop_reason_mask_t reasons, uint32_t evidence_id)
{
    controller_event_t stop = {CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED,
        reasons, MISSION_IDLE, false};
    if (m == NULL || !m->bound) { return; }
    (void)controller_service_handle(m->service, &stop);
    controller_motion_sync(m);
    if (evidence_id != 0U) { m->outputs.acknowledge_stop(m->outputs.context, evidence_id); }
}
