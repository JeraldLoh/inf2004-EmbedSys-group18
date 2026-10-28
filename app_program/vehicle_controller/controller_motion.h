#ifndef GROUP18_CONTROLLER_MOTION_H
#define GROUP18_CONTROLLER_MOTION_H
#include "controller_service.h"
#include "../subsystem2/motion_hal.h"

/* Internal typed ingress, not a change to MQTT SET_SPEED or compact v1. */
typedef enum { CONTROLLER_MOTION_START = 0, CONTROLLER_MOTION_PROFILE,
    CONTROLLER_MOTION_CONTINUOUS, CONTROLLER_MOTION_GOAL } controller_motion_kind_t;
typedef enum { MOTION_PROFILE_STOP = 0, MOTION_PROFILE_CAUTION = 1,
    MOTION_PROFILE_CRUISE = 2 } motion_profile_t;
typedef struct
{
    controller_motion_kind_t kind;
    uint32_t session, id, generation, created_ms, expires_ms;
    motion_source_t source;
    union
    {
        motion_profile_t profile;
        struct { int32_t speed_mm_s, curvature_u1_m; uint32_t cap_mm_s; } continuous;
        struct { motion_goal_type_t type; int32_t mm, mdeg; uint32_t speed_mm_s, timeout_ms; } goal;
    } value;
} controller_motion_request_t;

typedef struct
{
    void *context;
    uint32_t session;
    motion_status_t (*authorize)(void *context, uint32_t generation);
    motion_status_t (*continuous)(void *context, const motion_continuous_request_t *request, uint32_t ttl_us);
    motion_status_t (*goal)(void *context, const motion_goal_request_t *request, uint32_t ttl_us);
    bool (*cancel)(void *context);
    void (*acknowledge_stop)(void *context, uint32_t evidence_id);
} controller_motion_outputs_t;

typedef struct
{
    uint32_t session, caution_mm_s, cruise_mm_s, authority_generation;
    motion_profile_t selected_profile;
    uint32_t accepted, rejected, terminal_sequence;
    motion_goal_result_t last_terminal;
    bool terminal_valid, enabled;
} controller_motion_status_t;
typedef struct
{
    controller_service_t *service;
    controller_motion_outputs_t outputs;
    controller_motion_status_t status;
    uint32_t last_id[6];
    uint32_t executor_generation;
    bool bound;
} controller_motion_t;

bool controller_motion_init(controller_motion_t *motion, controller_service_t *service,
    const controller_motion_outputs_t *outputs, uint32_t session,
    uint32_t caution_mm_s, uint32_t cruise_mm_s);
bool controller_motion_handle(controller_motion_t *motion,
    const controller_motion_request_t *request, uint32_t now_ms);
/* Called after ordinary controller events. STOP/mode change never resumes. */
void controller_motion_sync(controller_motion_t *motion);
void controller_motion_result(controller_motion_t *motion, const motion_goal_result_t *result);
void controller_motion_stop_evidence(controller_motion_t *motion,
    stop_reason_mask_t reasons, uint32_t evidence_id);
#endif
