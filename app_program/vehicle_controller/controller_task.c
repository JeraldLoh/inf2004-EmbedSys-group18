#include "controller_task.h"
#include <tk/smp.h>

#define CONTROLLER_QUEUE_DEPTH 16U
#define CONTROLLER_TASK_PRIORITY 10
#define CONTROLLER_TASK_STACK 2048
#define CONTROLLER_STATUS_WAIT_MS 10

typedef enum { MAIL_EVENT = 0, MAIL_MOTION, MAIL_RESULT, MAIL_SAFETY, MAIL_WAKE, MAIL_REMOTE_START } mail_kind_t;
/* Compact, task-local format. Low three bits are the tag, and the remaining
 * bits are tag-specific. The kernel copies the whole record before publication. */
typedef struct
{
    uint32_t header;
    union
    {
        struct { uint32_t restart_ticket; stop_reason_mask_t reasons; } event;
        struct {
            uint32_t session, id, generation, created_ms, expires_ms;
            int32_t mm, mdeg;
            uint32_t speed_mm_s, timeout_ms;
        } motion;
        struct { uint32_t id, generation; int32_t mm, mdeg; } result;
        struct { stop_reason_mask_t reasons; uint32_t evidence_id; } safety;
    } value;
} controller_mail_t;
_Static_assert(sizeof(controller_mail_t) == 40U, "mailbox record budget");
_Static_assert(MAIL_REMOTE_START <= 7 && CONTROLLER_EVENT_COUNT <= 8 &&
    SYSTEM_MODE_FAULT <= 7 && MISSION_SAFE_STOP <= 15 &&
    CONTROLLER_MOTION_GOAL <= 3 && MOTION_SOURCE_CALIBRATION <= 7 &&
    MOTION_GOAL_RETRACE <= 7 && MOTION_PROFILE_CRUISE <= 7 &&
    MOTION_RESULT_FAULT <= 7, "mailbox header field budget");

static uint32_t mail_field(uint32_t header, unsigned shift, uint32_t mask)
{
    return (header >> shift) & mask;
}

static void unpack_motion(const controller_mail_t *mail, controller_motion_request_t *request)
{
    *request = (controller_motion_request_t){0};
    request->kind = (controller_motion_kind_t)mail_field(mail->header, 3U, 3U);
    request->source = (motion_source_t)mail_field(mail->header, 5U, 7U);
    request->session = mail->value.motion.session;
    request->id = mail->value.motion.id;
    request->generation = mail->value.motion.generation;
    request->created_ms = mail->value.motion.created_ms;
    request->expires_ms = mail->value.motion.expires_ms;
    if (request->kind == CONTROLLER_MOTION_GOAL)
    {
        request->value.goal.type = (motion_goal_type_t)mail_field(mail->header, 8U, 7U);
        request->value.goal.mm = mail->value.motion.mm;
        request->value.goal.mdeg = mail->value.motion.mdeg;
        request->value.goal.speed_mm_s = mail->value.motion.speed_mm_s;
        request->value.goal.timeout_ms = mail->value.motion.timeout_ms;
    }
    else if (request->kind == CONTROLLER_MOTION_PROFILE)
    {
        request->value.profile = (motion_profile_t)mail_field(mail->header, 8U, 7U);
    }
}

static controller_service_t service;
static controller_motion_t motion;
static ID queue, guard, task;
static bool initialized, started;
#if TK_SUPPORT_SMP
static T_SPLOCK ingress_lock;
#endif
static stop_reason_mask_t urgent_reasons;
static uint32_t urgent_sequence;
static uint32_t obstacle_unavailable; /* protected by ingress_lock */
static controller_motion_request_t latest_continuous;
static bool continuous_pending;
static bool (*terrain_reader)(terrain_report_t *report); /* set before start */
static bool (*obstacle_goal_fn)(const motion_goal_request_t *goal);
static bool (*obstacle_continuous_fn)(
    const motion_continuous_request_t *request);

static UINT ingress_enter(void)
{
    UINT saved;
#if TK_SUPPORT_SMP
    (void)ISpinLock(&ingress_lock, &saved);
#else
    saved = disint();
#endif
    return saved;
}
static void ingress_leave(UINT saved)
{
#if TK_SUPPORT_SMP
    (void)ISpinUnlock(&ingress_lock, saved);
#else
    set_primask(saved);
#endif
}
bool controller_urgent_stop_pending(void)
{
    UINT saved = ingress_enter();
    bool pending = urgent_reasons != STOP_REASON_NONE;
    ingress_leave(saved); return pending;
}

static uint32_t now_ms(void)
{
    SYSTIM time;
    (void)tk_get_otm(&time);
    return (uint32_t)time.lo; /* Full kernel millisecond domain, not raw_us/1000. */
}

static void drain_urgent(void)
{
    stop_reason_mask_t reasons;
    uint32_t sequence;
    UINT saved = ingress_enter();
    reasons = urgent_reasons; sequence = urgent_sequence;
    ingress_leave(saved);
    if (reasons != STOP_REASON_NONE)
    {
        controller_event_t event = {CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED,
            reasons, MISSION_IDLE, false};
        (void)controller_service_handle(&service, &event);
        controller_motion_sync(&motion);
        saved = ingress_enter();
        if (urgent_sequence == sequence && sequence != UINT32_MAX)
        { urgent_reasons = STOP_REASON_NONE; }
        ingress_leave(saved);
    }
}

static void controller_task_main(INT code, void *exinf)
{
    controller_mail_t mail;
    controller_motion_request_t request;
    uint32_t last_tick = now_ms();
    (void)code; (void)exinf;
    for (;;)
    {
        uint32_t waited = now_ms()-last_tick;
        INT wait = waited >= 10U ? 0 : (INT)(10U-waited);
        INT received = tk_rcv_mbf(queue, &mail, wait);
        if (tk_loc_mtx(guard, TMO_FEVR) != E_OK) { continue; }
        drain_urgent();
        if (received == (INT)sizeof(mail))
        {
            switch (mail_field(mail.header, 0U, 7U))
            {
                case MAIL_EVENT:
                {
                    controller_event_t event = {
                        (controller_event_type_t)mail_field(mail.header, 3U, 7U),
                        (system_mode_t)mail_field(mail.header, 6U, 7U),
                        mail.value.event.reasons,
                        (mission_state_t)mail_field(mail.header, 9U, 15U),
                        mail_field(mail.header, 13U, 1U) != 0U
                    };
                    bool restart=event.type==CONTROLLER_EVENT_CLEAR_STOP ||
                        (event.type==CONTROLLER_EVENT_SET_MODE && event.mode==SYSTEM_MODE_AUTO);
                    if(restart){
                        UINT saved=ingress_enter();
                        bool stale=mail.value.event.restart_ticket!=urgent_sequence ||
                            urgent_reasons!=STOP_REASON_NONE || urgent_sequence==UINT32_MAX;
                        ingress_leave(saved);
                        if(stale) break;
                    }
                    (void)controller_service_handle(&service, &event);
                    controller_motion_sync(&motion);
                    if(restart) drain_urgent();
                    break;
                }
                case MAIL_MOTION:
                    unpack_motion(&mail, &request);
                    (void)controller_motion_handle(&motion, &request, now_ms());
                    break;
                case MAIL_RESULT:
                {
                    motion_goal_result_t result = {
                        mail.value.result.id, mail.value.result.generation,
                        (motion_result_code_t)mail_field(mail.header, 3U, 7U),
                        mail.value.result.mm, mail.value.result.mdeg,
                        mail_field(mail.header, 6U, 1U) != 0U
                    };
                    controller_motion_result(&motion, &result);
                    break;
                }
                case MAIL_SAFETY:
                    controller_motion_stop_evidence(&motion, mail.value.safety.reasons,
                        mail.value.safety.evidence_id);
                    break;
                case MAIL_REMOTE_START:
                {
                    uint32_t sequence;
                    bool pending;
                    UINT saved=ingress_enter();
                    sequence=urgent_sequence;pending=urgent_reasons!=STOP_REASON_NONE;
                    ingress_leave(saved);
                    (void)controller_service_remote_start(&service,
                        mail.value.event.restart_ticket,sequence,pending,
                        mail.value.event.reasons);
                    controller_motion_sync(&motion);
                    /* A STOP racing with the conditional transition is applied
                     * before this task can issue any new line request. */
                    drain_urgent();
                    break;
                }
                case MAIL_WAKE:
                default: break;
            }
        }
        {
            bool pending;
            UINT saved = ingress_enter();
            pending = continuous_pending;
            if (pending) { request = latest_continuous; }
            continuous_pending = false;
            ingress_leave(saved);
            if (pending) { (void)controller_motion_handle(&motion, &request, now_ms()); }
        }
        if (now_ms()-last_tick >= 10U)
        {
            last_tick = now_ms();
            if (terrain_reader != NULL)
            {
                terrain_report_t terrain;
                if (terrain_reader(&terrain))
                {
                    vehicle_controller_update_terrain(&service.controller, &terrain);
                }
            }
            if (!motion.bound) { controller_service_tick(&service, last_tick); }
        }
        (void)tk_unl_mtx(guard);
    }
}

static void release(void)
{
    if (task > 0) { (void)tk_del_tsk(task); task = 0; }
    if (queue > 0) { (void)tk_del_mbf(queue); queue = 0; }
    if (guard > 0) { (void)tk_del_mtx(guard); guard = 0; }
}
ER controller_task_init(const controller_outputs_t *outputs)
{
    T_CMBF mbf = {0};
    T_CMTX mtx = {0};
    T_CTSK ctsk = {0};
    ID id;
    if (initialized) { return E_OBJ; }
    if (!controller_service_init(&service, outputs)) { return E_PAR; }
#if TK_SUPPORT_SMP
    (void)InitSpinLock(&ingress_lock);
#endif
    mtx.mtxatr = TA_INHERIT;
    id = tk_cre_mtx(&mtx);
    if (id < E_OK) { return id; }
    guard = id;
    mbf.mbfatr = TA_TFIFO;
    mbf.maxmsz = (INT)sizeof(controller_mail_t);
    mbf.bufsz = (SZ)(CONTROLLER_QUEUE_DEPTH*(sizeof(controller_mail_t)+sizeof(UW)));
    id = tk_cre_mbf(&mbf);
    if (id < E_OK) { release(); return id; }
    queue = id;
    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task = (FP)controller_task_main;
    ctsk.itskpri = CONTROLLER_TASK_PRIORITY;
    ctsk.stksz = CONTROLLER_TASK_STACK;
    id = tk_cre_tsk(&ctsk);
    if (id < E_OK) { release(); return id; }
    task = id; initialized = true;
    return E_OK;
}
ER controller_task_bind_junction(const controller_junction_outputs_t *outputs)
{
    if (!initialized || started || motion.bound) { return E_OBJ; }
    return controller_service_bind_junction(&service, outputs) ? E_OK : E_PAR;
}
ER controller_task_bind_motion(const controller_motion_outputs_t *outputs,
    uint32_t session, uint32_t caution, uint32_t cruise)
{
    if (!initialized || started || motion.bound || service.outputs.drive != NULL) { return E_OBJ; }
    return controller_motion_init(&motion, &service, outputs, session, caution, cruise)
        ? E_OK : E_PAR;
}
ER controller_task_bind_route_map(uint32_t (*read_revision)(void))
{
    if (!initialized || started) return E_OBJ;
    if (read_revision == NULL) return E_PAR;
    service.read_route_map_revision = read_revision;
    return E_OK;
}

ER controller_task_bind_terrain(bool (*read_terrain)(terrain_report_t *report))
{
    if (!initialized || started) { return E_OBJ; }
    if (read_terrain == NULL) { return E_PAR; }
    terrain_reader = read_terrain;
    return E_OK;
}
ER controller_task_bind_obstacle_motion(
    bool (*goal)(const motion_goal_request_t *goal),
    bool (*continuous)(const motion_continuous_request_t *request))
{
    if (!initialized || started) { return E_OBJ; }
    if ((goal == NULL) || (continuous == NULL)) { return E_PAR; }
    obstacle_goal_fn = goal;
    obstacle_continuous_fn = continuous;
    return E_OK;
}
ER controller_task_start(void)
{
    ER result;
    if (!initialized || started) { return E_OBJ; }
    result = tk_sta_tsk(task, 0);
    if (result == E_OK) { started = true; }
    return result;
}
static bool post(controller_mail_t *mail)
{
    return started && tk_snd_mbf(queue, mail, (INT)sizeof(*mail), TMO_POL) == E_OK;
}
uint32_t controller_remote_start_ticket(void)
{
    UINT saved=ingress_enter();
    uint32_t ticket=urgent_reasons==STOP_REASON_NONE ? urgent_sequence : UINT32_MAX;
    ingress_leave(saved);
    return ticket;
}

bool controller_post_remote_start(uint32_t ticket, stop_reason_mask_t expected_stops)
{
    controller_mail_t mail={0};
    UINT saved;
    if(!started) return false;
    saved=ingress_enter();
    if(urgent_reasons!=STOP_REASON_NONE || urgent_sequence!=ticket || ticket==UINT32_MAX){
        ingress_leave(saved);return false;
    }
    mail.header=MAIL_REMOTE_START;
    mail.value.event.restart_ticket=ticket;
    mail.value.event.reasons=expected_stops;
    ingress_leave(saved);
    return post(&mail);
}
bool controller_post(const controller_event_t *event)
{
    controller_mail_t mail = {0};
    if (!started || event == NULL) { return false; }
    if (event->type == CONTROLLER_EVENT_LATCH_STOP && event->reasons != STOP_REASON_NONE)
    {
        UINT saved;
        /* Immediate disable + separate sticky lane: never depend on FIFO room. */
        saved = ingress_enter();
        urgent_reasons |= event->reasons;
        if (urgent_sequence != UINT32_MAX) { ++urgent_sequence; }
        continuous_pending = false;
        ingress_leave(saved);
        (void)service.outputs.stop_motion(service.outputs.context);
        mail.header = MAIL_WAKE;
        (void)post(&mail);
        return true;
    }
    if ((unsigned)event->type >= CONTROLLER_EVENT_COUNT ||
        (unsigned)event->mode > SYSTEM_MODE_FAULT ||
        (unsigned)event->mission > MISSION_SAFE_STOP) { return false; }
    /* Physical restart events also cannot erase a newer remote/physical STOP. */
    if(event->type==CONTROLLER_EVENT_CLEAR_STOP ||
       (event->type==CONTROLLER_EVENT_SET_MODE && event->mode==SYSTEM_MODE_AUTO)){
        UINT saved=ingress_enter();
        if(urgent_reasons!=STOP_REASON_NONE || urgent_sequence==UINT32_MAX){
            ingress_leave(saved);return false;
        }
        mail.value.event.restart_ticket=urgent_sequence;
        ingress_leave(saved);
    }
    mail.header = MAIL_EVENT | ((uint32_t)event->type << 3U) |
        ((uint32_t)event->mode << 6U) | ((uint32_t)event->mission << 9U) |
        ((uint32_t)event->rejoined_line << 13U);
    mail.value.event.reasons = event->reasons;
    return post(&mail);
}
bool controller_post_motion(const controller_motion_request_t *request)
{
    controller_mail_t mail = {0};
    if (!started || !motion.bound || request == NULL) { return false; }
    if (request->kind == CONTROLLER_MOTION_PROFILE && request->value.profile == MOTION_PROFILE_STOP)
    {
        controller_event_t stop = {CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED,
            STOP_REASON_REMOTE, MISSION_IDLE, false};
        return controller_post(&stop);
    }
    if ((unsigned)request->kind > CONTROLLER_MOTION_GOAL ||
        (unsigned)request->source > MOTION_SOURCE_CALIBRATION ||
        controller_urgent_stop_pending()) { return false; }
    if (request->kind == CONTROLLER_MOTION_CONTINUOUS)
    {
        UINT saved = ingress_enter();
        latest_continuous = *request; continuous_pending = true;
        ingress_leave(saved);
        mail.header = MAIL_WAKE; (void)post(&mail);
        return true; /* Latest slot accepted even if normal queue is full. */
    }
    mail.header = MAIL_MOTION | ((uint32_t)request->kind << 3U) |
        ((uint32_t)request->source << 5U);
    if (request->kind == CONTROLLER_MOTION_GOAL)
    {
        if ((unsigned)request->value.goal.type > MOTION_GOAL_RETRACE) { return false; }
        mail.header |= (uint32_t)request->value.goal.type << 8U;
        mail.value.motion.mm = request->value.goal.mm;
        mail.value.motion.mdeg = request->value.goal.mdeg;
        mail.value.motion.speed_mm_s = request->value.goal.speed_mm_s;
        mail.value.motion.timeout_ms = request->value.goal.timeout_ms;
    }
    else if (request->kind == CONTROLLER_MOTION_PROFILE)
    {
        if ((unsigned)request->value.profile > MOTION_PROFILE_CRUISE) { return false; }
        mail.header |= (uint32_t)request->value.profile << 8U;
    }
    mail.value.motion.session = request->session;
    mail.value.motion.id = request->id;
    mail.value.motion.generation = request->generation;
    mail.value.motion.created_ms = request->created_ms;
    mail.value.motion.expires_ms = request->expires_ms;
    return post(&mail);
}
bool controller_post_motion_result(const motion_goal_result_t *result)
{
    controller_mail_t mail = {0};
    if (!motion.bound || result == NULL || (unsigned)result->code > MOTION_RESULT_FAULT)
    { return false; }
    mail.header = MAIL_RESULT | ((uint32_t)result->code << 3U) |
        ((uint32_t)result->odometry_valid << 6U);
    mail.value.result.id = result->goal_id;
    mail.value.result.generation = result->authority_generation;
    mail.value.result.mm = result->travelled_mm;
    mail.value.result.mdeg = result->turned_mdeg;
    return post(&mail);
}
bool controller_post_motion_stop(stop_reason_mask_t reasons, uint32_t evidence_id)
{
    controller_mail_t mail = {0};
    if (!motion.bound || reasons == STOP_REASON_NONE) { return false; }
    mail.header = MAIL_SAFETY;
    mail.value.safety.reasons = reasons; mail.value.safety.evidence_id = evidence_id;
    return post(&mail); /* Caller retains platform evidence until processed. */
}
bool controller_get_status(controller_status_t *status)
{
    if (!initialized || status == NULL || tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK)
    { return false; }
    controller_service_status(&service, status);
    {
        UINT saved = ingress_enter();
        uint32_t missed = obstacle_unavailable;
        ingress_leave(saved);
        status->obstacle_moves_rejected = missed > UINT32_MAX - status->obstacle_moves_rejected ?
            UINT32_MAX : status->obstacle_moves_rejected + missed;
    }
    (void)tk_unl_mtx(guard); return true;
}
bool controller_get_motion_status(controller_motion_status_t *status)
{
    if (!motion.bound || status == NULL || tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK)
    { return false; }
    *status = motion.status;
    status->authority_generation = service.controller.authority_generation;
    (void)tk_unl_mtx(guard); return true;
}

/* Caller holds the guard. Lock order is controller guard, then the Buddy 2
 * mutex, the same as the line tick. */
static bool obstacle_approved(
    motion_source_t source,
    authority_generation_t generation)
{
    authority_generation_t current = 0U;

    return (source == MOTION_SOURCE_OBSTACLE) &&
        controller_service_authority(&service, MOTION_SOURCE_OBSTACLE,
            &current) &&
        (generation == current);
}

static bool obstacle_unavailable_reject(void)
{
    if (initialized)
    {
        UINT saved = ingress_enter();
        if (obstacle_unavailable != UINT32_MAX) { obstacle_unavailable++; }
        ingress_leave(saved);
    }
    return false;
}

bool controller_obstacle_goal(const motion_goal_request_t *goal)
{
    bool ok;

    if (!initialized || (goal == NULL) || (obstacle_goal_fn == NULL) ||
        (tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK))
    {
        return obstacle_unavailable_reject();
    }
    ok = obstacle_approved(goal->source, goal->authority_generation) &&
        obstacle_goal_fn(goal);
    if (ok)
    {
        service.status.obstacle_moves_accepted++;
    }
    else
    {
        service.status.obstacle_moves_rejected++;
    }
    (void)tk_unl_mtx(guard);
    return ok;
}

bool controller_obstacle_continuous(const motion_continuous_request_t *request)
{
    bool ok;

    if (!initialized || (request == NULL) || (obstacle_continuous_fn == NULL) ||
        (tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK))
    {
        return obstacle_unavailable_reject();
    }
    ok = obstacle_approved(request->source, request->authority_generation) &&
        obstacle_continuous_fn(request);
    if (ok)
    {
        service.status.obstacle_moves_accepted++;
    }
    else
    {
        service.status.obstacle_moves_rejected++;
    }
    (void)tk_unl_mtx(guard);
    return ok;
}

bool controller_authority(motion_source_t source, authority_generation_t *generation)
{
    bool ok;

    if (!initialized || (tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK))
    {
        return false;
    }
    ok = controller_service_authority(&service, source, generation);
    (void)tk_unl_mtx(guard);
    return ok;
}
