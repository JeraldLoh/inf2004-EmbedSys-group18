#include "controller_task.h"
#include <tk/smp.h>

#define CONTROLLER_QUEUE_DEPTH 16U
#define CONTROLLER_TASK_PRIORITY 10
#define CONTROLLER_TASK_STACK 2048
#define CONTROLLER_STATUS_WAIT_MS 10

typedef enum { MAIL_EVENT = 0, MAIL_MOTION, MAIL_RESULT, MAIL_SAFETY, MAIL_WAKE, MAIL_REMOTE_START } mail_kind_t;
typedef struct
{
    mail_kind_t kind;
    uint32_t restart_stop_sequence;
    union
    {
        uint32_t remote_start_sequence;
        controller_event_t event;
        controller_motion_request_t motion;
        motion_goal_result_t result;
        struct { stop_reason_mask_t reasons; uint32_t evidence_id; } safety;
    } value;
} controller_mail_t;

static controller_service_t service;
static controller_motion_t motion;
static ID queue, guard, task;
static bool initialized, started;
#if TK_SUPPORT_SMP
static T_SPLOCK ingress_lock;
#endif
static stop_reason_mask_t urgent_reasons;
static uint32_t urgent_sequence;
static controller_motion_request_t latest_continuous;
static bool continuous_pending;

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
            switch (mail.kind)
            {
                case MAIL_EVENT:
                {
                    bool restart=mail.value.event.type==CONTROLLER_EVENT_CLEAR_STOP ||
                        (mail.value.event.type==CONTROLLER_EVENT_SET_MODE &&
                         mail.value.event.mode==SYSTEM_MODE_AUTO);
                    if(restart){
                        UINT saved=ingress_enter();
                        bool stale=mail.restart_stop_sequence!=urgent_sequence ||
                            urgent_reasons!=STOP_REASON_NONE || urgent_sequence==UINT32_MAX;
                        ingress_leave(saved);
                        if(stale) break;
                    }
                    (void)controller_service_handle(&service, &mail.value.event);
                    controller_motion_sync(&motion);
                    if(restart) drain_urgent();
                    break;
                }
                case MAIL_MOTION:
                    (void)controller_motion_handle(&motion, &mail.value.motion, now_ms());
                    break;
                case MAIL_RESULT:
                    controller_motion_result(&motion, &mail.value.result);
                    break;
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
                        mail.value.remote_start_sequence,sequence,pending);
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
            controller_motion_request_t latest;
            bool pending;
            UINT saved = ingress_enter();
            pending = continuous_pending;
            latest = latest_continuous;
            continuous_pending = false;
            ingress_leave(saved);
            if (pending) { (void)controller_motion_handle(&motion, &latest, now_ms()); }
        }
        if (now_ms()-last_tick >= 10U)
        {
            last_tick = now_ms();
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
bool controller_post_remote_start(void)
{
    controller_mail_t mail={0};
    UINT saved;
    if(!started) return false;
    saved=ingress_enter();
    if(urgent_reasons!=STOP_REASON_NONE || urgent_sequence==UINT32_MAX){
        ingress_leave(saved);return false;
    }
    mail.kind=MAIL_REMOTE_START;mail.value.remote_start_sequence=urgent_sequence;
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
        mail.kind = MAIL_WAKE;
        (void)post(&mail);
        return true;
    }
    /* Physical restart events also cannot erase a newer remote/physical STOP. */
    if(event->type==CONTROLLER_EVENT_CLEAR_STOP ||
       (event->type==CONTROLLER_EVENT_SET_MODE && event->mode==SYSTEM_MODE_AUTO)){
        UINT saved=ingress_enter();
        if(urgent_reasons!=STOP_REASON_NONE || urgent_sequence==UINT32_MAX){
            ingress_leave(saved);return false;
        }
        mail.restart_stop_sequence=urgent_sequence;
        ingress_leave(saved);
    }
    mail.kind = MAIL_EVENT; mail.value.event = *event; return post(&mail);
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
    if (controller_urgent_stop_pending()) { return false; }
    if (request->kind == CONTROLLER_MOTION_CONTINUOUS)
    {
        UINT saved = ingress_enter();
        latest_continuous = *request; continuous_pending = true;
        ingress_leave(saved);
        mail.kind = MAIL_WAKE; (void)post(&mail);
        return true; /* Latest slot accepted even if normal queue is full. */
    }
    mail.kind = MAIL_MOTION; mail.value.motion = *request; return post(&mail);
}
bool controller_post_motion_result(const motion_goal_result_t *result)
{
    controller_mail_t mail = {0};
    if (!motion.bound || result == NULL) { return false; }
    mail.kind = MAIL_RESULT; mail.value.result = *result; return post(&mail);
}
bool controller_post_motion_stop(stop_reason_mask_t reasons, uint32_t evidence_id)
{
    controller_mail_t mail = {0};
    if (!motion.bound || reasons == STOP_REASON_NONE) { return false; }
    mail.kind = MAIL_SAFETY;
    mail.value.safety.reasons = reasons; mail.value.safety.evidence_id = evidence_id;
    return post(&mail); /* Caller retains platform evidence until processed. */
}
bool controller_get_status(controller_status_t *status)
{
    if (!initialized || status == NULL || tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK)
    { return false; }
    controller_service_status(&service, status);
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
