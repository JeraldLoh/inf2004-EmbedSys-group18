#include "motion_task.h"
#include "subsystem2.h"
#include "b2_platform_rp2040.h"
#include "../vehicle_controller/controller_task.h"

static measured_motion_executor_t executor;
static measured_motion_executor_io_t backend;
static ID mutex, task;
static _Alignas(8) uint32_t motion_stack[1024];
static bool initialized, started;
static uint32_t session;

static bool stopped(void *context)
{
    (void)context;
    return controller_urgent_stop_pending() || backend.stop_pending(backend.context);
}
static bool watchdog(void *context, uint32_t duration)
{ (void)context; return !stopped(NULL) && backend.arm_watchdog(backend.context, duration); }
static void cancel_watchdog(void *context)
{ (void)context; backend.cancel_watchdog(backend.context); }

static bool lock(void) { return initialized && tk_loc_mtx(mutex, TMO_POL) == E_OK; }
static void unlock(void) { (void)tk_unl_mtx(mutex); }

static motion_status_t authorize(void *context, uint32_t generation)
{
    motion_status_t result;
    (void)context;
    if (!lock()) { return MOTION_STATE; }
    result = measured_motion_executor_authorize(&executor, generation);
    unlock(); return result;
}
static motion_status_t continuous(void *context, const motion_continuous_request_t *request, uint32_t ttl)
{
    motion_encoder_snapshot_t sample;
    motion_status_t result;
    (void)context;
    if (!lock()) { return MOTION_STATE; }
    result = drv_encoder_capture(executor.encoder, executor.motor, &sample);
    if (result == MOTION_OK)
    { result = measured_motion_executor_continuous(&executor, request, sample.acquired_us, ttl); }
    unlock(); return result;
}
static motion_status_t goal(void *context, const motion_goal_request_t *request, uint32_t ttl)
{
    motion_encoder_snapshot_t sample;
    motion_status_t result;
    (void)context;
    if (!lock()) { return MOTION_STATE; }
    result = drv_encoder_capture(executor.encoder, executor.motor, &sample);
    if (result == MOTION_OK)
    { result = measured_motion_executor_goal(&executor, request, sample.acquired_us, ttl); }
    unlock(); return result;
}
static bool cancel(void *context)
{
    bool ok;
    (void)context;
    /* Always disable immediately even if another task owns executor state. */
    ok = drv_motor_stop(executor.motor) == MOTION_OK;
    if (lock()) { measured_motion_executor_cancel(&executor, MOTION_RESULT_CANCELLED); unlock(); }
    return ok;
}
static void ack_stop(void *context, uint32_t id)
{ (void)context; (void)b2_platform_ack_events(id); }

static void motion_main(INT code, void *exinf)
{
    uint32_t queued_evidence = 0U;
    uint32_t reported_generation = 0U;
    (void)code; (void)exinf;
    for (;;)
    {
        b2_platform_events_t events = {0};
        motion_goal_result_t result;
        bool terminal = false;
        stop_reason_mask_t operational_stop = STOP_REASON_NONE;
        b2_platform_peek_events(&events);
        if (lock())
        {
            if (events.reasons != STOP_REASON_NONE || events.cutoff_fired)
            {
                measured_motion_executor_cancel(&executor, events.cutoff_fired
                    ? MOTION_RESULT_TIMEOUT : MOTION_RESULT_CANCELLED);
            }
            (void)measured_motion_executor_step(&executor);
            terminal = measured_motion_executor_result(&executor, &result);
            if (!executor.status.authorized && executor.status.generation != reported_generation &&
                executor.status.generation != 0U && !terminal)
            {
                if (executor.status.last_stop == MOTION_RESULT_TIMEOUT)
                {
                    operational_stop = executor.continuous.source == MOTION_SOURCE_MANUAL
                        ? STOP_REASON_MANUAL_EXPIRED : STOP_REASON_CONTROLLER_FAULT;
                }
                else if (executor.status.last_stop == MOTION_RESULT_FAULT ||
                    executor.status.last_stop == MOTION_RESULT_ABORTED)
                { operational_stop = STOP_REASON_MOTION_FAULT; }
            }
            unlock();
        }
        /* Never hold the executor mutex while posting to its controller. */
        if ((events.reasons != STOP_REASON_NONE || events.cutoff_fired) &&
            events.stop_events != queued_evidence)
        {
            stop_reason_mask_t reasons = events.reasons;
            if (events.cutoff_fired) { reasons |= STOP_REASON_CONTROLLER_FAULT; }
            if (controller_post_motion_stop(reasons, events.stop_events))
            { queued_evidence = events.stop_events; }
        }
        if (terminal && controller_post_motion_result(&result) && lock())
        {
            (void)measured_motion_executor_ack(&executor, result.goal_id, result.authority_generation);
            unlock();
        }
        if (operational_stop != STOP_REASON_NONE &&
            controller_post_motion_stop(operational_stop, 0U) && lock())
        { reported_generation = executor.status.generation; unlock(); }
        (void)tk_dly_tsk(executor.config.period_us/1000U);
    }
}

ER motion_task_init(const measured_motion_executor_config_t *config,
    const measured_motion_executor_io_t *io, uint32_t boot_session)
{
    drv_motor_t *motor;
    drv_encoder_t *encoder;
    T_CMTX cm = {0};
    T_CTSK ct = {0};
    measured_motion_executor_io_t wrapped = {NULL, stopped, watchdog, cancel_watchdog};
    if (initialized) { return E_OBJ; }
    if (!subsystem2_executor_drivers(&motor, &encoder)) { return E_OBJ; }
    if (io == NULL || io->stop_pending == NULL || io->arm_watchdog == NULL ||
        io->cancel_watchdog == NULL || boot_session == 0U) { return E_PAR; }
    backend = *io;
    if (!measured_motion_executor_init(&executor, motor, encoder, config, &wrapped)) { return E_PAR; }
    cm.mtxatr = TA_INHERIT;
    mutex = tk_cre_mtx(&cm);
    if (mutex < E_OK) { return mutex; }
    ct.tskatr = TA_HLNG | TA_RNG0 | TA_ASSPRC | TA_USERBUF;
    ct.task = (FP)motion_main; ct.itskpri = 6; /* Provisional, above controller. */
    ct.stksz = sizeof(motion_stack); ct.bufptr = motion_stack; ct.assprc = TP_PRC1;
    task = tk_cre_tsk(&ct);
    if (task < E_OK) { (void)tk_del_mtx(mutex); mutex = 0; return task; }
    session = boot_session; initialized = true; return E_OK;
}
ER motion_task_start(void)
{
    ER result;
    if (!initialized || started) { return E_OBJ; }
    result = tk_sta_tsk(task, 0);
    if (result == E_OK) { started = true; }
    return result;
}
bool motion_task_outputs(controller_motion_outputs_t *o)
{
    if (!initialized || started || o == NULL) { return false; }
    *o = (controller_motion_outputs_t){NULL, session, authorize, continuous, goal, cancel, ack_stop};
    return true;
}
bool motion_task_status(measured_motion_executor_status_t *status)
{
    if (status == NULL || !lock()) { return false; }
    *status = executor.status; unlock(); return true;
}
