/* CAR_RUN's single motor task: measured per-wheel PID and finite goals. */
#include "subsystem2_motion.h"
#if APP_CAR_RUN
#include "subsystem2_internal.h"
#include "car_motion.h"
#include "../vehicle_controller/controller_task.h"

static car_motion_t motion;
static b2_trial_io_t io;
static drv_motor_t *motor;
static drv_encoder_t *encoder;
static ID task, guard;
static _Alignas(8) uint32_t stack[1024];
static bool initialized, started;
static uint32_t driver_errors;

static uint32_t now_ms(void)
{
    SYSTIM now;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}
static bool pending(void *context)
{
    (void)context;
    return controller_urgent_stop_pending() || io.stop_pending(io.context);
}
static bool watchdog(void *context, uint32_t duration)
{ (void)context; return !pending(NULL) && io.arm_cutoff(io.context, duration); }
static void cancel_watchdog(void *context)
{ (void)context; io.cancel_cutoff(io.context); }
static bool lock(void)
{ return initialized && tk_loc_mtx(guard, 10) == E_OK; }
static void unlock(void) { (void)tk_unl_mtx(guard); }

static void executor_task(INT code, void *exinf)
{
    stop_reason_mask_t fault_pending = STOP_REASON_NONE;
    (void)code; (void)exinf;
    for (;;)
    {
        controller_status_t controller;
        bool known = controller_get_status(&controller);
        if (lock())
        {
            measured_motion_executor_t *e = &motion.executor;
            bool authorized = e->status.authorized;
            /* Status was copied before taking this mutex. Another core may
             * already have submitted a newer controller generation meanwhile;
             * an older snapshot must not revoke that fresh work. ISR STOP is
             * still independently enforced by the driver's permit ticket. */
            bool current = known && (e->status.generation == 0U ||
                controller.authority_generation-e->status.generation < 0x80000000UL);
            if (!known || (current && (controller.mode != SYSTEM_MODE_AUTO ||
                controller.stop_reasons != STOP_REASON_NONE || controller.hardware_faults != 0U)))
            { car_motion_cancel(&motion, MOTION_RESULT_CANCELLED); }
            if (current || (known && controller.authority_generation == 0U))
            { car_motion_generation(&motion, controller.authority_generation); }
            car_motion_step(&motion);
            if (authorized && !e->status.authorized &&
                (e->status.last_stop == MOTION_RESULT_FAULT ||
                 e->status.last_stop == MOTION_RESULT_ABORTED ||
                 e->status.last_stop == MOTION_RESULT_TIMEOUT))
            { ++driver_errors; fault_pending = STOP_REASON_MOTION_FAULT; }
            unlock();
        }
        /* No controller calls while holding the motion mutex: the controller
         * tick owns its guard while submitting a line or junction command. */
        if (fault_pending != STOP_REASON_NONE)
        {
            controller_event_t stop = {CONTROLLER_EVENT_LATCH_STOP, SYSTEM_MODE_STOPPED,
                fault_pending, MISSION_IDLE, false};
            if (controller_post(&stop)) { fault_pending = STOP_REASON_NONE; }
        }
        (void)tk_dly_tsk(motion.executor.config.period_us/1000U);
    }
}

ER subsystem2_motion_init(const b2_trial_io_t *trial_io)
{
    measured_motion_executor_config_t config;
    measured_motion_executor_io_t measured_io = {NULL, pending, watchdog, cancel_watchdog};
    T_CMTX mtx = {0};
    T_CTSK ct = {0};
    if (initialized) { return E_OBJ; }
    if (trial_io == NULL || trial_io->stop_pending == NULL ||
        trial_io->arm_cutoff == NULL || trial_io->cancel_cutoff == NULL) { return E_PAR; }
    motor = subsystem2_executor_motor(); encoder = subsystem2_executor_encoder();
    if (motor == NULL || encoder == NULL) { return E_OBJ; }
    io = *trial_io;
    car_motion_candidate_config(&config);
    if (!car_motion_init(&motion, motor, encoder, &config, &measured_io)) { return E_PAR; }
    mtx.mtxatr = TA_INHERIT;
    guard = tk_cre_mtx(&mtx);
    if (guard < E_OK) { return guard; }
    ct.tskatr = TA_HLNG | TA_RNG0 | TA_ASSPRC | TA_USERBUF;
    ct.assprc = TP_PRC1; ct.task = (FP)executor_task; ct.itskpri = 6;
    ct.stksz = sizeof(stack); ct.bufptr = stack;
    task = tk_cre_tsk(&ct);
    if (task < E_OK) { (void)tk_del_mtx(guard); guard = 0; return task; }
    initialized = true; return E_OK;
}
ER subsystem2_motion_start(void)
{
    ER result;
    if (!initialized || started) { return E_OBJ; }
    result = tk_sta_tsk(task, 0);
    if (result == E_OK) { started = true; }
    return result;
}
bool subsystem2_motion_qualified(void)
{ return initialized && motion.executor.config.qualified; }

bool subsystem2_motion_continuous(const motion_continuous_request_t *request)
{
    motion_encoder_snapshot_t sample;
    bool ok;
    if (!started || request == NULL || !lock()) { return false; }
    ok = drv_encoder_capture(encoder, motor, &sample) == MOTION_OK &&
        car_motion_continuous(&motion, request, now_ms(), sample.acquired_us);
    unlock(); return ok;
}
bool subsystem2_motion_goal(const motion_goal_request_t *request)
{
    motion_encoder_snapshot_t sample;
    bool ok;
    if (!started || request == NULL || !lock()) { return false; }
    ok = drv_encoder_capture(encoder, motor, &sample) == MOTION_OK &&
        car_motion_goal(&motion, request, sample.acquired_us);
    unlock(); return ok;
}
void subsystem2_motion_halt(void)
{
    if (!initialized) { (void)subsystem2_urgent_stop(); return; }
    if (lock()) { car_motion_halt(&motion); unlock(); }
    else { (void)drv_motor_stop(motor); }
}
bool subsystem2_motion_goal_result_for(motion_source_t source, goal_id_t id,
    motion_goal_result_t *result)
{
    bool ok;
    if (!started || result == NULL || !lock()) { return false; }
    ok = car_motion_result(&motion, source, id, result);
    unlock(); return ok;
}
bool subsystem2_motion_telemetry(subsystem2_motion_telemetry_t *out)
{
    motion_motor_state_t state;
    measured_motion_executor_t *e = &motion.executor;
    if (out == NULL || !lock()) { return false; }
    if (drv_motor_read(motor, &state) != MOTION_OK) { unlock(); return false; }
    *out = (subsystem2_motion_telemetry_t){0};
    out->left_effort = state.left_effort; out->right_effort = state.right_effort;
    out->left_mm_s = (int32_t)(e->status.wheels.speed_um_s[0]/1000U);
    out->right_mm_s = (int32_t)(e->status.wheels.speed_um_s[1]/1000U);
    out->left_target_mm_s = e->status.target_um_s[0]/1000;
    out->right_target_mm_s = e->status.target_um_s[1]/1000;
    out->speed_valid = e->status.wheels.speed_valid;
    out->qualified = e->config.qualified;
    out->mode = (uint32_t)e->status.state;
    out->armed_generation = state.armed ? state.authority_generation : 0U;
    out->accepted = motion.accepted; out->rejected = motion.rejected;
    out->driver_errors = driver_errors;
    unlock(); return true;
}
#endif
