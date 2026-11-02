/* CAR_RUN's single motor task: measured per-wheel PID and finite goals. */
#include "subsystem2_motion.h"
#if APP_CAR_RUN
#include "subsystem2_internal.h"
#include "car_motion.h"
#if APP_CAR_B2_TRIAL
#include "../diagnostics/b2_move_profile.h"
#include "../project_config/car_trial_tuning.h"
#include "b2_line_tuning.h"
#endif
#include "../vehicle_controller/controller_task.h"

static car_motion_t motion;
static b2_trial_io_t io;
static drv_motor_t *motor;
static drv_encoder_t *encoder;
static ID task, guard;
static _Alignas(8) uint32_t stack[1024];
static bool initialized, started;
static uint32_t driver_errors;
#if APP_CAR_B2_TRIAL
static ID diagnostic_task;
static _Alignas(8) uint32_t diagnostic_stack[1024];
/* Atomic last-failure category for failures before the motion mutex. This
 * is not a lossless event counter and never changes motor authority. */
static volatile uint32_t ingress_diagnostic;
enum { INGRESS_NONE, INGRESS_NOT_STARTED, INGRESS_NULL, INGRESS_LOCK };
#endif

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

#if APP_CAR_B2_TRIAL
static const char *engine_status_name(motion_status_t status)
{
    switch (status) {
    case MOTION_OK: return "OK";
    case MOTION_INVALID: return "INVALID_PARAMETERS";
    case MOTION_STATE: return "GOAL_ACTIVE_OR_RESULT_PENDING";
    case MOTION_INHIBITED: return "INHIBITED_OR_UNAUTHORIZED";
    case MOTION_STALE: return "STALE_ID_OR_AUTHORITY";
    case MOTION_IO_ERROR: return "IO_ERROR";
    case MOTION_UNAVAILABLE: return "UNAVAILABLE";
    case MOTION_REVERSAL_REQUIRED: return "REVERSAL_REQUIRED";
    default: return "UNKNOWN";
    }
}
static const char *ingress_name(uint32_t reason)
{
    switch (reason) {
    case INGRESS_NONE: return "NONE";
    case INGRESS_NOT_STARTED: return "NOT_STARTED";
    case INGRESS_NULL: return "NULL_REQUEST";
    case INGRESS_LOCK: return "MOTION_MUTEX_BUSY";
    default: return "UNKNOWN";
    }
}
/* Dedicated low-priority logger: never print in the20ms motor task or while
 * holding its mutex. Periodic snapshots may coalesce repeated events. */
static void diagnostic_worker(INT code, void *exinf)
{
    uint32_t last_detail = 0U, last_rejected = 0U, last_ingress = 0U;
    bool announced = false;
    (void)code; (void)exinf;
    for (;;) {
        if (started && !announced) {
            tm_printf((UB*)"[B2 CONFIG] fw=%s base=%umm/s curve_scale=%lu inner=%u%% extra_inner=%u.%u%% min_line_wheel=%umm/s UNQUALIFIED; this configuration supersedes older CAR banner speed text.\n",
                B2_FIRMWARE_TAG, (unsigned)CONTROLLER_LINE_CRUISE_MM_S,
                (unsigned long)CONTROLLER_LINE_CURVATURE_PER_STEER,
                (unsigned)CAR_TRIAL_LINE_INNER_SPEED_PERCENT,
                (unsigned)(B2_LINE_WIDE_TURN_INNER_PERMILLE / 10U),
                (unsigned)(B2_LINE_WIDE_TURN_INNER_PERMILLE % 10U),
                (unsigned)B2_LINE_MIN_ROLLING_MM_S);
            announced = true;
        }
        if (started && lock()) {
            measured_motion_executor_status_t s = motion.executor.status;
            car_motion_reject_t rejected_reason = motion.last_reject;
            motion_status_t rejected_status = motion.last_reject_status;
            motion_motor_state_t outputs = {0};
            bool have_outputs = drv_motor_read(motor,&outputs) == MOTION_OK;
            uint32_t count = motion.rejected, id = motion.rejected_id;
            uint32_t generation = motion.rejected_generation, floor = motion.line_floor;
            unsigned source = (unsigned)motion.rejected_source;
            uint32_t ingress = ingress_diagnostic;
            unlock();
            tm_printf((UB*)"[B2 TRACK] fw=%s target=%ld/%ld measured=%lu/%lu mm/s speed_valid=%d effort=%d/%d output_valid=%d floor_mask=0x%x state=%d gen=%lu rejected=%lu\n",
                B2_FIRMWARE_TAG,(long)(s.target_um_s[0]/1000),(long)(s.target_um_s[1]/1000),
                (unsigned long)(s.wheels.speed_um_s[0]/1000U),(unsigned long)(s.wheels.speed_um_s[1]/1000U),
                (int)s.wheels.speed_valid,(int)outputs.left_effort,(int)outputs.right_effort,
                (int)have_outputs,(unsigned)s.low_speed_floor_mask,(int)s.state,
                (unsigned long)s.generation,(unsigned long)count);
            if (s.detail_sequence != last_detail) {
                tm_printf((UB*)"[B2 STOP DETAIL] seq=%lu reason=%s result=%d gen=%lu command=%lu wheel_mask=0x%x before_target=%ld/%ld before_speed=%lu/%lu mm/s speed_valid=%d\n",
                    (unsigned long)s.detail_sequence,measured_motion_detail_name(s.detail_reason),
                    (int)s.detail_result,(unsigned long)s.detail_generation,(unsigned long)s.detail_command,
                    (unsigned)s.detail_wheel_mask,(long)(s.detail_target_um_s[0]/1000),
                    (long)(s.detail_target_um_s[1]/1000),
                    (unsigned long)(s.detail_speed_um_s[0]/1000U),
                    (unsigned long)(s.detail_speed_um_s[1]/1000U),(int)s.detail_speed_valid);
                last_detail = s.detail_sequence;
            }
            if (count != last_rejected) {
                tm_printf((UB*)"[B2 REJECT] total=%lu reason=%s engine=%s source=%u id=%lu request_gen=%lu executor_gen=%lu paused_line_gen=%lu state=%d authorized=%d\n",
                    (unsigned long)count,car_motion_reject_name(rejected_reason),engine_status_name(rejected_status),
                    source,(unsigned long)id,(unsigned long)generation,(unsigned long)s.generation,
                    (unsigned long)floor,(int)s.state,(int)s.authorized);
                last_rejected = count;
            }
            if (ingress != last_ingress) {
                tm_printf((UB*)"[B2 INGRESS] last_failure=%s\n",ingress_name(ingress));
                last_ingress = ingress;
            }
        }
        (void)tk_dly_tsk(1000U);
    }
}
#endif

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
#if APP_CAR_B2_TRIAL
    b2_move_candidate_profile(&config); /* Exact standalone v6 parameter builder. */
#else
    car_motion_candidate_config(&config);
#endif
    if (!car_motion_init(&motion, motor, encoder, &config, &measured_io)) { return E_PAR; }
#if APP_CAR_B2_TRIAL
    /* No overall run limit: physical STOP, command leases and faults remain. */
    if(!car_motion_enable_trial(&motion,MEASURED_MOTION_NO_RUN_LIMIT)) { return E_PAR; }
#endif
    mtx.mtxatr = TA_INHERIT;
    guard = tk_cre_mtx(&mtx);
    if (guard < E_OK) { return guard; }
    ct.tskatr = TA_HLNG | TA_RNG0 | TA_ASSPRC | TA_USERBUF;
    ct.assprc = TP_PRC1; ct.task = (FP)executor_task; ct.itskpri = 6;
    ct.stksz = sizeof(stack); ct.bufptr = stack;
    task = tk_cre_tsk(&ct);
    if (task < E_OK) { (void)tk_del_mtx(guard); guard = 0; return task; }
#if APP_CAR_B2_TRIAL
    ct.task = (FP)diagnostic_worker; ct.itskpri = 18;
    ct.stksz = sizeof(diagnostic_stack); ct.bufptr = diagnostic_stack;
    diagnostic_task = tk_cre_tsk(&ct);
    if (diagnostic_task < E_OK) {
        ER failure = diagnostic_task;
        (void)tk_del_tsk(task); (void)tk_del_mtx(guard);
        task = guard = 0; return failure;
    }
#endif
    initialized = true; return E_OK;
}
ER subsystem2_motion_start(void)
{
    ER result;
    if (!initialized || started) { return E_OBJ; }
    result = tk_sta_tsk(task, 0);
#if APP_CAR_B2_TRIAL
    if (result == E_OK) {
        result = tk_sta_tsk(diagnostic_task, 0);
        if (result != E_OK) { (void)drv_motor_stop(motor); }
    }
#endif
    if (result == E_OK) { started = true; }
    return result;
}
bool subsystem2_motion_qualified(void)
{ return initialized && motion.executor.config.qualified; }

bool subsystem2_motion_trial_enabled(void)
{ return initialized && motion.trial_limit_us!=0U; }

bool subsystem2_motion_continuous(const motion_continuous_request_t *request)
{
    motion_encoder_snapshot_t sample;
    bool ok;
    if (!started || request == NULL || !lock()) {
#if APP_CAR_B2_TRIAL
        ingress_diagnostic = !started ? INGRESS_NOT_STARTED : request == NULL ? INGRESS_NULL : INGRESS_LOCK;
#endif
        return false;
    }
    if (drv_encoder_capture(encoder, motor, &sample) != MOTION_OK)
    { ok = car_motion_record_rejection(&motion,CAR_REJECT_ENCODER_CAPTURE,MOTION_UNAVAILABLE,
        request->source,request->command_id,request->authority_generation); }
    else { ok = car_motion_continuous(&motion, request, now_ms(), sample.acquired_us); }
    unlock(); return ok;
}
bool subsystem2_motion_goal(const motion_goal_request_t *request)
{
    motion_encoder_snapshot_t sample;
    bool ok;
    if (!started || request == NULL || !lock()) {
#if APP_CAR_B2_TRIAL
        ingress_diagnostic = !started ? INGRESS_NOT_STARTED : request == NULL ? INGRESS_NULL : INGRESS_LOCK;
#endif
        return false;
    }
    if (drv_encoder_capture(encoder, motor, &sample) != MOTION_OK)
    { ok = car_motion_record_rejection(&motion,CAR_REJECT_ENCODER_CAPTURE,MOTION_UNAVAILABLE,
        request->source,request->goal_id,request->authority_generation); }
    else { ok = car_motion_goal(&motion, request, sample.acquired_us); }
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
    motion_encoder_snapshot_t sample;
    uint32_t age;
    if (out == NULL || !lock()) { return false; }
    if (drv_motor_read(motor, &state) != MOTION_OK) { unlock(); return false; }
    *out = (subsystem2_motion_telemetry_t){0};
    if (drv_encoder_capture(encoder,motor,&sample)!=MOTION_OK) { unlock(); return false; }
    age=sample.acquired_us-e->status.wheels.acquired_us;
    out->sampled_ms = now_ms()-age/1000U;
    out->valid = e->measurement.primed && sample.reset_epoch==e->status.wheels.reset_epoch &&
        age<=e->config.measurement.max_gap_us;
    out->left_effort = state.left_effort; out->right_effort = state.right_effort;
    out->left_mm_s = (int32_t)(e->status.wheels.speed_um_s[0]/1000U);
    out->right_mm_s = (int32_t)(e->status.wheels.speed_um_s[1]/1000U);
    out->left_target_mm_s = e->status.target_um_s[0]/1000;
    out->right_target_mm_s = e->status.target_um_s[1]/1000;
    out->speed_valid = out->valid && e->status.wheels.speed_valid;
    out->qualified = e->config.qualified;
    out->mode = (uint32_t)e->status.state;
    out->armed_generation = state.armed ? state.authority_generation : 0U;
    out->accepted = motion.accepted; out->rejected = motion.rejected;
    out->driver_errors = driver_errors;
    unlock(); return true;
}
#endif
