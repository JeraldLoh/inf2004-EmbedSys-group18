/* DRAFT for Buddy 2's review: motion executor kernel task (CAR_RUN only). */
#include "subsystem2_motion.h"

#if APP_CAR_RUN
#include "subsystem2_internal.h"
#include "motion_executor.h"

#define EXEC_PERIOD_MS 5
#define EXEC_PRIORITY 8        /* provisional: above controller (10) and B5 (12) */
#define EXEC_STACK 1024
#define EXEC_CUTOFF_US 100000U /* hardware cutoff while driving; <= B2_TRIAL_MAX_US */
#define EXEC_LOCK_WAIT_MS 10

static motion_executor_t executor;
static b2_trial_io_t io;
static drv_motor_t *motor;
static drv_encoder_t *encoder;
static ID task;
static ID guard;
static bool initialized;
static bool started;
static uint32_t driver_errors;
static subsystem2_motion_telemetry_t telemetry;

/* Same clock as every other CAR_RUN task (kernel ms, wraps after 49 days). */
static uint32_t now_ms(void)
{
    SYSTIM now;
    (void)tk_get_otm(&now);
    return (uint32_t)now.lo;
}

/* b2_trial_begin() order: STOP released, stop, clear inhibit, arm. */
static bool rearm(authority_generation_t generation)
{
    motion_motor_state_t state;

    if (io.stop_pressed(io.context) || io.stop_pending(io.context))
    {
        return false;
    }
    return (drv_motor_stop(motor) == MOTION_OK) &&
        (drv_motor_read(motor, &state) == MOTION_OK) &&
        (drv_motor_clear_inhibit(motor, state.stop_sequence) == MOTION_OK) &&
        (drv_motor_arm(motor, state.stop_sequence, generation) == MOTION_OK);
}

/* Driver refused or STOP fired: motors off, command ended, and only fresh
 * authority can drive again. */
static void fail(motion_result_code_t code)
{
    (void)drv_motor_stop(motor);
    io.cancel_cutoff(io.context);
    motion_executor_abort(&executor, code);
    driver_errors++;
}

static void service(void)
{
    motion_encoder_snapshot_t snapshot;
    motion_motor_state_t state;
    motion_executor_output_t out;

    if ((drv_encoder_capture(encoder, motor, &snapshot) != MOTION_OK) ||
        !snapshot.acquisition_valid)
    {
        if (executor.mode != MOTION_EXECUTOR_IDLE)
        {
            fail(MOTION_RESULT_FAULT);
        }
        return;
    }
    out = motion_executor_step(&executor, now_ms(), snapshot.left_count,
        snapshot.right_count);

    if (drv_motor_read(motor, &state) != MOTION_OK)
    {
        fail(MOTION_RESULT_FAULT);
        return;
    }
    if (!out.active)
    {
        /* Stay armed at zero so the same authority can drive again. */
        if (state.armed && ((state.left_effort != 0) || (state.right_effort != 0)))
        {
            (void)drv_motor_apply(motor, state.authority_generation, 0, 0);
        }
        io.cancel_cutoff(io.context);
    }
    else if ((!state.armed || (state.authority_generation != out.generation)) &&
        !rearm(out.generation))
    {
        fail(MOTION_RESULT_ABORTED);
    }
    else if (!io.arm_cutoff(io.context, EXEC_CUTOFF_US))
    {
        fail(MOTION_RESULT_ABORTED);   /* STOP/cutoff evidence pending */
    }
    else if (drv_motor_apply(motor, out.generation, out.left, out.right) != MOTION_OK)
    {
        fail(MOTION_RESULT_ABORTED);
    }

    telemetry.sampled_ms = now_ms();
    telemetry.valid = true;
    telemetry.left_effort = executor.last_left;
    telemetry.right_effort = executor.last_right;
    telemetry.left_mm_s = executor.left_mm_s;
    telemetry.right_mm_s = executor.right_mm_s;
    telemetry.mode = (uint32_t)executor.mode;
    telemetry.armed_generation = state.armed ? state.authority_generation : 0U;
    telemetry.accepted = executor.accepted;
    telemetry.rejected = executor.rejected;
    telemetry.driver_errors = driver_errors;
}

static void executor_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;
    for (;;)
    {
        if (tk_loc_mtx(guard, TMO_FEVR) == E_OK)
        {
            service();
            (void)tk_unl_mtx(guard);
        }
        /* Period, or sooner when subsystem2_motion_halt() wakes us. */
        (void)tk_slp_tsk(EXEC_PERIOD_MS);
    }
}

ER subsystem2_motion_init(const b2_trial_io_t *trial_io)
{
    motion_executor_config_t config;
    T_CMTX mtx = {0};
    T_CTSK ctsk = {0};
    ID id;

    if (initialized)
    {
        return E_OBJ;
    }
    if ((trial_io == NULL) || (trial_io->stop_pressed == NULL) ||
        (trial_io->stop_pending == NULL) || (trial_io->arm_cutoff == NULL) ||
        (trial_io->cancel_cutoff == NULL))
    {
        return E_PAR;
    }
    motor = subsystem2_executor_motor();
    encoder = subsystem2_executor_encoder();
    if ((motor == NULL) || (encoder == NULL))
    {
        return E_NOSPT;   /* subsystem2_start() has not succeeded */
    }
    motion_executor_default_config(&config);
    if (!motion_executor_init(&executor, &config))
    {
        return E_PAR;
    }
    io = *trial_io;

    mtx.mtxatr = TA_INHERIT;
    id = tk_cre_mtx(&mtx);
    if (id < E_OK)
    {
        return id;
    }
    guard = id;

    ctsk.tskatr = TA_HLNG | TA_RNG0 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1;   /* core 0, with Buddy 2's platform */
    ctsk.task = (FP)executor_task;
    ctsk.itskpri = EXEC_PRIORITY;
    ctsk.stksz = EXEC_STACK;
    id = tk_cre_tsk(&ctsk);
    if (id < E_OK)
    {
        (void)tk_del_mtx(guard);
        guard = 0;
        return id;
    }
    task = id;
    initialized = true;
    return E_OK;
}

ER subsystem2_motion_start(void)
{
    ER er;

    if (!initialized || started)
    {
        return E_OBJ;
    }
    er = tk_sta_tsk(task, 0);
    if (er == E_OK)
    {
        started = true;
    }
    return er;
}

bool subsystem2_motion_continuous(const motion_continuous_request_t *request)
{
    bool ok;

    if (!started || (request == NULL) ||
        (tk_loc_mtx(guard, EXEC_LOCK_WAIT_MS) != E_OK))
    {
        return false;
    }
    ok = motion_executor_continuous(&executor, request, now_ms());
    (void)tk_unl_mtx(guard);
    return ok;
}

bool subsystem2_motion_goal(const motion_goal_request_t *goal)
{
    motion_encoder_snapshot_t snapshot;
    bool ok = false;

    if (!started || (goal == NULL) ||
        (tk_loc_mtx(guard, EXEC_LOCK_WAIT_MS) != E_OK))
    {
        return false;
    }
    if ((drv_encoder_capture(encoder, motor, &snapshot) == MOTION_OK) &&
        snapshot.acquisition_valid)
    {
        ok = motion_executor_goal(&executor, goal, now_ms(),
            snapshot.left_count, snapshot.right_count);
    }
    (void)tk_unl_mtx(guard);
    return ok;
}

void subsystem2_motion_halt(void)
{
    if (!started)
    {
        return;
    }
    if (tk_loc_mtx(guard, EXEC_LOCK_WAIT_MS) == E_OK)
    {
        motion_executor_halt(&executor);
        (void)tk_unl_mtx(guard);
        (void)tk_wup_tsk(task);   /* zero now, not at the next period */
    }
    else
    {
        /* Executor stuck: latch the driver stop instead. */
        (void)drv_motor_stop(motor);
    }
}

bool subsystem2_motion_goal_result(goal_id_t goal_id, motion_goal_result_t *result)
{
    bool ok;

    if (!started || (tk_loc_mtx(guard, EXEC_LOCK_WAIT_MS) != E_OK))
    {
        return false;
    }
    ok = motion_executor_goal_result(&executor, goal_id, result);
    (void)tk_unl_mtx(guard);
    return ok;
}

bool subsystem2_motion_telemetry(subsystem2_motion_telemetry_t *out)
{
    if (!started || (out == NULL) ||
        (tk_loc_mtx(guard, EXEC_LOCK_WAIT_MS) != E_OK))
    {
        return false;
    }
    *out = telemetry;
    (void)tk_unl_mtx(guard);
    return true;
}
#endif
