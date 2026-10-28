#include "subsystem2.h"

static motion_hal_t subsystem_hal;
static drv_motor_t motor;
static drv_encoder_t encoder;
/* Lifecycle is serialized by the all-init-before-any-start owner. Runtime
 * driver state uses the injected common IRQ/core lock. Static storage means
 * no task/queue/allocation failure is concealed by this foundation. */
static bool hal_bound;
static bool initialized;
static bool started;
static bool attached;

ER subsystem2_bind_hal(const motion_hal_t *hal)
{
    if (!motion_hal_is_valid(hal))
    {
        return E_PAR;
    }
    if (hal_bound)
    {
        return E_OBJ;
    }
    subsystem_hal = *hal;
    hal_bound = true;
    return E_OK;
}

EXPORT ER subsystem2_init(void)
{
    motion_encoder_snapshot_t initial_sample;
    if (!hal_bound)
    {
        return E_NOSPT;
    }
    if (initialized || attached)
    {
        (void)drv_motor_stop(&motor);
        return E_OBJ;
    }
    if (drv_motor_init(&motor, &subsystem_hal,
        MOTION_PWM_WRAP_CANDIDATE) != MOTION_OK)
    {
        return E_SYS;
    }
    if (drv_encoder_init(&encoder, &subsystem_hal) != MOTION_OK)
    {
        (void)drv_motor_deinit(&motor);
        return E_SYS;
    }
    if (!subsystem_hal.attach_encoders(subsystem_hal.context,
        drv_encoder_on_edges, &encoder))
    {
        (void)drv_encoder_deinit(&encoder);
        (void)drv_motor_deinit(&motor);
        return E_SYS;
    }
    attached = true;
    if (drv_encoder_capture(&encoder, &motor, &initial_sample) != MOTION_OK)
    {
        (void)subsystem2_shutdown();
        return E_SYS;
    }
    initialized = true;
    return E_OK;
}

EXPORT ER subsystem2_start(void)
{
    motion_motor_state_t state;
    motion_encoder_snapshot_t sample;
    if (!hal_bound)
    {
        return E_NOSPT;
    }
    if (!initialized || started)
    {
        (void)drv_motor_stop(&motor);
        return E_OBJ;
    }
    if ((drv_motor_read(&motor, &state) != MOTION_OK) || !state.ready ||
        state.io_fault || state.sequence_exhausted || state.armed ||
        !state.inhibited ||
        (drv_encoder_capture(&encoder, &motor, &sample) != MOTION_OK))
    {
        (void)subsystem2_shutdown();
        return E_SYS;
    }
    started = true;
    return E_OK;
}

ER subsystem2_shutdown(void)
{
    bool ok;
    if (!hal_bound)
    {
        return E_NOSPT;
    }
    ok = drv_motor_deinit(&motor) == MOTION_OK;
    (void)drv_encoder_deinit(&encoder);
    if (attached)
    {
        if (subsystem_hal.detach_encoders(subsystem_hal.context))
        {
            attached = false;
        }
        else
        {
            ok = false; /* No reinit while a callback may still be live. */
        }
    }
    initialized = false;
    started = false;
    return ok ? E_OK : E_SYS;
}

motion_status_t subsystem2_urgent_stop(void)
{
    return drv_motor_stop(&motor);
}

motion_status_t subsystem2_read_motor(motion_motor_state_t *state)
{
    return drv_motor_read(&motor, state);
}

motion_status_t subsystem2_read_encoders(motion_encoder_snapshot_t *snapshot)
{
    return drv_encoder_capture(&encoder, &motor, snapshot);
}

#if APP_B2_BENCH
drv_motor_t *subsystem2_bench_motor(void) { return started ? &motor : NULL; }
drv_encoder_t *subsystem2_bench_encoder(void) { return started ? &encoder : NULL; }
#endif

#if APP_CAR_RUN
/* DRAFT (Buddy 2 review): the motion executor task is the only caller. */
drv_motor_t *subsystem2_executor_motor(void) { return started ? &motor : NULL; }
drv_encoder_t *subsystem2_executor_encoder(void) { return started ? &encoder : NULL; }
#endif
