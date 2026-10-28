#include "b2_bench_target.h"
#if APP_B2_BENCH
#include "subsystem2_internal.h"
#include "b2_platform_rp2040.h"
#include "b2_gesture.h"

static void b2_bench_run(void)
{
    motion_hal_t hal;
    b2_trial_io_t io;
    b2_trial_t trial;
    vehicle_controller_t controller;
    b2_trial_result_t result;
    b2_platform_events_t events = {0};
    b2_gesture_t gesture = {0};
    uint32_t last_print = 0U;
    static const int16_t left[6] = {150, -150, 0, 0, 150, -150};
    static const int16_t right[6] = {0, 0, 150, -150, 150, -150};
    if (!b2_platform_init(&hal, &io) || (subsystem2_bind_hal(&hal) != E_OK) ||
        (subsystem2_init() != E_OK) || (subsystem2_start() != E_OK))
    {
        (void)subsystem2_urgent_stop();
        tm_printf((UB*)"[B2] INIT FAILED; no trial permitted. Verify outputs; reset after diagnosis.\n");
        return;
    }
    vehicle_controller_init(&controller);
    if (!b2_trial_init(&trial, subsystem2_bench_motor(), subsystem2_bench_encoder(),
        &controller, &io))
    {
        (void)subsystem2_shutdown();
        return;
    }
    tm_printf((UB*)"[B2 BENCH] production drivers, raised wheels only. NO boot motion.\n");
    tm_printf((UB*)"GP21 STOP always. GP20 short press cycles cases 0..5.\n");
    tm_printf((UB*)"0=L+,1=L-,2=R+,3=R-,4=both+,5=both-. Effort=15%%, time=250ms.\n");
    tm_printf((UB*)"Hold GP20 >=1s then RELEASE to run selected case ONCE.\n");
    for (;;)
    {
        uint32_t now = b2_platform_now_us();
        bool start = b2_platform_start_pressed();
        bool stop = b2_platform_stop_pressed();
        b2_platform_take_events(&events);
        b2_trial_service(&trial, events.reasons, events.cutoff_fired);
        if (b2_trial_take_result(&trial, &result))
        {
            gesture.phase = 0U;
            gesture.since = now;
            tm_printf((UB*)"[B2] trial=%lu gen=%lu end=%u L=%lu R=%lu counts_valid=%u\n",
                (unsigned long)result.trial_id, (unsigned long)result.generation,
                (unsigned)result.state, (unsigned long)result.left_edges,
                (unsigned long)result.right_edges, (unsigned)result.counts_valid);
            tm_printf((UB*)"[B2] last ISR shutdown-call dt=%luus; lock hold/wait max=%lu/%luus (physical pins UNMEASURED)\n",
                (unsigned long)(events.output_command_us - events.confirmed_us),
                (unsigned long)events.max_lock_hold_us,
                (unsigned long)events.max_lock_wait_us);
            tm_printf((UB*)"[B2] controller stop reasons=0x%lx; end 2=time,3=STOP,4=failure (not distance completion)\n",
                (unsigned long)controller.safety.active_reasons);
        }
        {
            b2_ui_event_t ui = b2_gesture_update(&gesture, now, start, stop,
                trial.result.state == B2_TRIAL_RUNNING);
            if (ui == B2_UI_SELECTED)
            { tm_printf((UB*)"[B2] selected case=%u\n", gesture.selected); }
            else if (ui == B2_UI_CONFIRM)
            { tm_printf((UB*)"[B2] release GP20 to run case=%u once; GP21 cancels.\n", gesture.selected); }
            else if (ui == B2_UI_RUN)
            {
                motion_status_t status = b2_trial_begin(&trial, left[gesture.selected],
                    right[gesture.selected], 250000U);
                if (status != MOTION_OK)
                { tm_printf((UB*)"[B2] trial rejected status=%u; remains stopped.\n", (unsigned)status); }
            }
        }
        if ((trial.result.state != B2_TRIAL_RUNNING) && ((now - last_print) >= 1000000U))
        {
            motion_encoder_snapshot_t sample;
            if (subsystem2_read_encoders(&sample) == MOTION_OK)
            {
                tm_printf((UB*)"[B2] idle case=%u raw L=%lu R=%lu epoch=%lu\n", gesture.selected,
                    (unsigned long)sample.left_count, (unsigned long)sample.right_count,
                    (unsigned long)sample.reset_epoch);
            }
            last_print = now;
        }
        (void)tk_dly_tsk(10);
    }
}

static _Alignas(8) uint32_t bench_stack[1024];
static ID bench_task_id;

static void bench_task(INT code, void *exinf)
{
    (void)code; (void)exinf;
    b2_bench_run();
    tk_ext_tsk(); /* Failed init returns here; never enter a drive loop. */
}

ER b2_bench_start(void)
{
    T_CTSK task = {0};
    ER status;
    if (bench_task_id != 0) { return E_OBJ; }
    task.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF | TA_ASSPRC;
    task.task = (FP)bench_task;
    task.itskpri = 5;
    task.stksz = sizeof(bench_stack);
    task.bufptr = bench_stack;
    task.assprc = TP_PRC1; /* TP_PRC1 is core 0, not core 1. */
    bench_task_id = tk_cre_tsk(&task);
    if (bench_task_id < E_OK)
    {
        status = bench_task_id;
        bench_task_id = 0;
        return status;
    }
    status = tk_sta_tsk(bench_task_id, 0);
    if (status != E_OK)
    {
        if (tk_del_tsk(bench_task_id) == E_OK) { bench_task_id = 0; }
    }
    return status;
}
#endif
