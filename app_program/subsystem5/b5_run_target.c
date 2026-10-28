#include "b5_run_target.h"
#if APP_B5_RUN
#include "subsystem5.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/b2_platform_rp2040.h"
#include "../vehicle_controller/controller_task.h"

#define CONSOLE_WAIT_MS 5000U

#if TM_CONSOLE_USB_CDC
IMPORT UW tm_usb_state(void); /* libtm USB console: 3 = port opened (DTR) */
#endif

/* B2's driver keeps a pointer to the HAL, so it must outlive startup. */
static motion_hal_t hal;
static b2_trial_io_t trial_io;

static bool urgent_stop(void)
{
    return subsystem2_urgent_stop() == MOTION_OK;
}

static bool controller_stop(void *context)
{
    (void)context;
    return urgent_stop();
}

/* Startup lines are printed a few ms after power-on, before the PC has
 * opened the COM port, and the console drops what nobody reads. So: start
 * everything first (STOP works from then on), then wait up to 5 s for a
 * serial monitor, then report. Monitors that never assert DTR just see the
 * report 5 s late. */
static void wait_for_console(void)
{
#if TM_CONSOLE_USB_CDC
    UW waited = 0U;
    while ((tm_usb_state() < 3U) && (waited < CONSOLE_WAIT_MS))
    {
        (void)tk_dly_tsk(100);
        waited += 100U;
    }
    (void)tk_dly_tsk(200); /* let the monitor settle before the first line */
#endif
}

static const char *failure;

static ER bring_up(void)
{
    /* No drive/line outputs and no motion bindings: obstacle-only build. */
    controller_outputs_t outputs = {NULL, controller_stop, NULL, NULL};
    subsystem5_bindings_t bindings = {0};
    controller_event_t auto_mode = {CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO,
        STOP_REASON_NONE, MISSION_IDLE, false};
    ER er;

    /* 1. Motor/STOP/GPIO-interrupt foundation. Motors stay inhibited. */
    if (!b2_platform_init(&hal, &trial_io) || (subsystem2_bind_hal(&hal) != E_OK) ||
        (subsystem2_init() != E_OK) || (subsystem2_start() != E_OK))
    {
        (void)subsystem2_urgent_stop();
        failure = "motion foundation failed; nothing started";
        return E_SYS;
    }

    /* 2. Every init before any start. */
    bindings.dispatcher = b2_platform_dispatcher();
    bindings.urgent_stop = urgent_stop;
    bindings.post_event = controller_post;
    bindings.controller_status = controller_get_status;
    er = controller_task_init(&outputs);
    if (er == E_OK) { er = subsystem5_bind(&bindings); }
    if (er == E_OK) { er = subsystem5_init(); }
    if (er != E_OK)
    {
        failure = "init failed; no task started";
        return er;
    }

    /* 3. Start: controller first, so Buddy 5's first events have a reader. */
    er = controller_task_start();
    if (er == E_OK) { er = subsystem5_start(); }
    if ((er != E_OK) || !controller_post(&auto_mode))
    {
        (void)subsystem2_urgent_stop();
        failure = "start failed";
        return (er != E_OK) ? er : E_SYS;
    }
    return E_OK;
}

static _Alignas(8) uint32_t startup_stack[512];
static ID startup_task_id;

/* B2's platform and the GPIO dispatcher only accept setup from core 0, so
 * bring-up runs in a short-lived task pinned there (as B2_BENCH does). */
static void startup_task(INT code, void *exinf)
{
    ER er;

    (void)code;
    (void)exinf;
    er = bring_up();
    wait_for_console();
    if (er != E_OK)
    {
        tm_printf((UB*)"[B5 RUN] %s (%d). Motors held stopped.\n", failure, (int)er);
    }
    else
    {
        tm_printf((UB*)"[B5 RUN] obstacle task + controller running. Sonar GP16/17, servo GP12.\n");
        tm_printf((UB*)"[B5 RUN] Motors stay inhibited: nothing drives the wheels in this build\n");
        tm_printf((UB*)"         (no Buddy 2 move executor). Turns/moves are refused, so a wide\n");
        tm_printf((UB*)"         obstacle ends in a latched safe stop. GP21 STOP always works.\n");
        tm_printf((UB*)"[B5 RUN] Status line every 1 s; hand under 25 cm = obstacle.\n");
    }
    tk_ext_tsk();
}

ER b5_run_start(void)
{
    T_CTSK task = {0};
    ER status;

    if (startup_task_id != 0)
    {
        return E_OBJ;
    }
    task.tskatr = TA_HLNG | TA_RNG0 | TA_USERBUF | TA_ASSPRC;
    task.task = (FP)startup_task;
    task.itskpri = 5;
    task.stksz = sizeof(startup_stack);
    task.bufptr = startup_stack;
    task.assprc = TP_PRC1; /* TP_PRC1 is core 0 */
    startup_task_id = tk_cre_tsk(&task);
    if (startup_task_id < E_OK)
    {
        status = startup_task_id;
        startup_task_id = 0;
        return status;
    }
    status = tk_sta_tsk(startup_task_id, 0);
    if ((status != E_OK) && (tk_del_tsk(startup_task_id) == E_OK))
    {
        startup_task_id = 0;
    }
    return status;
}
#endif
