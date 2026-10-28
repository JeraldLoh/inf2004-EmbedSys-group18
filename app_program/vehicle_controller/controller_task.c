#include "controller_task.h"

#define CONTROLLER_QUEUE_DEPTH 16U
#define CONTROLLER_TASK_PRIORITY 10  /* provisional: above Buddy 5 (12) */
#define CONTROLLER_TASK_STACK 1024
#define CONTROLLER_STATUS_WAIT_MS 10

static controller_service_t service;
static ID queue;
static ID guard;
static ID task;
static bool initialized;
static bool started;

static void controller_task_main(INT stacd, void *exinf)
{
    controller_event_t event;

    (void)stacd;
    (void)exinf;
    for (;;)
    {
        if (tk_rcv_mbf(queue, &event, TMO_FEVR) != (INT)sizeof(event))
        {
            continue;
        }
        if (tk_loc_mtx(guard, TMO_FEVR) == E_OK)
        {
            (void)controller_service_handle(&service, &event);
            (void)tk_unl_mtx(guard);
        }
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

    if (initialized)
    {
        return E_OBJ;
    }
    if (!controller_service_init(&service, outputs))
    {
        return E_PAR;
    }

    mtx.mtxatr = TA_INHERIT;
    id = tk_cre_mtx(&mtx);
    if (id < E_OK) { return id; }
    guard = id;

    mbf.mbfatr = TA_TFIFO;
    mbf.maxmsz = (INT)sizeof(controller_event_t);
    mbf.bufsz = (SZ)(CONTROLLER_QUEUE_DEPTH *
        (sizeof(controller_event_t) + sizeof(UW)));
    id = tk_cre_mbf(&mbf);
    if (id < E_OK) { release(); return id; }
    queue = id;

    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task = (FP)controller_task_main;
    ctsk.itskpri = CONTROLLER_TASK_PRIORITY;
    ctsk.stksz = CONTROLLER_TASK_STACK;
    id = tk_cre_tsk(&ctsk);
    if (id < E_OK) { release(); return id; }
    task = id;

    initialized = true;
    return E_OK;
}

ER controller_task_start(void)
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

bool controller_post(const controller_event_t *event)
{
    if (!started || (event == NULL))
    {
        return false;
    }
    return tk_snd_mbf(queue, (void *)event, (INT)sizeof(*event), TMO_POL) == E_OK;
}

bool controller_get_status(controller_status_t *status)
{
    if (!initialized || (status == NULL) ||
        (tk_loc_mtx(guard, CONTROLLER_STATUS_WAIT_MS) != E_OK))
    {
        return false;
    }
    controller_service_status(&service, status);
    (void)tk_unl_mtx(guard);
    return true;
}
