#include "subsystem1.h"
#include "buddy1_bridge.h"
static int initialised;
static ID telemetry_task;
EXPORT ER subsystem1_init(void)
{
    if (initialised) return E_OBJ;
    ER er = b1_init();
    if (er == E_OK) initialised = 1;
    return er;
}
EXPORT ER subsystem1_start(void)
{
    if (!initialised || telemetry_task > 0) return E_OBJ;
    T_CTSK task = {0};
    task.task = (FP)b1_telemetry_task;
    task.itskpri = 18; /* Provisional; measure with team tasks before integration. */
    task.stksz = 4096;
    task.tskatr = TA_HLNG | TA_RNG3;
    ID id = tk_cre_tsk(&task);
    if (id < E_OK) return id;
    ER er = tk_sta_tsk(id, 0);
    if (er < E_OK) { (void)tk_del_tsk(id); return er; }
    telemetry_task = id;
    return E_OK;
}
