#include "b2_bench_target.h"
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

static unsigned creates;
static unsigned deletes;
static bool fail_create;
static bool fail_start;

ID tk_cre_tsk(const T_CTSK *task)
{
    ++creates;
    assert(task->tskatr & TA_ASSPRC);
    assert(task->assprc == TP_PRC1);
    assert(task->tskatr & TA_USERBUF);
    assert(task->stksz == 4096 && task->bufptr != NULL);
    assert(((uintptr_t)task->bufptr % 8U) == 0U);
    assert(task->task != NULL && task->itskpri == 5);
    return fail_create ? E_SYS : 7;
}
ER tk_sta_tsk(ID id, INT code)
{ assert(id == 7 && code == 0); return fail_start ? E_SYS : E_OK; }
ER tk_del_tsk(ID id) { assert(id == 7); ++deletes; return E_OK; }
void tk_ext_tsk(void) { assert(false); }
ER tk_dly_tsk(UINT delay) { (void)delay; assert(false); return E_SYS; }
void tm_printf(UB *format, ...) { (void)format; }

int main(int argc, char **argv)
{
    if ((argc == 2) && strcmp(argv[1], "create") == 0) { fail_create = true; }
    if ((argc == 2) && strcmp(argv[1], "start") == 0) { fail_start = true; }
    if (fail_create || fail_start)
    {
        assert(b2_bench_start() == E_SYS);
        assert(deletes == (fail_start ? 1U : 0U));
        fail_create = false; fail_start = false;
    }
    assert(b2_bench_start() == E_OK);
    assert(b2_bench_start() == E_OBJ);
    assert(creates == ((argc == 2) ? 2U : 1U));
    (void)puts("Core-0 bench task, explicit stack, create/start rollback, duplicate-start checks passed");
    return 0;
}
