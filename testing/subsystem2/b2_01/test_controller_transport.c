/* Deterministic transport mock; NOT an RTOS timing or SMP qualification. */
#include "../vehicle_controller/controller_task.h"
#include <tk/smp.h>
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

static unsigned char messages[16][256];
static unsigned head,count,message_size,receives,stops,continuous_calls;
static motion_continuous_request_t last_continuous;
static FP entry;
static jmp_buf boundary;
ID tk_cre_mtx(const T_CMTX *c) { (void)c; return 1; }
ER tk_del_mtx(ID id) { (void)id; return E_OK; }
ER tk_loc_mtx(ID id, INT t) { (void)id; (void)t; return E_OK; }
ER tk_unl_mtx(ID id) { (void)id; return E_OK; }
ID tk_cre_mbf(const T_CMBF *c) { assert(c->maxmsz<=256); message_size=(unsigned)c->maxmsz; return 2; }
ER tk_del_mbf(ID id) { (void)id; return E_OK; }
ID tk_cre_tsk(const T_CTSK *c) { entry=c->task; return 3; }
ER tk_sta_tsk(ID id,INT code) { (void)id; (void)code; return E_OK; }
ER tk_del_tsk(ID id) { (void)id; return E_OK; }
ER tk_get_otm(SYSTIM *s) { s->hi=0U; s->lo=10U; return E_OK; }
ER tk_snd_mbf(ID id,const void *m,INT size,INT t) {
    (void)id; assert(t==TMO_POL && size==(INT)message_size);
    if(count==16U) { return E_SYS; }
    memcpy(messages[(head+count)%16U],m,(size_t)size); ++count; return E_OK;
}
INT tk_rcv_mbf(ID id,void *m,INT t) {
    (void)id; assert(t==10);
    if(receives++!=0U) { longjmp(boundary,1); }
    if(count==0U) { return E_SYS; }
    memcpy(m,messages[head],message_size); head=(head+1U)%16U; --count; return (INT)message_size;
}
ER InitSpinLock(T_SPLOCK *lock) { *lock=0U; return E_OK; }
ER ISpinLock(T_SPLOCK *lock,UINT *saved) { assert(*lock==0U); *lock=1U; *saved=0U; return E_OK; }
ER ISpinUnlock(T_SPLOCK *lock,UINT saved) { (void)saved; assert(*lock==1U); *lock=0U; return E_OK; }
static void run_once(void) { receives=0U; if(setjmp(boundary)==0) { ((void (*)(INT,void *))entry)(0,NULL); } }
static bool stop(void *p) { (void)p; ++stops; return true; }
static motion_status_t authorize(void *p,uint32_t gen) { (void)p; assert(gen>0U); return MOTION_OK; }
static motion_status_t continuous(void *p,const motion_continuous_request_t *r,uint32_t ttl) {
    (void)p; assert(ttl>0U); ++continuous_calls; last_continuous=*r; return MOTION_OK;
}
static motion_status_t goal(void *p,const motion_goal_request_t *r,uint32_t ttl) {
    (void)p; (void)r; (void)ttl; return MOTION_OK;
}
static void ack(void *p,uint32_t id) { (void)p; (void)id; }
int main(int argc,char **argv) {
    controller_outputs_t o={NULL,stop};
    controller_motion_outputs_t mo={NULL,42U,authorize,continuous,goal,stop,ack};
    controller_event_t mode={CONTROLLER_EVENT_SET_MODE,SYSTEM_MODE_MANUAL,0U,MISSION_IDLE,false};
    controller_event_t halt={CONTROLLER_EVENT_LATCH_STOP,SYSTEM_MODE_STOPPED,STOP_REASON_REMOTE,MISSION_IDLE,false};
    controller_motion_request_t r={0};
    controller_motion_status_t status;
    unsigned i,before;
    assert(argc==2 && controller_task_init(&o)==E_OK);
    assert(controller_task_bind_motion(&mo,42U,15U,30U)==E_OK);
    assert(controller_task_start()==E_OK);
    assert(controller_post(&mode)); run_once();
    assert(controller_get_motion_status(&status));
    r.kind=CONTROLLER_MOTION_START; r.session=42U; r.id=1U;
    r.generation=status.authority_generation; r.source=MOTION_SOURCE_MANUAL;
    r.created_ms=0U; r.expires_ms=1000U;
    assert(controller_post_motion(&r)); run_once();
    assert(controller_get_motion_status(&status) && status.enabled);
    r.generation=status.authority_generation;
    if(strcmp(argv[1],"latest")==0) {
        r.kind=CONTROLLER_MOTION_CONTINUOUS;
        for(i=2U;i<32U;++i) {
            r.id=i; r.value.continuous.speed_mm_s=(int32_t)(i%30U);
            assert(controller_post_motion(&r));
        }
        assert(count==16U); run_once();
        assert(continuous_calls==1U && last_continuous.command_id==31U);
        while(count) { run_once(); }
        assert(continuous_calls==1U);
    } else {
        r.kind=CONTROLLER_MOTION_PROFILE; r.value.profile=MOTION_PROFILE_CAUTION;
        for(i=2U;i<18U;++i) { r.id=i; assert(controller_post_motion(&r)); }
        assert(count==16U && !controller_post_motion(&r));
        if(strcmp(argv[1],"stop")==0) {
            before=stops;
            assert(controller_post(&halt) && stops==before+1U && controller_urgent_stop_pending());
            while(count) { run_once(); }
            assert(!controller_urgent_stop_pending());
            assert(controller_get_motion_status(&status) && !status.enabled);
            assert(continuous_calls==0U);
        } else {
            motion_goal_result_t result={7U,r.generation,MOTION_RESULT_CANCELLED,0,0,false};
            assert(!controller_post_motion_result(&result)); /* Producer must retain/retry. */
            run_once(); assert(controller_post_motion_result(&result));
            while(count) { run_once(); }
            assert(controller_get_motion_status(&status) && status.terminal_valid);
            assert(status.last_terminal.goal_id==7U && status.terminal_sequence==1U);
        }
    }
    printf("PASS transport %s\n",argv[1]); return 0;
}
