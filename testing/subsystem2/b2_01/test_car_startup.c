/* Execute the actual CAR_RUN supervisor with bounded task/peripheral mocks.
 * Verifies wiring and startup ordering, not RTOS or hardware timing. */
#include "../vehicle_controller/car_run_target.h"
#include "../vehicle_controller/controller_task.h"
#include "subsystem2_motion.h"
#include "b2_platform_rp2040.h"
#include "b2_gesture.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem5/subsystem5.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

static FP supervisor;
static jmp_buf boundary;
static const char *scenario;
static unsigned inits,starts,urgent,auto_posts,stop_posts,acks,loops;
static bool qualified,stop_latched;
static controller_status_t status;
static char log_text[4096];
static gpio_dispatcher_t dispatcher;

ID tk_cre_tsk(const T_CTSK *c) {
    assert(c->assprc==TP_PRC1 && c->bufptr!=NULL && c->stksz==4096);
    supervisor=c->task; return 1;
}
ER tk_sta_tsk(ID id,INT code) { (void)code; assert(id==1); return E_OK; }
ER tk_del_tsk(ID id) { (void)id; return E_OK; }
void tk_ext_tsk(void) { longjmp(boundary,1); }
ER tk_dly_tsk(UINT ms) {
    assert(ms==5U);
    if(strcmp(scenario,"stop_ack")==0 && loops==0U) {
        assert(stop_posts==1U && acks==0U); /* FIFO acceptance is not controller application. */
        status.stop_reasons=STOP_REASON_PHYSICAL; status.mode=SYSTEM_MODE_STOPPED;
        stop_latched=true;
    }
    if(++loops==2U) { longjmp(boundary,1); }
    return E_OK;
}
ER tk_get_otm(SYSTIM *t) { t->hi=0U; t->lo=loops*5U; return E_OK; }
void tm_printf(UB *format,...) {
    size_t length=strlen(log_text);
    va_list args; va_start(args,format);
    (void)vsnprintf(log_text+length,sizeof(log_text)-length,(const char *)format,args);
    va_end(args);
}
bool b2_platform_init(motion_hal_t *h,b2_trial_io_t *io) { (void)h; (void)io; return true; }
gpio_dispatcher_t *b2_platform_dispatcher(void) { return &dispatcher; }
bool b2_platform_start_pressed(void) { return false; }
bool b2_platform_stop_pressed(void) { return false; }
uint32_t b2_platform_now_us(void) { return loops*5000U; }
void b2_platform_peek_events(b2_platform_events_t *e) {
    *e=(b2_platform_events_t){0};
    if(strcmp(scenario,"stop_ack")==0 && acks==0U)
    { e->reasons=STOP_REASON_PHYSICAL; e->stop_events=1U; }
}
bool b2_platform_ack_events(uint32_t id) {
    assert(id==1U && stop_latched && (status.stop_reasons&STOP_REASON_PHYSICAL));
    ++acks; return true;
}
b2_ui_event_t b2_gesture_update(b2_gesture_t *g,uint32_t us,bool start,bool stop,bool busy) {
    (void)g; (void)us; (void)start; (void)stop; (void)busy;
    return strcmp(scenario,"stop_ack")==0 ? B2_UI_NONE : B2_UI_RUN;
}
ER subsystem2_bind_hal(const motion_hal_t *h) { (void)h; return E_OK; }
ER subsystem2_init(void) { assert(inits++==0U); return E_OK; }
ER subsystem2_start(void) { assert(inits==7U && starts++==0U); return E_OK; }
motion_status_t subsystem2_urgent_stop(void) { ++urgent; return MOTION_OK; }
motion_status_t subsystem2_read_motor(motion_motor_state_t *s) { *s=(motion_motor_state_t){0}; return MOTION_OK; }
ER subsystem2_motion_init(const b2_trial_io_t *io) { (void)io; assert(inits++==1U); return E_OK; }
ER subsystem2_motion_start(void) { assert(inits==7U && starts++==1U); return E_OK; }
bool subsystem2_motion_qualified(void) { return qualified; }
void subsystem2_motion_halt(void) { }
bool subsystem2_motion_continuous(const motion_continuous_request_t *r) { (void)r; return true; }
bool subsystem2_motion_goal(const motion_goal_request_t *g) { (void)g; return true; }
bool subsystem2_motion_goal_result_for(motion_source_t source,goal_id_t id,motion_goal_result_t *r) {
    assert(source==MOTION_SOURCE_JUNCTION || source==MOTION_SOURCE_OBSTACLE);
    *r=(motion_goal_result_t){id,2U,MOTION_RESULT_PROGRESS,0,0,false}; return true;
}
bool subsystem2_motion_telemetry(subsystem2_motion_telemetry_t *t) { *t=(subsystem2_motion_telemetry_t){0}; return true; }
ER controller_task_init(const controller_outputs_t *o) {
    assert(inits++==2U && o->drive!=NULL && o->read_line!=NULL);
    assert(o->stop_motion(o->context)); return E_OK;
}
ER controller_task_bind_junction(const controller_junction_outputs_t *o) {
    motion_goal_result_t result;
    assert(inits++==3U && o->goal_result(o->context,1U,&result)); return E_OK;
}
ER controller_task_start(void) { assert(starts++==2U); return E_OK; }
bool controller_post(const controller_event_t *e) {
    if(e->type==CONTROLLER_EVENT_SET_MODE && e->mode==SYSTEM_MODE_AUTO) {
        ++auto_posts; status.mode=SYSTEM_MODE_AUTO;
    }
    if(e->type==CONTROLLER_EVENT_LATCH_STOP) { ++stop_posts; }
    return true;
}
bool controller_get_status(controller_status_t *s) { *s=status; return true; }
bool controller_authority(motion_source_t source,authority_generation_t *g) { (void)source; *g=2U; return true; }
ER subsystem3_init(void) { assert(inits++==4U); return strcmp(scenario,"init_failure")==0 ? E_SYS : E_OK; }
ER subsystem3_start(void) { assert(starts++==3U); return E_OK; }
line_status_t subsystem3_line_status(void) { line_status_t s={0}; s.left_black=1U; return s; }
int8_t subsystem3_steer(void) { return 0; }
BOOL subsystem3_take_route(route_instruction_t *r) { (void)r; return FALSE; }
BOOL subsystem3_take_junction(junction_observation_t *j) { (void)j; return FALSE; }
BOOL subsystem3_probe_requested(void) { return FALSE; }
ER subsystem3_probe_begin(void) { return E_OK; }
ER subsystem3_probe_end(void) { return E_OK; }
void subsystem3_reacquire(line_side_t s) { (void)s; }
void subsystem3_calibrate_barcode(void) { }
BOOL subsystem3_barcode_calibrated(void) { return TRUE; }
ER subsystem5_bind(const subsystem5_bindings_t *b) {
    motion_goal_result_t r;
    assert(inits++==5U && b->motion.goal==subsystem2_motion_goal);
    assert(b->motion.goal_result(1U,&r)); return E_OK;
}
ER subsystem5_init(void) { assert(inits++==6U); return E_OK; }
ER subsystem5_start(void) { assert(starts++==4U); return E_OK; }
int main(int argc,char **argv) {
    assert(argc==2); scenario=argv[1];
    qualified=strcmp(scenario,"qualified")==0;
    status.mode=SYSTEM_MODE_IDLE;
    assert(car_run_start()==E_OK);
    if(setjmp(boundary)==0) { ((void (*)(INT,void *))supervisor)(0,NULL); }
    if(strcmp(scenario,"init_failure")==0) { assert(starts==0U && urgent>0U); }
    else {
        assert(inits==7U && starts==5U);
        if(qualified) { assert(auto_posts==1U); }
        else { assert(auto_posts==0U); }
        if(strcmp(scenario,"stop_ack")==0) { assert(acks==1U && stop_posts==1U); }
        else if(!qualified) { assert(strstr(log_text,"START refused")!=NULL); }
    }
    printf("PASS actual CAR_RUN startup %s\n",scenario); return 0;
}
