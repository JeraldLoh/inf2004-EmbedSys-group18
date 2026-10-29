#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_controller/controller_service.h"
static bool stopped(void *ctx){(void)ctx;return true;}
static void stop(controller_service_t *s,stop_reason_mask_t reasons){
 controller_event_t e={CONTROLLER_EVENT_LATCH_STOP,SYSTEM_MODE_STOPPED,reasons,MISSION_IDLE,false};
 assert(controller_service_handle(s,&e));
}
int main(void){
 controller_service_t s;controller_outputs_t o={NULL,stopped,NULL,NULL};
 assert(controller_service_init(&s,&o));
 assert(controller_service_remote_start(&s,0,0,false));
 stop(&s,STOP_REASON_REMOTE);
 assert(!controller_service_remote_start(&s,0,1,false));
 assert(s.status.mode!=SYSTEM_MODE_AUTO && (s.status.stop_reasons&STOP_REASON_REMOTE));
 assert(controller_service_remote_start(&s,1,1,false));
 stop(&s,STOP_REASON_REMOTE);
 assert(!controller_service_remote_start(&s,1,2,false));
 assert(s.status.stop_reasons&STOP_REASON_REMOTE);
 assert(!controller_service_remote_start(&s,2,2,true));
 assert(s.status.stop_reasons&STOP_REASON_REMOTE);
 stop(&s,STOP_REASON_PHYSICAL);
 assert(!controller_service_remote_start(&s,3,3,false));
 assert(s.status.stop_reasons&STOP_REASON_PHYSICAL);
 assert(!controller_service_remote_start(&s,UINT32_MAX,UINT32_MAX,false));
 puts("PASS stale START cannot clear a newer remote/physical STOP; pending and saturated generations fail closed");
 return 0;
}
