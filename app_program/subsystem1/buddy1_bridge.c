#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>
#include "../app.h"
#include "buddy1_bridge.h"
#include "buddy1_diagnostics.h"
/* All queue objects are created before ready is published. */
static ID guard, incoming, vehicle;
static atomic_int ready, online;
static atomic_uint epoch;
static b1_packet slots[4];
static int occupied[4], have_snapshot;
static b1_diagnostics diagnostic;
static atomic_uint busy_submissions;
static b1_binary_snapshot latest;
static unsigned int snapshot_at;
typedef struct { unsigned int length, epoch, at; char data[128]; } incoming_msg;
typedef struct { unsigned int epoch, at; b1_command command; } vehicle_msg;
static unsigned int now_ms(void) { SYSTIM t; tk_get_otm(&t); return t.lo; }
static int lock(void) { return atomic_load_explicit(&ready,memory_order_acquire) && tk_loc_mtx(guard,TMO_POL)==E_OK; }
static ID buffer(unsigned int size) {
 T_CMBF c={0}; c.mbfatr=TA_TFIFO; c.maxmsz=size; c.bufsz=8*(size+sizeof(unsigned int));
 return tk_cre_mbf(&c);
}
int b1_init(void) {
 if(atomic_load_explicit(&ready,memory_order_acquire)) return E_OBJ;
 T_CMTX c={0}; c.mtxatr=TA_INHERIT;
 guard=tk_cre_mtx(&c); if(guard<0) return guard;
 incoming=buffer(sizeof(incoming_msg));
 if(incoming<0) { tk_del_mtx(guard); return incoming; }
 vehicle=buffer(sizeof(vehicle_msg));
 if(vehicle<0) { tk_del_mbf(incoming); tk_del_mtx(guard); return vehicle; }
 atomic_store_explicit(&ready,1,memory_order_release); return E_OK;
}
void b1_connection(int value) {
 if(!atomic_load_explicit(&ready,memory_order_acquire)) return;
 int previous=atomic_load(&online);
 if(!value) atomic_store(&online,0);
 if(previous!=value) atomic_store(&epoch,atomic_load(&epoch)+1);
 if(value) atomic_store(&online,1);
 /* Consumers reject messages from any previous connection epoch. */
}
int b1_network_receive(const char *data,unsigned int n) {
 incoming_msg m;
 if(!atomic_load_explicit(&ready,memory_order_acquire) || !atomic_load(&online) || n>sizeof(m.data)) return 0;
 m.length=n; m.epoch=atomic_load(&epoch); m.at=now_ms(); memcpy(m.data,data,n);
 return tk_snd_mbf(incoming,&m,sizeof(m),TMO_POL)==E_OK;
}
int b1_submit_snapshot(const b1_snapshot *s) {
 b1_binary_snapshot candidate={0};b1_validation_error error;
 if(s){candidate.base_valid=1;candidate.speed_percent=s->speed_percent;candidate.distance_cm=s->distance_cm;}
 int valid=b1_binary_validate(s?&candidate:0,&error);
 if(!lock()){atomic_fetch_add(&busy_submissions,1);return 0;}
 if(!valid){++diagnostic.rejected;if(error.reason==2)++diagnostic.out_of_range;
  diagnostic.last=error;diagnostic.last_at_ms=now_ms();tk_unl_mtx(guard);return 0;}
 latest.base_valid=1;latest.speed_percent=s->speed_percent;latest.distance_cm=s->distance_cm;
 have_snapshot=1; snapshot_at=now_ms(); tk_unl_mtx(guard); return 1;
}
int b1_submit_binary_snapshot(const b1_binary_snapshot *s) {
 b1_validation_error error;int valid=b1_binary_validate(s,&error);
 if(!lock()){atomic_fetch_add(&busy_submissions,1);return 0;}
 if(!valid){++diagnostic.rejected;if(error.reason==2)++diagnostic.out_of_range;
  diagnostic.last=error;diagnostic.last_at_ms=now_ms();tk_unl_mtx(guard);return 0;}
 latest=*s;have_snapshot=1;snapshot_at=now_ms();tk_unl_mtx(guard);return 1;
}
int b1_network_take(unsigned int kind,b1_packet *p) {
 int result=0;
 if(kind>3 || !p || !lock()) return 0;
 if(occupied[kind]) { *p=slots[kind]; occupied[kind]=0; result=1; }
 tk_unl_mtx(guard);
 if(result && (unsigned int)(now_ms()-p->created_ms)>(kind==B1_HEARTBEAT?5000u:1500u)) return 0;
 return result;
}
void b1_network_restore(const b1_packet *p) {
 if(p->kind>3 || !lock()) return;
 if(!occupied[p->kind]) { slots[p->kind]=*p;occupied[p->kind]=1; }
 tk_unl_mtx(guard);
}
static int put_bytes(unsigned int kind,const void *data,unsigned int length,unsigned int at) {
 if(kind>3 || !data || length>B1_PAYLOAD_MAX) return 0;
 if(!lock()) return 0;
 slots[kind].kind=kind; slots[kind].created_ms=at;slots[kind].length=length;
 memcpy(slots[kind].text,data,length);
 occupied[kind]=1; tk_unl_mtx(guard); return 1;
}
static int put(unsigned int kind,const char *text,unsigned int at) {
 return put_bytes(kind,text,(unsigned int)strlen(text),at);
}
int b1_receive_vehicle(b1_command *c) {
 vehicle_msg m;
 if(!c || !atomic_load_explicit(&ready,memory_order_acquire)) return 0;
 while(tk_rcv_mbf(vehicle,&m,TMO_POL)==sizeof(m)) {
  if(atomic_load(&online) && m.epoch==atomic_load(&epoch) && (unsigned int)(now_ms()-m.at)<=1000) { *c=m.command;return 1; }
 }
 return 0;
}
void b1_telemetry_task(int stacd,void *exinf) {
 unsigned int hb_at=now_ms(), tel_at=hb_at, diag_at=hb_at, seq=0, running=0;
 char text[B1_PAYLOAD_MAX]; incoming_msg m;
 (void)stacd;(void)exinf;
 for(;;) {
  unsigned int now=now_ms();
  /* Bound each iteration so command flooding cannot starve heartbeat. */
  for(unsigned int i=0;i<8 && tk_rcv_mbf(incoming,&m,TMO_POL)==sizeof(m);i++) {
   b1_command c={0}; const char *status;
   now=now_ms();
   if(!atomic_load(&online) || m.epoch!=atomic_load(&epoch) || (unsigned int)(now-m.at)>1000) continue;
   if(!b1_parse(m.data,m.length,&c)) status="rejected";
   else if(B1_DEMO) {
    if(c.kind==B1_START) running=1;
    if(c.kind==B1_STOP) running=0;
    status="simulated";
   } else {
    vehicle_msg v={m.epoch,m.at,c};
    status=tk_snd_mbf(vehicle,&v,sizeof(v),TMO_POL)==E_OK?"queued":"queue_full";
   }
   tm_sprintf((UB*)text,(const UB*)"{\"schema\":1,\"demo\":%s,\"status\":\"%s\",\"command_id\":%u}",B1_DEMO?"true":"false",status,c.command_id);
   put(B1_RESULT,text,now);
  }
  if((unsigned int)(now-hb_at)>=5000) {
   /* Do not advance the timer until the outgoing slot actually accepted it.
    * A nonblocking mutex miss retries next loop instead of losing 5 seconds. */
   tm_sprintf((UB*)text,(const UB*)"{\"schema\":1,\"seq\":%u,\"uptime_ms\":%u,\"demo\":%s}",seq,now,B1_DEMO?"true":"false");
   if(put(B1_HEARTBEAT,text,now)) { hb_at=now; ++seq; }
  }
  if((unsigned int)(now-tel_at)>=200) {
   b1_binary_snapshot s={0}; int valid=0;
   if(B1_DEMO) {
    s.present=B1_SECTION_2;s.s2_at_ms=now;
    s.s2.target_speed_mm_s=running?30:0;s.s2.left_speed_mm_s=running?30:0;s.s2.right_speed_mm_s=running?30:0;
    s.s2.target_speed_valid=1;s.s2.left_speed_valid=1;s.s2.right_speed_valid=1;
    s.s2.motion_active=running;s.s2.system_mode=running?2:5;s.s2.mission_state=running?1:0;
    s.s2.uptime_ms=now;s.s2.sequence=(now/200)&255;valid=1;
   }
   else if(lock()) {s=latest;valid=have_snapshot && (unsigned int)(now_ms()-snapshot_at)<=1500;tk_unl_mtx(guard);}
   if(valid) {
    unsigned int n=b1_encode_telemetry((unsigned char*)text,sizeof(text),&s,seq,now,B1_DEMO);
    if(n && put_bytes(B1_TELEMETRY,text,n,now)){tel_at=now;++seq;}
   }
  }
  if((unsigned int)(now-diag_at)>=1000){
   b1_diagnostics d;int copied=0;
   if(lock()){d=diagnostic;tk_unl_mtx(guard);copied=1;}
   if(copied){d.uptime_ms=now;d.busy=atomic_load(&busy_submissions);
    unsigned int n=b1_encode_diagnostics((unsigned char*)text,sizeof(text),&d,B1_DEMO);
    if(n&&put_bytes(B1_DIAGNOSTICS,text,n,now))diag_at=now;
   }
  }
  tk_dly_tsk(20);
 }
}
