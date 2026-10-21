#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>
#include "../app.h"
#include "buddy1_bridge.h"
/* All queue objects are created before ready is published. */
static ID guard, incoming, vehicle;
static atomic_int ready, online;
static atomic_uint epoch;
static b1_packet slots[3];
static int occupied[3], have_snapshot;
static b1_snapshot latest;
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
 if(!s || s->speed_percent>100 || s->distance_cm>100000 || !lock()) return 0;
 latest=*s; have_snapshot=1; snapshot_at=now_ms(); tk_unl_mtx(guard); return 1;
}
int b1_network_take(unsigned int kind,b1_packet *p) {
 int result=0;
 if(kind>2 || !p || !lock()) return 0;
 if(occupied[kind]) { *p=slots[kind]; occupied[kind]=0; result=1; }
 tk_unl_mtx(guard);
 if(result && (unsigned int)(now_ms()-p->created_ms)>(kind==B1_HEARTBEAT?5000u:1500u)) return 0;
 return result;
}
void b1_network_restore(const b1_packet *p) {
 if(p->kind>2 || !lock()) return;
 if(!occupied[p->kind]) { slots[p->kind]=*p;occupied[p->kind]=1; }
 tk_unl_mtx(guard);
}
static void put(unsigned int kind,const char *text,unsigned int at) {
 if(!lock()) return;
 slots[kind].kind=kind; slots[kind].created_ms=at;
 strncpy(slots[kind].text,text,B1_PAYLOAD_MAX-1); slots[kind].text[B1_PAYLOAD_MAX-1]=0;
 occupied[kind]=1; tk_unl_mtx(guard);
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
 unsigned int hb_at=now_ms(), tel_at=hb_at, seq=0, speed=0, running=0;
 char text[B1_PAYLOAD_MAX]; incoming_msg m;
 (void)stacd;(void)exinf;
 for(;;) {
  unsigned int now=now_ms();
  /* Bound each iteration so command flooding cannot starve heartbeat. */
  for(unsigned int i=0;i<8 && tk_rcv_mbf(incoming,&m,TMO_POL)==sizeof(m);i++) {
   b1_command c; const char *status;
   now=now_ms();
   if(!atomic_load(&online) || m.epoch!=atomic_load(&epoch) || (unsigned int)(now-m.at)>1000) continue;
   if(!b1_parse(m.data,m.length,&c)) status="rejected";
   else if(B1_DEMO) {
    if(c.kind==B1_START) running=1;
    if(c.kind==B1_STOP) running=0;
    if(c.kind==B1_SPEED) speed=c.speed;
    status="simulated";
   } else {
    vehicle_msg v={m.epoch,m.at,c};
    status=tk_snd_mbf(vehicle,&v,sizeof(v),TMO_POL)==E_OK?"queued":"queue_full";
   }
   tm_sprintf((UB*)text,(const UB*)"{\"schema\":1,\"demo\":%s,\"status\":\"%s\"}",B1_DEMO?"true":"false",status);
   put(B1_RESULT,text,now);
  }
  if((unsigned int)(now-hb_at)>=5000) {
   hb_at=now;
   tm_sprintf((UB*)text,(const UB*)"{\"schema\":1,\"seq\":%u,\"uptime_ms\":%u,\"demo\":%s}",seq++,now,B1_DEMO?"true":"false");
   put(B1_HEARTBEAT,text,now);
  }
  if((unsigned int)(now-tel_at)>=1000) {
   b1_snapshot s; int valid=0; tel_at=now;
   if(B1_DEMO) {s.speed_percent=running?speed:0;s.distance_cm=80+(now/1000)%20;valid=1;}
   else if(lock()) {s=latest;valid=have_snapshot && (unsigned int)(now_ms()-snapshot_at)<=1500;tk_unl_mtx(guard);}
   if(valid) {
    tm_sprintf((UB*)text,(const UB*)"{\"schema\":1,\"seq\":%u,\"uptime_ms\":%u,\"demo\":%s,\"speed_percent\":%u,\"distance_cm\":%u}",seq++,now,B1_DEMO?"true":"false",s.speed_percent,s.distance_cm);
    put(B1_TELEMETRY,text,now);
   }
  }
  tk_dly_tsk(20);
 }
}
