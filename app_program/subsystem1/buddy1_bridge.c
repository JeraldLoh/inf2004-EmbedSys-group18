#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <stdatomic.h>
#include <string.h>
#include "../app.h"
#include "buddy1_bridge.h"
#include "buddy1_diagnostics.h"
#include "../subsystem3/barcode_map.h"
#if defined(_PICO_RP2040_)
#include <tk/smp.h>
#endif
/* All queue objects are created before ready is published. */
static ID guard, incoming, vehicle;
static atomic_int ready, online;
static atomic_uint epoch;
/* One latest packet per topic, under guard. */
static struct {
 uint32_t created_ms[B1_SLOT_COUNT];
 uint8_t length[B1_SLOT_COUNT]; /* 0 = empty; all real packets are nonempty */
 char heartbeat[B1_HEARTBEAT_SIZE], telemetry[B1_BINARY_MAX], result[B1_RESULT_MAP_SIZE];
 char diagnostics[B1_DIAGNOSTICS_SIZE], live[B1_LIVE_MAP_SIZE];
} outbox;
static const unsigned slot_capacity[B1_SLOT_COUNT]={
 sizeof outbox.heartbeat,sizeof outbox.telemetry,sizeof outbox.result,
 sizeof outbox.diagnostics,sizeof outbox.live
};
static char *const slot_payload[B1_SLOT_COUNT]={
 outbox.heartbeat,outbox.telemetry,outbox.result,outbox.diagnostics,outbox.live
};
_Static_assert(B1_PAYLOAD_MAX <= UINT8_MAX,"outbox lengths must fit");
static int have_snapshot;
static b1_diagnostics diagnostic;
static atomic_uint busy_submissions;

/* Single Buddy 3 writer. All concurrent fields are atomic, and the sequence
 * and payload share a sequentially consistent order. Bit 19 is presence,
 * which remains set even when the sequence wraps. */
static struct {
 _Atomic uint32_t seq, flags, at_ms, scan, failures, event_ms, map_revision;
} s3_line;
#define S3_LIVE_PRESENT (1U<<19)

static void s3_line_publish(const b1_live_line *l) {
 /* Sanitize the original-width values with their existing fallbacks. */
 uint32_t flags=S3_LIVE_PRESENT|(l->paused!=0U)
  |((l->calibration_state<=4U?l->calibration_state:0U)<<1)
  |((l->calibration_reason<=1U?l->calibration_reason:0U)<<4)
  |((l->barcode_state<=4U?l->barcode_state:3U)<<5)
  |((l->barcode_character<=127U?l->barcode_character:0U)<<8)
  |((l->barcode_route<=4U?l->barcode_route:0U)<<15)
  |((uint32_t)(l->barcode_digital!=0U)<<18);
 uint32_t q=atomic_load(&s3_line.seq);
 atomic_store(&s3_line.seq,q+1U);
 atomic_store(&s3_line.flags,flags);
 atomic_store(&s3_line.at_ms,l->at_ms);
 atomic_store(&s3_line.scan,l->barcode_sequence);
 atomic_store(&s3_line.failures,l->barcode_failures);
 atomic_store(&s3_line.event_ms,l->barcode_event_ms);
 atomic_store(&s3_line.map_revision,l->map_revision);
 atomic_store(&s3_line.seq,q+2U);
}
static int s3_line_read(b1_live_line *out) {
 for(unsigned t=0;t<4U;t++){
  uint32_t a=atomic_load(&s3_line.seq);
  if(a&1U)continue;
  uint32_t flags=atomic_load(&s3_line.flags);
  out->at_ms=atomic_load(&s3_line.at_ms);
  out->barcode_sequence=atomic_load(&s3_line.scan);
  out->barcode_failures=atomic_load(&s3_line.failures);
  out->barcode_event_ms=atomic_load(&s3_line.event_ms);
  out->map_revision=atomic_load(&s3_line.map_revision);
  if(atomic_load(&s3_line.seq)!=a)continue;
  out->paused=flags&1U;
  out->calibration_state=(flags>>1)&7U;
  out->calibration_reason=(flags>>4)&1U;
  out->barcode_state=(flags>>5)&7U;
  out->barcode_character=(flags>>8)&127U;
  out->barcode_route=(flags>>15)&7U;
  out->barcode_digital=(flags>>18)&1U;
  return (flags&S3_LIVE_PRESENT)!=0U;
 }
 return 0;
}
#if defined(_PICO_RP2040_) && TK_SUPPORT_SMP
static T_SPLOCK busy_lock;
#endif
static void count_busy(void) {
 /* ARMv6-M lacks the atomic fetch-add instruction/libcall. Use the kernel's
  * cross-core interrupt-safe lock, only around this diagnostic increment. */
#if defined(_PICO_RP2040_)
 UINT saved;
#if TK_SUPPORT_SMP
 (void)ISpinLock(&busy_lock,&saved);
#else
 saved=disint();
#endif
 atomic_store(&busy_submissions,atomic_load(&busy_submissions)+1U);
#if TK_SUPPORT_SMP
 (void)ISpinUnlock(&busy_lock,saved);
#else
 set_primask(saved);
#endif
#else
 atomic_fetch_add(&busy_submissions,1);
#endif
}
static b1_binary_cache latest;
/* Buddy 2/5 staging is guarded by the same mutex as before. Only bounded
 * flags are narrowed; the diagnostic counters and signed targets stay wide. */
static struct {
 uint32_t present;
 struct {
  uint32_t at_ms,flags,searches,search_fails,accepted,rejected,driver_errors;
  uint32_t encoder_left_counts,encoder_right_counts,wheel_diameter_mm;
  int32_t left_target,right_target;
 } motion;
 struct {uint32_t at_ms,flags,backoffs,elapsed_ms;} bypass;
} live_latest;
static unsigned int snapshot_at;
typedef struct { unsigned int length, epoch, at; char data[128]; } incoming_msg;
typedef struct { unsigned int epoch, at; b1_command command; } vehicle_msg;
static unsigned int now_ms(void) { SYSTIM t; tk_get_otm(&t); return t.lo; }
/* This comms task is the sole map writer. */
static bool apply_map(const b1_map_command *m,uint8_t held[B1_MAP_BYTES]) {
 if(m->kind==B1_MAP_SET) {
  uint8_t replacement[BARCODE_MAP_SIZE];
  for(unsigned int i=0;i<B1_MAP_ENTRIES;i++)replacement[i]=b1_map_entry(m->packed,i);
  if(!barcode_map_replace(replacement))return false;
 }
 for(unsigned int i=0;i<B1_MAP_ENTRIES;i++)b1_map_put(held,i,(unsigned int)barcode_map_get(b1_map_character(i)));
 return true;
}
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
 b1_validation_error error={.section=1,.reason=1};
 if(!lock()){count_busy();return 0;}
 if(!s||!b1_cache_base(&latest,s->speed_percent,s->distance_cm,&error)){
  ++diagnostic.rejected;if(error.reason==2)++diagnostic.out_of_range;
  diagnostic.last=error;diagnostic.last_at_ms=now_ms();tk_unl_mtx(guard);return 0;}
 have_snapshot=1; snapshot_at=now_ms(); tk_unl_mtx(guard); return 1;
}
int b1_submit_binary_snapshot(const b1_binary_snapshot *s) {
 b1_validation_error error;
 if(!lock()){count_busy();return 0;}
 if(!b1_cache_snapshot(&latest,s,&error)){++diagnostic.rejected;if(error.reason==2)++diagnostic.out_of_range;
  diagnostic.last=error;diagnostic.last_at_ms=now_ms();tk_unl_mtx(guard);return 0;}
 have_snapshot=1;snapshot_at=now_ms();tk_unl_mtx(guard);return 1;
}
int b1_submit_live_status(unsigned id,const void *section) {
 if(!section || (id!=2 && id!=3 && id!=5))return 0;
 if(id==3){s3_line_publish((const b1_live_line*)section);return 1;}
 if(!lock()){count_busy();return 0;}
 int valid;
 if(id==2){
  const b1_live_motion *m=section;
  valid=m->targets_valid<=1U&&m->qualified<=1U&&m->searching<=1U;
  if(valid){
   live_latest.motion.at_ms=m->at_ms;
   live_latest.motion.flags=m->targets_valid|(m->qualified<<1)|(m->searching<<2);
   live_latest.motion.left_target=m->left_target;live_latest.motion.right_target=m->right_target;
   live_latest.motion.searches=m->searches;live_latest.motion.search_fails=m->search_fails;
   live_latest.motion.accepted=m->accepted;live_latest.motion.rejected=m->rejected;
   live_latest.motion.driver_errors=m->driver_errors;
   live_latest.motion.encoder_left_counts=m->encoder_left_counts;
   live_latest.motion.encoder_right_counts=m->encoder_right_counts;
   live_latest.motion.wheel_diameter_mm=m->wheel_diameter_mm;
   live_latest.present|=1U;
  }
 }else{
  const b1_live_bypass *b=section;
  valid=b->fast<=1U&&b->active<=1U&&b->backing_off<=1U&&b->segments<=7U&&
   b->segment<=b->segments&&b->side<=2U;
  if(valid){
   live_latest.bypass.at_ms=b->at_ms;live_latest.bypass.backoffs=b->backoffs;
   live_latest.bypass.elapsed_ms=b->elapsed_ms;
   live_latest.bypass.flags=b->fast|(b->active<<1)|(b->backing_off<<2)|
    (b->segment<<3)|(b->segments<<6)|(b->side<<9);
   live_latest.present|=4U;
  }
 }
 tk_unl_mtx(guard);return valid;
}

static void read_live_locked(b1_live_status *s){
 uint32_t flags=live_latest.motion.flags;
 s->motion=(b1_live_motion){
  .at_ms=live_latest.motion.at_ms,.left_target=live_latest.motion.left_target,
  .right_target=live_latest.motion.right_target,.targets_valid=flags&1U,
  .qualified=(flags>>1)&1U,.searching=(flags>>2)&1U,
  .searches=live_latest.motion.searches,.search_fails=live_latest.motion.search_fails,
  .accepted=live_latest.motion.accepted,.rejected=live_latest.motion.rejected,
  .driver_errors=live_latest.motion.driver_errors,
  .encoder_left_counts=live_latest.motion.encoder_left_counts,
  .encoder_right_counts=live_latest.motion.encoder_right_counts,
  .wheel_diameter_mm=live_latest.motion.wheel_diameter_mm};
 flags=live_latest.bypass.flags;
 s->bypass=(b1_live_bypass){
  .at_ms=live_latest.bypass.at_ms,.fast=flags&1U,.active=(flags>>1)&1U,
  .backing_off=(flags>>2)&1U,.segment=(flags>>3)&7U,.segments=(flags>>6)&7U,
  .side=(flags>>9)&3U,.backoffs=live_latest.bypass.backoffs,
  .elapsed_ms=live_latest.bypass.elapsed_ms};
 s->present=live_latest.present;
}
int b1_is_online(void) { return atomic_load(&online); }
int b1_submit_section(unsigned int id,const void *section,unsigned int at) {
 b1_validation_error error;
 if(!section || (id!=2 && id!=3 && id!=4 && id!=5)) return 0;
 if(!lock()){count_busy();return 0;}
 if(!b1_cache_section(&latest,id,section,at,&error)){
  ++diagnostic.rejected;if(error.reason==2)++diagnostic.out_of_range;
  diagnostic.last=error;diagnostic.last_at_ms=now_ms();tk_unl_mtx(guard);return 0;
 }
 have_snapshot=1;snapshot_at=now_ms();tk_unl_mtx(guard);return 1;
}
int b1_network_take(unsigned int kind,b1_packet *p) {
 int result=0;
 if(kind>=B1_SLOT_COUNT || !p || !lock()) return 0;
 if(outbox.length[kind]) {
  p->kind=kind;p->created_ms=outbox.created_ms[kind];p->length=outbox.length[kind];
  memcpy(p->text,slot_payload[kind],p->length);
  outbox.length[kind]=0;result=1;
 }
 tk_unl_mtx(guard);
 if(result && (unsigned int)(now_ms()-p->created_ms)>(kind==B1_HEARTBEAT?5000u:1500u)) return 0;
 return result;
}
void b1_network_restore(const b1_packet *p) {
 if(!p || p->kind>=B1_SLOT_COUNT || p->length==0 || p->length>slot_capacity[p->kind] || !lock()) return;
 unsigned kind=p->kind;
 if(!outbox.length[kind]) {
  memcpy(slot_payload[kind],p->text,p->length);
  outbox.created_ms[kind]=p->created_ms;outbox.length[kind]=(uint8_t)p->length;
 }
 tk_unl_mtx(guard);
}
static int put_bytes(unsigned int kind,const void *data,unsigned int length,unsigned int at) {
 if(kind>=B1_SLOT_COUNT || !data || length==0 || length>slot_capacity[kind]) return 0;
 if(!lock()) return 0;
 memcpy(slot_payload[kind],data,length);
 outbox.created_ms[kind]=at;outbox.length[kind]=(uint8_t)length;
 tk_unl_mtx(guard); return 1;
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
 unsigned int hb_at=now_ms(), tel_at=hb_at, diag_at=hb_at, live_at=hb_at, seq=0, running=0;
 uint8_t payload[B1_PAYLOAD_MAX]; incoming_msg m;
 (void)stacd;(void)exinf;
 for(;;) {
  unsigned int now=now_ms();
  /* Bound each iteration so command flooding cannot starve heartbeat. */
  for(unsigned int i=0;i<8 && tk_rcv_mbf(incoming,&m,TMO_POL)==sizeof(m);i++) {
   b1_command c={0}; b1_map_command mc; uint8_t map[B1_MAP_BYTES]={0};
   unsigned status=B1_RESULT_REJECTED;
   now=now_ms();
   if(!atomic_load(&online) || m.epoch!=atomic_load(&epoch) || (unsigned int)(now-m.at)>1000) continue;
   if(b1_parse_map(m.data,m.length,&mc)) {
    c.command_id=mc.command_id;
    if(apply_map(&mc,map))status=B1_RESULT_APPLIED;
   }else if(b1_parse(m.data,m.length,&c)){
    if(B1_DEMO){
     running=c.kind==B1_START;
     status=B1_RESULT_SIMULATED;
    }else{
     vehicle_msg v={m.epoch,m.at,c};
     status=tk_snd_mbf(vehicle,&v,sizeof(v),TMO_POL)==E_OK?B1_RESULT_QUEUED:B1_RESULT_QUEUE_FULL;
    }
   }
   bool mapped=status==B1_RESULT_APPLIED;
   unsigned n=b1_encode_result(payload,sizeof(payload),c.command_id,status,B1_DEMO,
       mapped?map:NULL,mapped?barcode_map_revision():0U);
   if(n)put_bytes(B1_RESULT,payload,n,now);
  }
  if((unsigned int)(now-hb_at)>=5000) {
   /* Do not advance the timer until the outgoing slot actually accepted it.
    * A nonblocking mutex miss retries next loop instead of losing 5 seconds. */
   unsigned n=b1_encode_heartbeat(payload,sizeof(payload),seq,now,B1_DEMO);
   if(n && put_bytes(B1_HEARTBEAT,payload,n,now)) { hb_at=now; ++seq; }
  }
  if((unsigned int)(now-tel_at)>=200) {
   unsigned n=0;
   if(B1_DEMO) {
    b1_binary_snapshot s={0};
    s.present=B1_SECTION_2;s.s2_at_ms=now;
    s.s2.target_speed_mm_s=running?30:0;s.s2.left_speed_mm_s=running?30:0;s.s2.right_speed_mm_s=running?30:0;
    s.s2.target_speed_valid=1;s.s2.left_speed_valid=1;s.s2.right_speed_valid=1;
    s.s2.motion_active=running;s.s2.system_mode=running?2:5;s.s2.mission_state=running?1:0;
    s.s2.uptime_ms=now;s.s2.sequence=(now/200)&255;
    n=b1_encode_telemetry(payload,sizeof(payload),&s,seq,now,B1_DEMO);
   }
   else if(lock()) {
    if(have_snapshot && (unsigned int)(now_ms()-snapshot_at)<=1500)
     n=b1_encode_cached(payload,sizeof(payload),&latest,seq,now,B1_DEMO);
    tk_unl_mtx(guard);
   }
   if(n && put_bytes(B1_TELEMETRY,payload,n,now)){tel_at=now;++seq;}
  }
  if((unsigned int)(now-live_at)>=200){
   b1_live_status s={0};int locked=0;
   if(lock()){read_live_locked(&s);tk_unl_mtx(guard);locked=1;}
   b1_live_line line;
   if(locked && s3_line_read(&line)){s.line=line;s.present|=2;}
   if(locked && s.present){s.uptime_ms=now_ms();
    unsigned n=b1_encode_live_status(payload,sizeof(payload),&s,B1_DEMO);
    if(n && put_bytes(B1_LIVE_STATUS,payload,n,now))live_at=now;
   }
  }
  if((unsigned int)(now-diag_at)>=1000){
   b1_diagnostics d;int copied=0;
   if(lock()){d=diagnostic;tk_unl_mtx(guard);copied=1;}
   if(copied){d.uptime_ms=now;d.busy=atomic_load(&busy_submissions);
    unsigned int n=b1_encode_diagnostics(payload,sizeof(payload),&d,B1_DEMO);
    if(n&&put_bytes(B1_DIAGNOSTICS,payload,n,now))diag_at=now;
   }
  }
  tk_dly_tsk(20);
 }
}
