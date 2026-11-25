from pathlib import Path
import json
root=Path(__file__).resolve().parents[1]
def write(p,s):
 p=root/p;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s,encoding='utf-8',newline='\n')
schema=json.loads((root/'docs/protocol/telemetry-fields.json').read_text())
sizes={key:(sum(field[1] for field in fields)+7)//8 for key,fields in schema.items()}
sections=[('2',sizes['2'],8),('3',sizes['3'],1),('4',17,2),('5',sizes['5'],4)]
frame_terms=[19]+[size+6 for _,size,_ in sections]
header='''/* Proposed binary telemetry v2. Wire encoding is explicit; never memcpy a C struct. */
#ifndef BUDDY1_BINARY_H
#define BUDDY1_BINARY_H
#include <stdint.h>
'''+'#define B1_BINARY_MAX ('+' + '.join(f'{size}U' for size in frame_terms)+''') /* full v3 telemetry frame */
#define B1_SECTION_2 8u
#define B1_SECTION_3 1u
#define B1_SECTION_4 2u
#define B1_SECTION_5 4u
'''
for key,fields in schema.items():
 header+='typedef struct {\n'+''.join(f' {"int32_t" if lo<0 else "uint32_t"} {name};\n' for name,bits,lo,hi,unit in fields)+f'}} b1_s{key}_telemetry;\n'
header+='''typedef struct {
 uint32_t base_valid, speed_percent, distance_cm, fault_mask, present;
 uint32_t s2_at_ms, s3_at_ms, s4_at_ms, s5_at_ms;
 b1_s2_telemetry s2;
 b1_s3_telemetry s3;
 unsigned char s4[17]; /* original Buddy 4 frame, including 0x21 header */
 b1_s5_telemetry s5;
} b1_binary_snapshot;
typedef struct {
 uint32_t section, field, reason; /* 1-based field index; reason 1 missing, 2 range, 3 format */
 int64_t value, minimum, maximum;
} b1_validation_error;
int b1_binary_validate(const b1_binary_snapshot *s,b1_validation_error *error);
int b1_binary_valid(const b1_binary_snapshot *s);
unsigned int b1_encode_telemetry(unsigned char *out, unsigned int capacity,
 const b1_binary_snapshot *s, uint32_t seq, uint32_t uptime_ms, int demo);
/* Fixed section positions in RAM, compacted on publication. */
typedef struct { uint8_t bytes[B1_BINARY_MAX]; } b1_binary_cache;
int b1_cache_base(b1_binary_cache *cache,uint32_t speed,uint32_t distance,b1_validation_error *error);
int b1_cache_snapshot(b1_binary_cache *cache,const b1_binary_snapshot *s,b1_validation_error *error);
int b1_cache_section(b1_binary_cache *cache,unsigned id,const void *section,uint32_t at,
 b1_validation_error *error);
unsigned b1_encode_cached(uint8_t *out,unsigned capacity,const b1_binary_cache *cache,
 uint32_t seq,uint32_t uptime_ms,int demo);
#endif
'''
write('app_program/subsystem1/buddy1_binary.h',header)
c='''#include "buddy1_binary.h"
#include <string.h>
/* Fixed widths and checked bounds: zero bytes and endianness are unambiguous. */
static void put32(unsigned char *p,uint32_t v){for(unsigned int i=0;i<4;i++)p[i]=(unsigned char)(v>>(8*i));}
static void bits(unsigned char *p,unsigned int *at,uint32_t value,unsigned int n){
 while(n){
  unsigned shift=*at%8U, count=8U-shift;
  if(count>n)count=n;
  p[*at/8U]|=(uint8_t)((value&((1U<<count)-1U))<<shift);
  value>>=count;*at+=count;n-=count;
 }
}
static int reject(b1_validation_error *e,uint32_t section,uint32_t field,uint32_t reason,
 int64_t value,int64_t minimum,int64_t maximum){
 if(e){e->section=section;e->field=field;e->reason=reason;e->value=value;e->minimum=minimum;e->maximum=maximum;}
 return 0;
}
'''
for key,fields in schema.items():
 c+=f'static int valid{key}(const b1_s{key}_telemetry *s,b1_validation_error *e){{\n'
 for index,(name,b,lo,hi,u) in enumerate(fields,1):
  if lo==-2147483648 and hi==2147483647:continue # int32_t already guarantees this full representation range
  condition=f's->{name} < ({lo}) || s->{name} > {hi}' if lo<0 else f's->{name} > {hi}u'
  if hi<4294967295 or lo<0:c+=f' if({condition})return reject(e,{key},{index},2,s->{name},{lo},{hi});\n'
 c+=' return 1;\n}\n'
 c+=f'static void pack{key}(unsigned char *p,const b1_s{key}_telemetry *s){{\n unsigned int at=0;\n'
 c+=''.join(f' bits(p,&at,(uint32_t)s->{n},{b});\n' for n,b,*_ in fields)+'}\n'
c+='''int b1_binary_validate(const b1_binary_snapshot *s,b1_validation_error *e){
 if(e)memset(e,0,sizeof(*e));
 if(!s)return reject(e,1,0,1,0,0,0);
 if(s->base_valid>1)return reject(e,1,1,2,s->base_valid,0,1);
 if(s->speed_percent>100)return reject(e,1,2,2,s->speed_percent,0,100);
 if(s->distance_cm>100000)return reject(e,1,3,2,s->distance_cm,0,100000);
 if(s->fault_mask>1048575)return reject(e,1,4,2,s->fault_mask,0,1048575);
 if(s->present>15)return reject(e,1,5,2,s->present,0,15);
 if((s->present&B1_SECTION_2) && !valid2(&s->s2,e))return 0;
 if((s->present&B1_SECTION_3) && !valid3(&s->s3,e))return 0;
 if((s->present&B1_SECTION_4) && s->s4[0]!=0x21)return reject(e,4,1,3,s->s4[0],0x21,0x21);
 if((s->present&B1_SECTION_5) && !valid5(&s->s5,e))return 0;
 return 1;
}
int b1_binary_valid(const b1_binary_snapshot *s){return b1_binary_validate(s,0);}
unsigned int b1_encode_telemetry(unsigned char *out,unsigned int capacity,
 const b1_binary_snapshot *s,uint32_t seq,uint32_t uptime_ms,int demo){
 if(!out || !b1_binary_valid(s))return 0;
 unsigned int n=19;
'''
for key,size,flag in sections:c+=f' if(s->present&{flag})n+={size+6};\n'
c+=''' if(n>capacity)return 0;
 memset(out,0,n);out[0]=0xb1;out[1]=3;out[2]=1;
 out[3]=(unsigned char)((demo?1:0)|(s->base_valid<<1)|(s->present<<2));
 put32(out+4,seq);put32(out+8,uptime_ms);
 out[12]=(unsigned char)s->fault_mask;out[13]=(unsigned char)(s->fault_mask>>8);
 out[14]=(unsigned char)(s->fault_mask>>16);
 out[15]=(unsigned char)s->speed_percent;
 for(unsigned int i=0;i<3;i++)out[16+i]=(unsigned char)(s->distance_cm>>(8*i));
 unsigned int at=19;
'''
for key,size,flag in sections:
 c+=f' if(s->present&{flag}){{out[at]={key};out[at+1]={size};put32(out+at+2,s->s{key}_at_ms);'
 c+=('memcpy(out+at+6,s->s4,17);' if key=='4' else f'pack{key}(out+at+6,&s->s{key});')+f'at+={size+6};}}\n'
c+=' return n;\n}\n'
cache_at=19
c+='\nstatic const uint8_t section_at[6]={0,0'
for key,size,flag in sections:
 c+=f',{cache_at}'
 cache_at+=size+6
c+='};\nstatic const uint8_t section_size[6]={0,0'+''.join(f',{size}' for _,size,_ in sections)+'};\n'
c+='static const uint8_t section_flag[6]={0,0'+''.join(f',{flag}' for _,_,flag in sections)+'};\n'
c+='''
static void copy_cached_section(b1_binary_cache *cache,unsigned id,const void *section,uint32_t at){
 unsigned char *p=cache->bytes+section_at[id];
 p[0]=(uint8_t)id;p[1]=section_size[id];put32(p+2,at);
 memset(p+6,0,p[1]);
 switch(id){
  case 2: pack2(p+6,section); break;
  case 3: pack3(p+6,section); break;
  case 4: memcpy(p+6,section,17); break;
  case 5: pack5(p+6,section); break;
 }
 cache->bytes[3]|=(uint8_t)(section_flag[id]<<2);
}

static void cache_faults(b1_binary_cache *cache,uint32_t faults){
 for(unsigned i=0;i<3;i++)cache->bytes[12+i]=(uint8_t)(faults>>(8*i));
}

int b1_cache_base(b1_binary_cache *cache,uint32_t speed,uint32_t distance,b1_validation_error *e){
 if(speed>100U)return reject(e,1,2,2,speed,0,100);
 if(distance>100000U)return reject(e,1,3,2,distance,0,100000);
 cache->bytes[3]|=2U;cache->bytes[15]=(uint8_t)speed;
 for(unsigned i=0;i<3;i++)cache->bytes[16+i]=(uint8_t)(distance>>(8*i));
 return 1;
}

int b1_cache_snapshot(b1_binary_cache *cache,const b1_binary_snapshot *s,b1_validation_error *e){
 if(!b1_binary_validate(s,e))return 0;
 memset(cache->bytes,0,19);
 (void)b1_cache_base(cache,s->speed_percent,s->distance_cm,0);
 if(!s->base_valid)cache->bytes[3]&=(uint8_t)~2U;
 cache_faults(cache,s->fault_mask);
 if(s->present&B1_SECTION_2)copy_cached_section(cache,2,&s->s2,s->s2_at_ms);
 if(s->present&B1_SECTION_3)copy_cached_section(cache,3,&s->s3,s->s3_at_ms);
 if(s->present&B1_SECTION_4)copy_cached_section(cache,4,s->s4,s->s4_at_ms);
 if(s->present&B1_SECTION_5)copy_cached_section(cache,5,&s->s5,s->s5_at_ms);
 return 1;
}

int b1_cache_section(b1_binary_cache *cache,unsigned id,const void *section,uint32_t at,
 b1_validation_error *e){
 if(!section)return reject(e,1,0,1,0,0,0);
 switch(id){
  case 2: {
   const b1_s2_telemetry *s=section;
   if(s->hardware_faults>1048575U)return reject(e,1,4,2,s->hardware_faults,0,1048575);
   if(!valid2(s,e))return 0;
   cache_faults(cache,s->hardware_faults);
   break;
  }
  case 3: if(!valid3(section,e))return 0; break;
  case 4: {
   const uint8_t *p=section;
   if(p[0]!=0x21)return reject(e,4,1,3,p[0],0x21,0x21);
   break;
  }
  case 5: if(!valid5(section,e))return 0; break;
  default: return 0;
 }
 copy_cached_section(cache,id,section,at);
 return 1;
}

unsigned b1_encode_cached(uint8_t *out,unsigned capacity,const b1_binary_cache *cache,
 uint32_t seq,uint32_t uptime_ms,int demo){
 unsigned n=19;
 for(unsigned id=2;id<=5;id++)
  if(cache->bytes[3]&(section_flag[id]<<2))n+=6U+section_size[id];
 if(!out||capacity<n)return 0;
 memcpy(out,cache->bytes,19);
 out[0]=0xb1;out[1]=3;out[2]=1;out[3]|=(uint8_t)(demo?1:0);
 put32(out+4,seq);put32(out+8,uptime_ms);
 unsigned at=19;
 for(unsigned id=2;id<=5;id++){
  if(!(cache->bytes[3]&(section_flag[id]<<2)))continue;
  unsigned size=6U+section_size[id];
  memcpy(out+at,cache->bytes+section_at[id],size);at+=size;
 }
 return n;
}
'''
write('app_program/subsystem1/buddy1_binary.c',c)
write('dashboard/web/binary-fields.mjs','// Generated from docs/protocol/telemetry-fields.json; widths are provisional.\nexport const fields='+json.dumps(schema,separators=(',',':'))+';\n')
print('Section sizes',sizes,'maximum telemetry',sum(frame_terms))
