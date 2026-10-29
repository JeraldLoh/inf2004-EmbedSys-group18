from pathlib import Path
import json
root=Path(__file__).resolve().parents[1]
def write(p,s):
 p=root/p;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s,encoding='utf-8',newline='\n')
schema=json.loads((root/'docs/protocol/telemetry-fields.json').read_text())
header='''/* Proposed binary telemetry v2. Wire encoding is explicit; never memcpy a C struct. */
#ifndef BUDDY1_BINARY_H
#define BUDDY1_BINARY_H
#include <stdint.h>
#define B1_BINARY_MAX 512
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
#endif
'''
write('app_program/subsystem1/buddy1_binary.h',header)
c='''#include "buddy1_binary.h"
#include <string.h>
/* Fixed widths and checked bounds: zero bytes and endianness are unambiguous. */
static void put32(unsigned char *p,uint32_t v){for(unsigned int i=0;i<4;i++)p[i]=(unsigned char)(v>>(8*i));}
static void bits(unsigned char *p,unsigned int *at,uint32_t value,unsigned int n){
 for(unsigned int i=0;i<n;i++,(*at)++)if((value>>i)&1u)p[*at/8]|=(unsigned char)(1u<<(*at%8));
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
sizes={k:(sum(f[1] for f in v)+7)//8 for k,v in schema.items()}
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
for key,size,flag in [('2',sizes['2'],8),('3',sizes['3'],1),('4',17,2),('5',sizes['5'],4)]:c+=f' if(s->present&{flag})n+={size+6};\n'
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
for key,size,flag in [('2',sizes['2'],8),('3',sizes['3'],1),('4',17,2),('5',sizes['5'],4)]:
 c+=f' if(s->present&{flag}){{out[at]={key};out[at+1]={size};put32(out+at+2,s->s{key}_at_ms);'
 c+=('memcpy(out+at+6,s->s4,17);' if key=='4' else f'pack{key}(out+at+6,&s->s{key});')+f'at+={size+6};}}\n'
c+=' return n;\n}\n'
write('app_program/subsystem1/buddy1_binary.c',c)
write('dashboard/web/binary-fields.mjs','// Generated from docs/protocol/telemetry-fields.json; widths are provisional.\nexport const fields='+json.dumps(schema,separators=(',',':'))+';\n')
print('Section sizes',sizes,'maximum telemetry',19+sizes['2']+sizes['3']+sizes['5']+17+24)
