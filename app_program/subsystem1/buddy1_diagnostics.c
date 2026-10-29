#include "buddy1_diagnostics.h"
#include <string.h>
static void put32(unsigned char *p,uint32_t v){for(unsigned int i=0;i<4;i++)p[i]=(unsigned char)(v>>(8*i));}
static void put64(unsigned char *p,int64_t v){uint64_t u=(uint64_t)v;for(unsigned int i=0;i<8;i++)p[i]=(unsigned char)(u>>(8*i));}
unsigned int b1_encode_diagnostics(unsigned char *out,unsigned int capacity,const b1_diagnostics *d,int demo){
 if(!out||!d||capacity<B1_DIAGNOSTICS_SIZE)return 0;
 memset(out,0,B1_DIAGNOSTICS_SIZE);out[0]=0xb1;out[1]=2;out[2]=2;out[3]=(unsigned char)(demo?1:0);
 put32(out+4,d->uptime_ms);put32(out+8,d->rejected);put32(out+12,d->out_of_range);
 put32(out+16,d->busy);put32(out+20,d->last_at_ms);
 out[24]=(unsigned char)d->last.section;out[25]=(unsigned char)d->last.field;out[26]=(unsigned char)d->last.reason;
 put64(out+28,d->last.value);put64(out+36,d->last.minimum);put64(out+44,d->last.maximum);
 return B1_DIAGNOSTICS_SIZE;
}
