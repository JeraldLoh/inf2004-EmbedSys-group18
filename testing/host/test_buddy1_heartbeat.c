/* Execute the real bridge loop with a fake clock and injected mutex contention.
 * This reproduces a lost producer heartbeat; it is not a hardware timing test. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <tk/tkernel.h>
#include "../../app_program/subsystem1/buddy1_bridge.h"
static unsigned int clock_ms,hold_until,times[4],count;
static int network_reading;
static jmp_buf finished;
ER tk_get_otm(SYSTIM*t){t->lo=clock_ms;t->hi=0;return E_OK;}
ER tk_loc_mtx(ID id,int wait){(void)id;(void)wait;return !network_reading&&clock_ms>=5000&&clock_ms<hold_until?-50:E_OK;}
ER tk_unl_mtx(ID id){(void)id;return E_OK;}
ID tk_cre_mtx(const T_CMTX*c){(void)c;return 1;}
ID tk_cre_mbf(const T_CMBF*c){(void)c;return 2;}
ER tk_del_mtx(ID i){(void)i;return E_OK;}
ER tk_del_mbf(ID i){(void)i;return E_OK;}
ER tk_snd_mbf(ID i,const void*p,unsigned int n,int t){(void)i;(void)p;(void)n;(void)t;return E_OK;}
INT tk_rcv_mbf(ID i,void*p,int t){(void)i;(void)p;(void)t;return -50;}
int tm_sprintf(UB*out,const UB*fmt,...){va_list ap;va_start(ap,fmt);int n=vsnprintf((char*)out,B1_PAYLOAD_MAX,(const char*)fmt,ap);va_end(ap);return n;}
ER tk_dly_tsk(unsigned int delay){
 b1_packet p;network_reading=1;
 if(b1_network_take(B1_HEARTBEAT,&p)){
  assert(count<4);times[count++]=p.created_ms;
 }
 network_reading=0;clock_ms+=delay;
 if(clock_ms>10500)longjmp(finished,1);
 return E_OK;
}
int main(int argc,char**argv){
 hold_until=argc>1?(unsigned int)strtoul(argv[1],0,10):0;
 assert(b1_init()==E_OK);
 if(setjmp(finished)==0)b1_telemetry_task(0,0);
 assert(count==2); /* A busy slot must not silently skip a whole interval. */
 assert(times[0]>=5000 && times[0]<=((hold_until>5000?hold_until:5000)+20));
 assert(times[1]-times[0]>=5000 && times[1]-times[0]<=5020);
 printf("PASS heartbeat retry: hold_until=%u first=%u second=%u\n",hold_until,times[0],times[1]);
 return 0;
}
