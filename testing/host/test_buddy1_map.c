/* Buddy 1 + Buddy 3: a binary MAP command from the dashboard reaches the
 * barcode map through the real bridge loop, and the command_result reply
 * reports what the car now holds. Kernel calls are mocked. */
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../../app_program/subsystem1/buddy1_bridge.h"
#include "../../app_program/subsystem3/barcode_map.h"

static unsigned clock_ms;
static jmp_buf done;
static unsigned char box[2][512];
static unsigned box_n[2], box_full[2];
static char reply[B1_PAYLOAD_MAX];

ER tk_get_otm(SYSTIM *t){t->lo=clock_ms;t->hi=0;return E_OK;}
ER tk_loc_mtx(ID id,int wait){(void)id;(void)wait;return E_OK;}
ER tk_unl_mtx(ID id){(void)id;return E_OK;}
ID tk_cre_mtx(const T_CMTX *c){(void)c;return 1;}
static ID next_mbf=0;
ID tk_cre_mbf(const T_CMBF *c){(void)c;return next_mbf++;}   /* 0 incoming, 1 vehicle */
ER tk_del_mtx(ID i){(void)i;return E_OK;}
ER tk_del_mbf(ID i){(void)i;return E_OK;}
ER tk_snd_mbf(ID i,const void *p,unsigned n,int t){(void)t;if(box_full[i])return -1;memcpy(box[i],p,n);box_n[i]=n;box_full[i]=1;return E_OK;}
INT tk_rcv_mbf(ID i,void *p,int t){(void)t;if(!box_full[i])return -50;memcpy(p,box[i],box_n[i]);box_full[i]=0;return (INT)box_n[i];}
int tm_sprintf(UB *out,const UB *fmt,...){va_list a;va_start(a,fmt);int n=vsnprintf((char*)out,B1_PAYLOAD_MAX,(const char*)fmt,a);va_end(a);return n;}
ER tk_dly_tsk(unsigned delay){
 b1_packet p;
 if(b1_network_take(B1_RESULT,&p)){memcpy(reply,p.text,p.length);reply[p.length]=0;}
 clock_ms+=delay;longjmp(done,1);
}
static void run(const unsigned char *bytes,unsigned n){
 reply[0]=0;
 assert(b1_network_receive((const char*)bytes,n));
 if(setjmp(done)==0)b1_telemetry_task(0,0);
}
int main(void){
 assert(b1_init()==E_OK);b1_connection(1);
 barcode_map_defaults();

 /* SHOW: the power-on map A=L B=R C=S D=U is reported, nothing changes.
  * A..D = 1,2,3,4 in 3-bit slots: 0b100_011_010_001 = 0x8d1 -> bytes d1 08. */
 unsigned char show[8]={0xb1,2,3,4,5,0,0,0};
 run(show,8);
 assert(strstr(reply,"\"status\":\"applied\""));
 assert(strstr(reply,"\"command_id\":5"));
 assert(strstr(reply,"\"map\":\"d1080000000000000000000000000000\"")==NULL);  /* 28 hex, not 32 */
 assert(strstr(reply,"\"map\":\"d108000000000000000000000000\""));
 assert(barcode_map_get('A')==ROUTE_COMMAND_LEFT);

 /* SET replaces the whole table: K=L, Q=R, Z=S, 7=U, A..D cleared. */
 unsigned char set[22]={0xb1,2,3,3,6,0,0,0};
 b1_map_put(set+8,'K'-'A',1);b1_map_put(set+8,'Q'-'A',2);b1_map_put(set+8,'Z'-'A',3);b1_map_put(set+8,26+7,4);
 run(set,22);
 assert(strstr(reply,"\"status\":\"applied\"")&&strstr(reply,"\"command_id\":6"));
 assert(barcode_map_get('K')==ROUTE_COMMAND_LEFT&&barcode_map_get('q')==ROUTE_COMMAND_RIGHT);
 assert(barcode_map_get('Z')==ROUTE_COMMAND_STRAIGHT&&barcode_map_get('7')==ROUTE_COMMAND_UTURN);
 assert(barcode_map_get('A')==ROUTE_COMMAND_NONE&&barcode_map_get('D')==ROUTE_COMMAND_NONE);

 /* A bad entry (value 6) is rejected and the table is untouched. */
 b1_map_put(set+8,0,6);set[4]=7;
 run(set,22);
 assert(strstr(reply,"\"status\":\"rejected\"")&&!strstr(reply,"\"map\""));
 assert(barcode_map_get('A')==ROUTE_COMMAND_NONE&&barcode_map_get('K')==ROUTE_COMMAND_LEFT);

 /* START still works and carries no map field. */
 unsigned char start[8]={0xb1,2,3,1,8,0,0,0};
 run(start,8);
 assert(!strstr(reply,"\"map\"")&&strstr(reply,"\"command_id\":8"));
 puts("PASS binary MAP set/show through the bridge, whole-table replace, bad entry rejected");
 return 0;
}
