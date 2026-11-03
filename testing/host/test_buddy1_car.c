#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <tk/tkernel.h>
#include "../../app_program/subsystem1/buddy1_bridge.h"
#include "../../app_program/subsystem1/buddy1_car_policy.h"
static unsigned clock_ms;
static jmp_buf done;
static b1_binary_snapshot expected;
static unsigned packets, live_packets;
static b1_live_status live_expected;
static FILE *fixture,*live_fixture;
ER tk_get_otm(SYSTIM *t){t->lo=clock_ms;t->hi=0;return E_OK;}
ER tk_loc_mtx(ID id,int wait){(void)id;(void)wait;return E_OK;}
ER tk_unl_mtx(ID id){(void)id;return E_OK;}
ID tk_cre_mtx(const T_CMTX *c){(void)c;return 1;}
ID tk_cre_mbf(const T_CMBF *c){(void)c;return 2;}
ER tk_del_mtx(ID i){(void)i;return E_OK;}
ER tk_del_mbf(ID i){(void)i;return E_OK;}
ER tk_snd_mbf(ID i,const void *p,unsigned n,int t){(void)i;(void)p;(void)n;(void)t;return E_OK;}
INT tk_rcv_mbf(ID i,void *p,int t){(void)i;(void)p;(void)t;return -50;}
int tm_sprintf(UB *out,const UB *fmt,...){va_list a;va_start(a,fmt);int n=vsnprintf((char*)out,B1_PAYLOAD_MAX,(const char*)fmt,a);va_end(a);return n;}
ER tk_dly_tsk(unsigned delay){
 b1_packet p;unsigned char bytes[B1_BINARY_MAX];
 if(b1_network_take(B1_TELEMETRY,&p)){
  unsigned n=b1_encode_telemetry(bytes,sizeof(bytes),&expected,0,200,0);
  assert(n==p.length);assert(memcmp(bytes,p.text,n)==0);++packets;
  if(fixture){for(unsigned i=0;i<n;++i)fprintf(fixture,"%02x",(unsigned char)p.text[i]);fputc(10,fixture);}
 }
 if(b1_network_take(B1_LIVE_STATUS,&p)){
  live_expected.uptime_ms=200;
  unsigned n=b1_encode_live_status(bytes,sizeof(bytes),&live_expected,0);
  assert(n==p.length && memcmp(bytes,p.text,n)==0);++live_packets;
  if(live_fixture){for(unsigned i=0;i<n;++i)fprintf(live_fixture,"%02x",(unsigned char)p.text[i]);fputc(10,live_fixture);}
 }
 clock_ms+=delay;if(clock_ms>220)longjmp(done,1);return E_OK;
}
int main(int argc,char **argv){
 if(argc>2){live_fixture=fopen(argv[2],"w");assert(live_fixture);}
 if(argc>1){fixture=fopen(argv[1],"w");assert(fixture);}
 assert(!b1_car_start_allowed(STOP_REASON_PHYSICAL,0,0,0));
 assert(!b1_car_start_allowed(STOP_REASON_MOTION_FAULT,0,0,0));
 assert(!b1_car_start_allowed(0,1,0,0));
 assert(!b1_car_start_allowed(0,0,1,0));
 assert(!b1_car_start_allowed(0,0,0,1));
 assert(b1_car_start_allowed(STOP_REASON_REMOTE,0,0,0));
 assert(b1_car_start_allowed(0,0,0,0));
 assert(b1_car_command_choice(B1_START,B1_STOP)==B1_STOP);
 assert(b1_car_command_choice(B1_STOP,B1_START)==B1_STOP);
 assert(b1_car_start_progress(SYSTEM_MODE_STARTUP,0,100,105)==0);
 assert(b1_car_start_progress(SYSTEM_MODE_IDLE,0,100,105)==0);
 assert(b1_car_start_progress(SYSTEM_MODE_AUTO,0,100,110)==1);
 assert(b1_car_start_progress(SYSTEM_MODE_IDLE,0,100,1100)==-1);
 assert(b1_car_start_progress(SYSTEM_MODE_STOPPED,STOP_REASON_PHYSICAL,100,105)==-1);
 assert(b1_car_start_progress(SYSTEM_MODE_IDLE,0,0xfffffff0U,0x10U)==0);
 assert(b1_init()==E_OK);
 expected.present=B1_SECTION_2|B1_SECTION_3|B1_SECTION_5;
 expected.s2_at_ms=11;expected.s2.left_speed_mm_s=-12;expected.s2.left_speed_valid=1;
 expected.s3_at_ms=22;expected.s3.steering=-100;
 expected.s5_at_ms=33;expected.s5.front_status=1;expected.s5.last_status=4;
 assert(b1_submit_section(2,&expected.s2,11));
 assert(b1_submit_section(3,&expected.s3,22));
 assert(b1_submit_section(5,&expected.s5,33));
 b1_s3_telemetry bad=expected.s3;bad.steering=-101;
 assert(!b1_submit_section(3,&bad,99));
 assert(!b1_submit_section(4,&bad,99));
 assert(!b1_submit_section(3,NULL,99));
 live_expected.present=7;
 live_expected.motion=(b1_live_motion){.at_ms=11,.left_target=-40,.right_target=80,.targets_valid=1,.searching=1};
 live_expected.line=(b1_live_line){.at_ms=22,.paused=1};
 live_expected.bypass=(b1_live_bypass){.at_ms=33,.active=1,.fast=1,.segment=2,.segments=5};
 assert(b1_submit_live_status(2,&live_expected.motion));assert(b1_submit_live_status(3,&live_expected.line));assert(b1_submit_live_status(5,&live_expected.bypass));
 b1_live_line invalid={.paused=2};assert(!b1_submit_live_status(3,&invalid));assert(!b1_submit_live_status(4,&invalid));assert(!b1_submit_live_status(3,NULL));
 if(setjmp(done)==0)b1_telemetry_task(0,0);
 if(fixture)assert(fclose(fixture)==0);
 if(live_fixture)assert(fclose(live_fixture)==0);
 assert(packets==1 && live_packets==1);puts("PASS merged owner snapshots, preserved timestamps, invalid rejection, STOP precedence and START safety gating");
 return 0;
}
