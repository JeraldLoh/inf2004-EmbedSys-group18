#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include "../../app_program/subsystem1/subsystem1.h"
#include "../../app_program/subsystem1/buddy1_bridge.h"
static int init_result,init_count,create_count,start_count,delete_count;
static ID create_result=7;
static ER start_result;
static jmp_buf asleep;
int b1_init(void){++init_count;return init_result;}
void b1_telemetry_task(int a,void*b){(void)a;(void)b;}
ID tk_cre_tsk(const T_CTSK*c){++create_count;assert(c->itskpri==18);assert(c->stksz>=4096);return create_result;}
ER tk_sta_tsk(ID i,int a){(void)a;assert(i==7);++start_count;return start_result;}
ER tk_del_tsk(ID i){assert(i==7);++delete_count;return E_OK;}
ER tk_slp_tsk(int t){assert(t==TMO_FEVR);longjmp(asleep,1);return 0;}
int tm_printf(UB*s,...){(void)s;return 0;}
extern INT usermain(void);
int main(void){
#if B1_STANDALONE_DEMO
 init_result=-10;
 if(setjmp(asleep)==0)usermain();
 assert(init_count==1 && create_count==0 && start_count==0);
 assert(subsystem1_start()==E_OBJ);
 init_result=-10;assert(subsystem1_init()==-10);assert(subsystem1_start()==E_OBJ);
 init_result=E_OK;assert(subsystem1_init()==E_OK);
 create_result=-11;assert(subsystem1_start()==-11);
 create_result=7;start_result=-12;assert(subsystem1_start()==-12);assert(delete_count==1);
 start_result=E_OK;assert(subsystem1_start()==E_OK);
 int n=create_count;assert(subsystem1_start()==E_OBJ);assert(create_count==n);
#else
 if(setjmp(asleep)==0)usermain();
 assert(init_count==0&&create_count==0&&start_count==0);
#endif
 return 0;
}
