#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H
#include <stdint.h>
typedef int ER; typedef int ID; typedef int INT; typedef unsigned int UW;
typedef unsigned char UB; typedef int PRI; typedef void (*FP)(int,void*);
#define EXPORT
#define IMPORT extern
#define E_OK 0
#define E_OBJ (-41)
#define E_NOSPT (-9)
#define TA_HLNG 1
#define TA_RNG3 2
#define TMO_FEVR (-1)
#define TK_SUPPORT_SMP 0
typedef struct {FP task; PRI itskpri; UW stksz,tskatr;} T_CTSK;
ID tk_cre_tsk(const T_CTSK*); ER tk_sta_tsk(ID,int); ER tk_del_tsk(ID);
ER tk_slp_tsk(int);
#endif
