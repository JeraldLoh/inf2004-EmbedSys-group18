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
#define TA_TFIFO 0
#define TA_INHERIT 1
#define TMO_POL 0
typedef struct { unsigned int lo,hi; } SYSTIM;
typedef struct { unsigned int mtxatr; } T_CMTX;
typedef struct { unsigned int mbfatr,maxmsz,bufsz; } T_CMBF;
ER tk_get_otm(SYSTIM*); ER tk_loc_mtx(ID,int); ER tk_unl_mtx(ID);
ID tk_cre_mtx(const T_CMTX*); ID tk_cre_mbf(const T_CMBF*);
ER tk_del_mtx(ID); ER tk_del_mbf(ID);
ER tk_snd_mbf(ID,const void*,unsigned int,int); INT tk_rcv_mbf(ID,void*,int);
ER tk_dly_tsk(unsigned int);
#endif
