#ifndef B2_TEST_TKERNEL_H
#define B2_TEST_TKERNEL_H
/* Compile-only lifecycle type shim. No fake scheduler/IRQ safety is claimed. */
#include <stdint.h>
#include <stddef.h>
typedef int32_t ER;
typedef int INT;
typedef int ID;
typedef unsigned char UB;
typedef unsigned int UINT;
typedef uint32_t UW;
typedef int32_t SZ;
typedef int BOOL;
#define EXPORT
#define IMPORT extern
#define E_OK 0
#define E_SYS (-5)
#define E_IO (-57)
#define E_NOSPT (-9)
#define E_PAR (-17)
#define E_OBJ (-41)
#define TRUE 1
#define FALSE 0
#define TK_SUPPORT_SMP 1
#define TA_HLNG 1U
#define TA_RNG0 0U
#define TA_USERBUF 0x20U
#define TA_ASSPRC 0x10000U
#define TP_PRC1 1U
#define TA_INHERIT 2U
#define TA_TFIFO 0U
#define TMO_POL 0
#define TMO_FEVR (-1)
typedef struct { UINT mtxatr; } T_CMTX;
typedef struct { UINT mbfatr; INT maxmsz; SZ bufsz; } T_CMBF;
typedef struct { UW hi, lo; } SYSTIM;
ID tk_cre_mtx(const T_CMTX *config);
ER tk_del_mtx(ID id);
ER tk_loc_mtx(ID id, INT timeout);
ER tk_unl_mtx(ID id);
ID tk_cre_mbf(const T_CMBF *config);
ER tk_del_mbf(ID id);
ER tk_snd_mbf(ID id, const void *message, INT size, INT timeout);
INT tk_rcv_mbf(ID id, void *message, INT timeout);
ER tk_get_otm(SYSTIM *time);
typedef void (*FP)(); /* Matches the course's generic C11 function pointer. */
typedef struct { UINT intatr; FP inthdr; } T_DINT;
ER tk_def_int(UINT intno, const T_DINT *handler);
void DisableInt(UINT intno);
void EnableInt(UINT intno, int priority);
void ClearInt(UINT intno);
typedef struct
{
    void *exinf;
    UINT tskatr;
    FP task;
    int itskpri;
    int stksz;
    void *bufptr;
    UW assprc;
} T_CTSK;
ID tk_cre_tsk(const T_CTSK *task);
ER tk_sta_tsk(ID id, INT code);
ER tk_del_tsk(ID id);
void tk_ext_tsk(void);
ER tk_dly_tsk(UINT delay);
#endif
