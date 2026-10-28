#ifndef B2_TEST_SMP_H
#define B2_TEST_SMP_H
#include <tk/tkernel.h>
typedef volatile UW T_SPLOCK;
ER InitSpinLock(T_SPLOCK *lock);
ER ISpinLock(T_SPLOCK *lock, UINT *saved);
ER ISpinUnlock(T_SPLOCK *lock, UINT saved);
#endif
