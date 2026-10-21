/*
 * Shared vehicle fault bitmask: atomic updates on the dual-core RP2040.
 *
 * Cortex-M0+ has no exclusive load/store, so "word |= bit" is three
 * instructions and two writers could lose each other's bits. The SMP build
 * uses the kernel's atomic_* (hardware spinlock + interrupts masked), which
 * excludes the other core and local handlers. The single-core build only
 * needs local interrupt masking.
 */
#include <tk/tkernel.h>
#if TK_SUPPORT_SMP
#include <tk/smp.h>
#endif

#include "fault_mask.h"

/* 32-bit aligned, as the kernel atomics require. */
static UW fault_word __attribute__((aligned(4)));

void fault_mask_set(fault_mask_t bits)
{
#if TK_SUPPORT_SMP
    (void)atomic_bitset(&fault_word, bits);
#else
    UINT intsts;

    DI(intsts);
    fault_word |= bits;
    EI(intsts);
#endif
}

void fault_mask_clear(fault_mask_t bits)
{
#if TK_SUPPORT_SMP
    (void)atomic_bitclr(&fault_word, bits);
#else
    UINT intsts;

    DI(intsts);
    fault_word &= ~bits;
    EI(intsts);
#endif
}

void fault_mask_update(fault_mask_t field, fault_mask_t value)
{
#if TK_SUPPORT_SMP
    UW old;
    UW next;

    /* Retry until no other writer changed the word in between. */
    do
    {
        old = *(volatile UW *)&fault_word;
        next = (old & ~field) | (value & field);
    } while (atomic_cmpxchg(&fault_word, next, old) != old);
#else
    UINT intsts;

    DI(intsts);
    fault_word = (fault_word & ~field) | (value & field);
    EI(intsts);
#endif
}

fault_mask_t fault_mask_read(void)
{
    return *(volatile UW *)&fault_word;
}
