#ifndef GROUP18_S4_ATOMIC_H
#define GROUP18_S4_ATOMIC_H

/*
 * Minimal single-word atomic access for Buddy 4 shared data.
 *
 * Aligned 32-bit loads and stores are single bus transactions on the RP2040
 * (both cores), so a whole packed word is always seen old or new, never torn.
 * The fence orders those accesses between cores (DMB on Cortex-M0+).
 *
 * Only plain load/store is offered on purpose: ARMv6-M has no exclusive
 * load/store, so read-modify-write would need a lock. Every shared Buddy 4
 * word has exactly one writer instead, and readers never write.
 */
#include <stdint.h>

#if defined(__GNUC__) || defined(__clang__)
#define S4_FENCE() __atomic_thread_fence(__ATOMIC_SEQ_CST)
#define S4_LOAD32(ptr) __atomic_load_n((ptr), __ATOMIC_RELAXED)
#define S4_STORE32(ptr, value) __atomic_store_n((ptr), (value), __ATOMIC_RELAXED)
#elif defined(_MSC_VER)
#include <intrin.h>
#define S4_FENCE() do { _ReadWriteBarrier(); _mm_mfence(); } while (0)
#define S4_LOAD32(ptr) (*(ptr))
#define S4_STORE32(ptr, value) (*(ptr) = (value))
#else
#error "Buddy 4 needs a compiler fence for this toolchain"
#endif

#endif
