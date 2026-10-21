#ifndef FAULT_MASK_H
#define FAULT_MASK_H

/*
 * Shared vehicle fault bitmask (Change Requirement 1, Week 8 tribunal).
 *
 * One 32-bit word holds every subsystem's fault conditions, one bit each,
 * instead of separate integer variables. Each buddy owns one 4-bit field
 * and only ever sets or clears bits inside its own field.
 *
 * Several tasks and handlers, on both RP2040 cores, write the same word, so
 * every write is an atomic read-modify-write (fault_mask.c): setting or
 * clearing one buddy's bit can never lose another buddy's bit. Reads are a
 * single aligned 32-bit load and need no lock.
 *
 * Field allocation agreed in principle at the tribunal ("separate bits for
 * each subsystem"). Bit names inside each field are defined by that field's
 * owner; review any change with the affected owner and Buddy 5.
 */
#include <stdint.h>

typedef uint32_t fault_mask_t;

#define FAULT_FIELD_WIDTH 4U
#define FAULT_FIELD(buddy) ((fault_mask_t)0xFU << (((buddy) - 1U) * FAULT_FIELD_WIDTH))

#define FAULT_MASK_B1 FAULT_FIELD(1U)   /* bits 3..0   comms (e.g. Wi-Fi loss) */
#define FAULT_MASK_B2 FAULT_FIELD(2U)   /* bits 7..4   motion (stall, encoder) */
#define FAULT_MASK_B3 FAULT_FIELD(3U)   /* bits 11..8  line/barcode */
#define FAULT_MASK_B4 FAULT_FIELD(4U)   /* bits 15..12 IMU/terrain */
#define FAULT_MASK_B5 FAULT_FIELD(5U)   /* bits 19..16 obstacle (no echo, ...) */
/* bits 31..20 reserved (controller / future use) */

/* Buddy 4 field. */
enum
{
    FAULT_B4_IMU = 1UL << 12,        /* IMU bus error or no fresh sample */
    FAULT_B4_IMU_UNCAL = 1UL << 13,  /* no valid level calibration */
    FAULT_B4_MAG = 1UL << 14,        /* magnetometer missing: no heading */
    FAULT_B4_COLLISION = 1UL << 15   /* collision latched until cleared */
};

/* Atomically OR bits in. Safe from tasks and handlers on either core. */
void fault_mask_set(fault_mask_t bits);

/* Atomically clear bits. Other bits are preserved. */
void fault_mask_clear(fault_mask_t bits);

/*
 * Atomically replace the bits selected by field with value (e.g. one buddy
 * publishing its whole field at once). Bits outside field are preserved.
 */
void fault_mask_update(fault_mask_t field, fault_mask_t value);

/* Current mask (one aligned load). */
fault_mask_t fault_mask_read(void);

#endif
