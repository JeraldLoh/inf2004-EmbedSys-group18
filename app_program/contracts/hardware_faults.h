#ifndef GROUP18_HARDWARE_FAULTS_H
#define GROUP18_HARDWARE_FAULTS_H

#include <stdbool.h>
#include <stdint.h>

/*
 * Shared hardware/sensor failure mask.
 *
 * Only the low 20 bits are carried by the compact protocol.  Individual bit
 * meanings are deliberately not allocated here until each subsystem has a
 * verified detection and clear rule.  Normal operating stops, command state
 * and network state have separate fields and must not be placed in this mask.
 *
 * The Vehicle Controller is the single runtime owner of the combined mask.
 * Subsystems report fault evidence; they do not update a shared word directly.
 */
typedef uint32_t hardware_fault_mask_t;

#define HARDWARE_FAULT_WIDTH 20U
#define HARDWARE_FAULT_VALID_MASK ((hardware_fault_mask_t)0x000fffffUL)

/*
 * PROVISIONAL allocations (Buddy 4 proposal, team to confirm). Bits 12..15
 * follow the tribunal's one-block-of-4-per-buddy idea. Detection and clear
 * rules are in app_program/subsystem4/README.md; not yet verified on the car.
 */
#define HARDWARE_FAULT_IMU ((hardware_fault_mask_t)1UL << 12)  /* B4: IMU unreachable or stale >= 1 s */
#define HARDWARE_FAULT_MAG ((hardware_fault_mask_t)1UL << 13)  /* B4: magnetometer missing/failing >= 1 s */

typedef struct
{
    hardware_fault_mask_t latched;
} hardware_fault_state_t;

bool hardware_fault_mask_is_valid(hardware_fault_mask_t faults);
void hardware_fault_state_init(hardware_fault_state_t *state);
void hardware_fault_state_latch(
    hardware_fault_state_t *state,
    hardware_fault_mask_t faults);
void hardware_fault_state_clear(
    hardware_fault_state_t *state,
    hardware_fault_mask_t faults);
hardware_fault_mask_t hardware_fault_state_read(
    const hardware_fault_state_t *state);

#endif
