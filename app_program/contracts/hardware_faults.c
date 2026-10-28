#include "hardware_faults.h"

#include <stddef.h>

bool hardware_fault_mask_is_valid(hardware_fault_mask_t faults)
{
    return (faults & ~HARDWARE_FAULT_VALID_MASK) == 0U;
}

void hardware_fault_state_init(hardware_fault_state_t *state)
{
    if (state != NULL)
    {
        state->latched = 0U;
    }
}

void hardware_fault_state_latch(
    hardware_fault_state_t *state,
    hardware_fault_mask_t faults)
{
    if ((state != NULL) && hardware_fault_mask_is_valid(faults))
    {
        state->latched |= faults;
    }
}

void hardware_fault_state_clear(
    hardware_fault_state_t *state,
    hardware_fault_mask_t faults)
{
    if ((state != NULL) && hardware_fault_mask_is_valid(faults))
    {
        state->latched &= ~faults;
    }
}

hardware_fault_mask_t hardware_fault_state_read(
    const hardware_fault_state_t *state)
{
    return (state == NULL) ? 0U : state->latched;
}
