#include "safety.h"

#include <stddef.h>

void safety_init(safety_state_t *state)
{
    if (state != NULL)
    {
        state->active_reasons = STOP_REASON_NONE;
    }
}

void safety_latch(safety_state_t *state, stop_reason_mask_t reasons)
{
    if (state != NULL)
    {
        state->active_reasons |= reasons;
    }
}

void safety_clear(safety_state_t *state, stop_reason_mask_t reasons)
{
    if (state != NULL)
    {
        state->active_reasons &= ~reasons;
    }
}

bool safety_motion_allowed(const safety_state_t *state)
{
    return (state != NULL) && (state->active_reasons == STOP_REASON_NONE);
}
