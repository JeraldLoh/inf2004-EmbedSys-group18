#ifndef SAFETY_H
#define SAFETY_H

#include "../contracts/system_types.h"

typedef struct
{
    stop_reason_mask_t active_reasons;
} safety_state_t;

void safety_init(safety_state_t *state);
void safety_latch(safety_state_t *state, stop_reason_mask_t reasons);
void safety_clear(safety_state_t *state, stop_reason_mask_t reasons);
bool safety_motion_allowed(const safety_state_t *state);

#endif
