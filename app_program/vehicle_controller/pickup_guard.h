#ifndef GROUP18_PICKUP_GUARD_H
#define GROUP18_PICKUP_GUARD_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    uint32_t sample_changed_ms, candidate_ms, floor_ms;
    uint16_t sequence;
    bool primed, candidate_timing, floor_timing, latched;
} pickup_guard_t;

/* Candidate polarity/ADC thresholds must come from floor/hump/lift readings.
 * A frozen sensor word must never turn into new evidence. Once stopped, a
 * missing sample keeps START locked until fresh floor evidence is available. */
static inline bool pickup_guard_update(pickup_guard_t *guard, uint32_t now,
    uint16_t sequence, bool valid, bool candidate, bool floor, bool eligible,
    bool imu_agrees, uint32_t confirm_ms, uint32_t alone_ms,
    uint32_t rearm_ms, uint32_t stale_ms)
{
    if (guard->primed && (uint32_t)(now - guard->sample_changed_ms) >= stale_ms)
    {
        guard->candidate_timing = false;
        guard->floor_timing = false;
    }
    if (!guard->primed || sequence != guard->sequence)
    {
        guard->primed = true;
        guard->sequence = sequence;
        guard->sample_changed_ms = now;
    }
    valid = valid && (uint32_t)(now - guard->sample_changed_ms) < stale_ms;
    if (guard->latched)
    {
        if (valid && floor)
        {
            if (!guard->floor_timing)
            { guard->floor_timing = true; guard->floor_ms = now; }
            if ((uint32_t)(now - guard->floor_ms) >= rearm_ms)
            { guard->latched = false; guard->candidate_timing = false; }
        }
        else { guard->floor_timing = false; }
        return guard->latched;
    }
    guard->floor_timing = false;
    if (!valid || !eligible || !candidate)
    { guard->candidate_timing = false; return false; }
    if (!guard->candidate_timing)
    { guard->candidate_timing = true; guard->candidate_ms = now; }
    if ((uint32_t)(now - guard->candidate_ms) >= (imu_agrees ? confirm_ms : alone_ms))
    { guard->latched = true; }
    return guard->latched;
}

#endif
