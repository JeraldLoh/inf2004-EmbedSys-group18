#ifndef GROUP18_WHEEL_PATH_H
#define GROUP18_WHEEL_PATH_H

#include <stdbool.h>
#include <stdint.h>

#ifndef B2_RETRACE_SAMPLES
#define B2_RETRACE_SAMPLES 256U
#endif
#if B2_RETRACE_SAMPLES < 2 || B2_RETRACE_SAMPLES > 4096
#error B2_RETRACE_SAMPLES must be between 2 and 4096
#endif

/* Forward wheel increments, newest first on playback. Encoder distances are
 * magnitudes: record only during forward LINE authority, never during pivots.
 * Playback follows successive wheel-distance waypoints, without stopping at
 * each sample. This preserves bend order; wheel slip remains unobservable. */
typedef struct
{
    uint16_t um[B2_RETRACE_SAMPLES][2];
    uint16_t head, count;
    uint16_t cursor, remaining;
    uint64_t waypoint_um[2];
    uint32_t segment_um[2];
} wheel_path_t;

static inline void wheel_path_clear(wheel_path_t *path)
{
    path->head = 0U;
    path->count = 0U;
    path->remaining = 0U;
}

static inline void wheel_path_record(wheel_path_t *path, uint64_t left,
    uint64_t right)
{
    /* Never turn an invalid gap into a fabricated saturated path sample. */
    if (left > UINT16_MAX || right > UINT16_MAX)
    {
        wheel_path_clear(path);
        return;
    }
    path->um[path->head][0] = (uint16_t)left;
    path->um[path->head][1] = (uint16_t)right;
    path->head = (uint16_t)((path->head + 1U) % B2_RETRACE_SAMPLES);
    if (path->count < B2_RETRACE_SAMPLES) { path->count++; }
}

static inline bool wheel_path_plan(wheel_path_t *path, uint32_t target_mm,
    uint64_t total_um[2])
{
    uint16_t index = path->head;
    uint16_t n;
    total_um[0] = 0U;
    total_um[1] = 0U;
    path->remaining = 0U;
    for (n = 0U; n < path->count; ++n)
    {
        index = (uint16_t)((index + B2_RETRACE_SAMPLES - 1U) % B2_RETRACE_SAMPLES);
        total_um[0] += path->um[index][0];
        total_um[1] += path->um[index][1];
        if ((total_um[0] + total_um[1]) / 2U >= (uint64_t)target_mm * 1000U)
        {
            path->cursor = path->head;
            path->remaining = (uint16_t)(n + 1U);
            path->waypoint_um[0] = 0U;
            path->waypoint_um[1] = 0U;
            return target_mm != 0U;
        }
    }
    return false; /* Full requested history is required, not half a barcode. */
}

static inline void wheel_path_next(wheel_path_t *path, uint32_t chunk_um)
{
    path->segment_um[0] = 0U;
    path->segment_um[1] = 0U;
    while (path->remaining != 0U)
    {
        path->cursor = (uint16_t)((path->cursor + B2_RETRACE_SAMPLES - 1U) %
            B2_RETRACE_SAMPLES);
        path->remaining--;
        path->segment_um[0] += path->um[path->cursor][0];
        path->segment_um[1] += path->um[path->cursor][1];
        if ((path->segment_um[0] + path->segment_um[1]) / 2U >= chunk_um)
        { break; }
    }
    path->waypoint_um[0] += path->segment_um[0];
    path->waypoint_um[1] += path->segment_um[1];
}

#endif
