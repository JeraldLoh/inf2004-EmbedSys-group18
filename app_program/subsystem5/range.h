#ifndef BUDDY5_RANGE_H
#define BUDDY5_RANGE_H

#include <stdbool.h>
#include <stddef.h>

#define RANGE_MAX_SAMPLES 15

typedef enum {
    RANGE_OK = 0,
    RANGE_NO_ECHO,   // nothing within max range: treated as open space
    RANGE_TOO_CLOSE, // inside the sensor's blind zone: treated as blocked
    RANGE_FAULT,     // no echo pulse at all / line stuck: wiring or sensor problem
    RANGE_STATUS_COUNT
} range_status_t;

typedef struct {
    range_status_t status;
    float cm; // meaningful only when status == RANGE_OK
} range_reading_t;

// Merges several raw samples into one reading. If at least half the samples
// are OK the result is their median (rejects single-sample spikes);
// otherwise it is the most frequent failure status, ties resolved toward the
// more conservative one (TOO_CLOSE > FAULT > NO_ECHO).
range_reading_t range_combine(const range_reading_t *samples, size_t count);

// Distance to use in comparisons: NO_ECHO counts as max_cm (open),
// TOO_CLOSE and FAULT count as 0 (blocked).
float range_effective_cm(range_reading_t r, float max_cm);

const char *range_status_name(range_status_t s);

#endif
