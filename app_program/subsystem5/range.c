#include "range.h"

#include <math.h>

range_reading_t range_combine(const range_reading_t *samples, size_t count) {
    range_reading_t out = {RANGE_FAULT, 0.0f};
    if (samples == NULL || count == 0 || count > RANGE_MAX_SAMPLES) {
        return out;
    }

    float ok[RANGE_MAX_SAMPLES];
    size_t n_ok = 0;
    size_t n_status[RANGE_STATUS_COUNT] = {0};

    for (size_t i = 0; i < count; i++) {
        range_status_t st = samples[i].status;
        float cm = samples[i].cm;
        if (st >= RANGE_STATUS_COUNT || (st == RANGE_OK && (!isfinite(cm) || cm < 0.0f))) {
            st = RANGE_FAULT;
        }
        n_status[st]++;
        if (st == RANGE_OK) {
            size_t j = n_ok++;
            while (j > 0 && ok[j - 1] > cm) {
                ok[j] = ok[j - 1];
                j--;
            }
            ok[j] = cm;
        }
    }

    if (n_ok * 2 >= count) {
        out.status = RANGE_OK;
        out.cm = (n_ok % 2 == 1) ? ok[n_ok / 2]
                                 : 0.5f * (ok[n_ok / 2 - 1] + ok[n_ok / 2]);
        return out;
    }

    static const range_status_t priority[] = {RANGE_TOO_CLOSE, RANGE_FAULT, RANGE_NO_ECHO};
    range_status_t best = priority[0];
    for (size_t i = 1; i < sizeof priority / sizeof priority[0]; i++) {
        if (n_status[priority[i]] > n_status[best]) {
            best = priority[i];
        }
    }
    out.status = best;
    return out;
}

float range_effective_cm(range_reading_t r, float max_cm) {
    switch (r.status) {
        case RANGE_OK: return r.cm;
        case RANGE_NO_ECHO: return max_cm;
        default: return 0.0f;
    }
}

const char *range_status_name(range_status_t s) {
    switch (s) {
        case RANGE_OK: return "ok";
        case RANGE_NO_ECHO: return "no-echo";
        case RANGE_TOO_CLOSE: return "too-close";
        case RANGE_FAULT: return "fault";
        default: return "invalid";
    }
}
