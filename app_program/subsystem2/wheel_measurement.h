#ifndef GROUP18_WHEEL_MEASUREMENT_H
#define GROUP18_WHEEL_MEASUREMENT_H
#include "drv_encoder.h"

typedef struct
{
    uint32_t diameter_mm;
    uint32_t counts_per_rev[2];
    uint32_t speed_window_us;
    uint32_t max_gap_us;
    uint32_t max_edge_rate;
    /* Zero = strict. Test candidate permits <=5% batched observed edges and
     * <=2 simultaneous R/F observations per100ms, independently per wheel. */
    uint32_t max_batched_edge_permille;
    uint32_t max_batches_per_100ms;
} wheel_measurement_config_t;

typedef struct
{
    uint64_t travel_um[2]; /* Unsigned wheel rotation, NOT ground position. */
    uint32_t speed_um_s[2]; /* Magnitude; channel B cannot measure direction. */
    uint32_t acquired_us;
    uint32_t reset_epoch;
    bool speed_valid;
    bool signed_speed_valid;
    int8_t command_sign[2];
    uint32_t window_batches[2]; /* Last completed speed window. */
} wheel_measurement_sample_t;

typedef enum {
    WHEEL_REJECT_NONE, WHEEL_REJECT_ACQUISITION, WHEEL_REJECT_RESET,
    WHEEL_REJECT_GAP, WHEEL_REJECT_EDGE_RATE, WHEEL_REJECT_BATCH_COUNT,
    WHEEL_REJECT_BATCH_RATE, WHEEL_REJECT_BATCH_FRACTION,
    WHEEL_REJECT_TRAVEL_RANGE, WHEEL_REJECT_SPEED_RANGE
} wheel_reject_reason_t;

typedef struct {
    wheel_reject_reason_t reason;
    uint32_t interval_us, edges[2], batches[2], window_edges[2], window_batches[2];
    uint8_t wheel_mask;
} wheel_reject_t;

typedef struct
{
    wheel_measurement_config_t config;
    motion_encoder_snapshot_t previous;
    wheel_measurement_sample_t sample;
    wheel_reject_t reject;
    uint64_t remainder[2];
    uint64_t window_um[2];
    uint32_t window_us;
    uint32_t window_edges[2], window_batches[2];
    uint32_t window_direction_epoch;
    bool direction_changed;
    bool primed;
} wheel_measurement_t;

bool wheel_measurement_init(wheel_measurement_t *measurement,
    const wheel_measurement_config_t *config);
/* Epoch/gap/rate/quality failure returns unavailable. A new clean window
 * can recover after rebasing. The executor decides whether recovery is permitted.
 * Cumulative coalescing counters never clear on capture; policy uses deltas. */
motion_status_t wheel_measurement_update(wheel_measurement_t *measurement,
    const motion_encoder_snapshot_t *snapshot);
const char *wheel_reject_name(wheel_reject_reason_t reason);
#endif
