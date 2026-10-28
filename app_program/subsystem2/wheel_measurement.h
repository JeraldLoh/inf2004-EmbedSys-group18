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
} wheel_measurement_sample_t;

typedef struct
{
    wheel_measurement_config_t config;
    motion_encoder_snapshot_t previous;
    wheel_measurement_sample_t sample;
    uint64_t remainder[2];
    uint64_t window_um[2];
    uint32_t window_us;
    uint32_t window_direction_epoch;
    bool direction_changed;
    bool primed;
} wheel_measurement_t;

bool wheel_measurement_init(wheel_measurement_t *measurement,
    const wheel_measurement_config_t *config);
/* Epoch change, excessive gap/rate, coalescing or clock failure is explicit
 * unavailable, never a large valid jump/zero. No counter reset per goal. */
motion_status_t wheel_measurement_update(wheel_measurement_t *measurement,
    const motion_encoder_snapshot_t *snapshot);
#endif
