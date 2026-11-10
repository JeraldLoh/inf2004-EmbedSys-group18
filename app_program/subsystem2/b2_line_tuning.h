#ifndef GROUP18_B2_LINE_TUNING_H
#define GROUP18_B2_LINE_TUNING_H

#include <stdint.h>

/* Buddy 3's digital hold maps to +/-2.8/m. Start with a gentle correction
 * at the first tape contact and retain the full turn at +/-14/m. */
#define B2_LINE_HOLD_DEADBAND_U1_M 2800000LL
#define B2_LINE_SOFT_DEADBAND_U1_M 3500000LL
#define B2_LINE_FULL_CURVATURE_U1_M 14000000LL
#define B2_LINE_FULL_CURVATURE_1_M 14U
/* V19: another 10% off V18's inside target: 0.85 * 0.90 = 0.765.
 * Applied after paired target/cap; outside and straight targets unchanged. */
#define B2_LINE_WIDE_TURN_INNER_PERMILLE 765U
/* V17's20mm/s floor widened the operator's curve. Disable it to restore
 * the V16 target geometry while retaining V17's diagnostic reporting. */
#define B2_LINE_MIN_ROLLING_MM_S 0U
/* Tighten a bend with twice the normal paired ramp rate. */
#define B2_LINE_BEND_ENTRY_RAMP_MULTIPLIER 2U
/* Let the inside wheel shed speed faster than the outside wheel changes. */
#define B2_LINE_INSIDE_DECEL_MULTIPLIER 2U
/* Carry only a developed bend through a brief sensor gap. */
#define B2_LINE_BEND_CARRY_MS 60U
#define B2_LINE_MIN_CARRY_CURVATURE_U1_M 5000000LL
/* Extra PWM headroom for the outside wheel under bend load. */
#define B2_LINE_OUTSIDE_MAX_EFFORT 400U
#define B2_LINE_OUTSIDE_SPEED_PERCENT 115U
#define B2_LINE_FAST_STRAIGHT_MM_S 250U
#define B2_LINE_FAST_STRAIGHT_MAX_EFFORT 400U
/* Pre-split speed ceilings for gentle and maximum-curvature bends. */
#define B2_LINE_BEND_ENTRY_MM_S 57U
#define B2_LINE_BEND_TIGHT_MM_S 49U
/* Two 30% increases followed by 15%: 1.30 * 1.30 * 1.15 = 1.9435. */
#define B2_LINE_BEND_SPEED_SCALE_10000 19435U
#if defined(APP_S3_LINE_PID) && APP_S3_LINE_PID
#define B2_FIRMWARE_TAG "b2-straight70-hybrid"
#else
#define B2_FIRMWARE_TAG "b2-straight70-soft-digital"
#endif

static inline int32_t b2_line_shape_curvature(int64_t curvature)
{
    int64_t amount = curvature < 0 ? -curvature : curvature;
    if (amount <= B2_LINE_SOFT_DEADBAND_U1_M) { return 0; }
    if (amount > B2_LINE_FULL_CURVATURE_U1_M)
    { amount = B2_LINE_FULL_CURVATURE_U1_M; }
    amount = (amount - B2_LINE_SOFT_DEADBAND_U1_M) *
        B2_LINE_FULL_CURVATURE_U1_M /
        (B2_LINE_FULL_CURVATURE_U1_M - B2_LINE_SOFT_DEADBAND_U1_M);
    return (int32_t)(curvature < 0 ? -amount : amount);
}

#endif
