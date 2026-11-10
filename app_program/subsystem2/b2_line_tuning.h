#ifndef GROUP18_B2_LINE_TUNING_H
#define GROUP18_B2_LINE_TUNING_H

#include <stdint.h>

/* Local B2 trial only: Buddy3's restored +/-20 hold maps to +/-2.8/m.
 * Remove that small command, then progressively recover the full turn
 * range by +/-14/m. This does not change Buddy3 perception or its output. */
#define B2_LINE_HOLD_DEADBAND_U1_M 2800000LL
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
/* Carry at most half the previous correction through a brief sensor gap. */
#define B2_LINE_BEND_CARRY_MS 160U
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
/* Give a fresh correction above Buddy3's hold a useful minimum wheel split. */
#define B2_LINE_MIN_DETECTED_CURVATURE_U1_M 3000000LL
#if defined(APP_S3_LINE_PID) && APP_S3_LINE_PID
#define B2_FIRMWARE_TAG "b2-cruise250-bend15-pid"
#else
#define B2_FIRMWARE_TAG "b2-cruise70-outside15"
#endif

static inline int32_t b2_line_shape_curvature(int64_t curvature)
{
    int64_t amount = curvature < 0 ? -curvature : curvature;
    if (amount <= B2_LINE_HOLD_DEADBAND_U1_M) { return 0; }
    if (amount > B2_LINE_FULL_CURVATURE_U1_M)
    { amount = B2_LINE_FULL_CURVATURE_U1_M; }
    amount = (amount - B2_LINE_HOLD_DEADBAND_U1_M) *
        B2_LINE_FULL_CURVATURE_U1_M /
        (B2_LINE_FULL_CURVATURE_U1_M - B2_LINE_HOLD_DEADBAND_U1_M);
    return (int32_t)(curvature < 0 ? -amount : amount);
}

#endif
