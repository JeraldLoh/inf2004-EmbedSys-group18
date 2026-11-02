#ifndef GROUP18_B2_LINE_TUNING_H
#define GROUP18_B2_LINE_TUNING_H

#include <stdint.h>

/* Local B2 trial only: Buddy3's restored +/-20 hold maps to +/-2.8/m.
 * Remove that small command, then progressively recover the full turn
 * range by +/-14/m. This does not change Buddy3 perception or its output. */
#define B2_LINE_HOLD_DEADBAND_U1_M 2800000LL
#define B2_LINE_FULL_CURVATURE_U1_M 14000000LL
#define B2_LINE_FULL_CURVATURE_1_M 14U
/* V16 wide-turn candidate: extra inside reduction after the paired target
 * calculation/cap. No extra outside boost; straight requests are untouched. */
#define B2_LINE_WIDE_TURN_INNER_PERCENT 85U
/* Unqualified local candidate: avoid requesting a sustained9-10mm/s crawl.
 * Request speed and hard wheel cap still win. Zero means no floor. */
#define B2_LINE_MIN_ROLLING_MM_S 20U
#define B2_FIRMWARE_TAG "v17-low-speed-diag"

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
