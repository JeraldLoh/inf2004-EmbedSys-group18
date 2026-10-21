#ifndef GROUP18_LINE_FOLLOW_H
#define GROUP18_LINE_FOLLOW_H

/* Buddy 3: two-sensor line following logic (hardware independent).
 *
 * Sensor layout assumed: the two digital IR sensors STRADDLE the 18 mm line.
 *
 *        left IR (GP7)   [ black line ]   right IR (GP0)
 *
 *   both white  -> line is between the sensors, car is centred
 *   left black  -> car drifted right, line is now under the left sensor -> steer left
 *   right black -> car drifted left -> steer right
 *   both black  -> a crossing line (junction such as "+") is under both sensors
 *
 * Processing per sample:
 *   raw reading -> 2-of-3 majority filter -> state machine -> steering value
 *
 * This file only turns sensor readings into evidence. It never drives motors;
 * Buddy 2 / the controller turn the steering proposal into wheel speeds.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/control_messages.h"

/* Line state: 5 values, 3 bits (Change Requirement 1). */
typedef enum
{
    LINE_STATE_CENTRED = 0,
    LINE_STATE_STEER_LEFT,
    LINE_STATE_STEER_RIGHT,
    LINE_STATE_JUNCTION,     /* both black for LINE_JUNCTION_CONFIRM_MS */
    LINE_STATE_REACQUIRE     /* searching for the line, e.g. after a bypass */
} line_state_t;

/* Which sensor saw the line most recently: 3 values, 2 bits. */
typedef enum
{
    LINE_SIDE_NONE = 0,
    LINE_SIDE_LEFT,
    LINE_SIDE_RIGHT
} line_side_t;

/* Published status packed into ONE byte: 1+1+3+2+1 = 8 bits.
 * A whole-byte store is a single STRB instruction on the Cortex-M0+, so a
 * reader on the other core always sees a complete, consistent value as long
 * as the writer builds it locally and stores it in one assignment.
 */
typedef struct
{
    uint8_t left_black : 1;      /* filtered reading */
    uint8_t right_black : 1;
    uint8_t state : 3;           /* line_state_t */
    uint8_t last_side : 2;       /* line_side_t  */
    uint8_t barcode_active : 1;  /* barcode being read: hold a steady speed */
} line_status_t;

/* Junction shape: 3 values, 2 bits.
 *
 *   CROSS "+"  line continues ahead    default (no barcode): STRAIGHT
 *   T          line ends at the bar    default (no barcode): RIGHT
 *
 * Two straddling sensors read the same on both shapes, so the shape is found
 * with a probe: stop just past the crossbar, pivot ~10 deg left and back. If
 * the line continues ahead, the swing puts it under the RIGHT sensor.
 */
typedef enum
{
    JUNCTION_TYPE_UNKNOWN = 0,
    JUNCTION_TYPE_CROSS,
    JUNCTION_TYPE_T
} junction_type_t;

typedef struct
{
    line_status_t status;
    uint8_t history_left;        /* last 3 raw samples, bit 0 = newest */
    uint8_t history_right;
    uint8_t previous_state;      /* state before the current one */
    uint32_t both_black_since_ms;
    uint32_t state_since_ms;
    bool both_black_active;
    bool junction_reported;
    bool probing;
    bool probe_saw_line;
} line_follow_t;

/* ---- Tuning (all overridable with -D, tune on the real track) ---------- */

/* Both sensors must read black this long before a junction is confirmed.
 * At 150 mm/s the 18 mm crossbar is under the sensors for ~120 ms. */
#ifndef LINE_JUNCTION_CONFIRM_MS
#define LINE_JUNCTION_CONFIRM_MS 30U
#endif

/* Steering output is -100 (full left) .. +100 (full right).
 * Correction starts at STEER_BASE and grows by 1 every STEER_RAMP_MS while
 * the car stays off centre, so a gentle drift gets a gentle correction and a
 * sharp curve gets a strong one. */
#ifndef LINE_STEER_BASE
#define LINE_STEER_BASE 30
#endif
#ifndef LINE_STEER_RAMP_MS
#define LINE_STEER_RAMP_MS 2U
#endif
#ifndef LINE_STEER_MAX
#define LINE_STEER_MAX 100
#endif

/* Damping: right after a correction ends the car is centred but still
 * pointing across the line. A short, small counter-steer straightens it and
 * stops the zig-zag (the role a D term plays in a PID controller). */
#ifndef LINE_DAMP_STEER
#define LINE_DAMP_STEER 15
#endif
#ifndef LINE_DAMP_MS
#define LINE_DAMP_MS 40U
#endif

/* Steering used while searching for a lost line. */
#ifndef LINE_REACQUIRE_STEER
#define LINE_REACQUIRE_STEER 60
#endif

/* ---- Line following ---------------------------------------------------- */

void line_follow_reset(line_follow_t *lf);

/* Call once per sample with RAW readings. Returns the new packed status. */
line_status_t line_follow_update(
    line_follow_t *lf,
    bool left_black,
    bool right_black,
    uint32_t now_ms);

/* Steering proposal, -100 (left) .. 0 (straight) .. +100 (right). */
int8_t line_follow_steer(const line_follow_t *lf, uint32_t now_ms);

/* How long the current state has lasted. */
uint32_t line_follow_state_age_ms(const line_follow_t *lf, uint32_t now_ms);

/* Set by the task while the barcode decoder is capturing. */
void line_follow_set_barcode_active(line_follow_t *lf, bool active);

/* ---- Reacquire (requirement 9: rejoin the line after a bypass) --------- */

/* Start searching. side_hint says which side the line should be on; pass
 * LINE_SIDE_NONE to use the side the line was last seen on. The search ends
 * by itself as soon as either sensor sees black. */
void line_follow_reacquire_start(
    line_follow_t *lf,
    line_side_t side_hint,
    uint32_t now_ms);
bool line_follow_reacquiring(const line_follow_t *lf);

/* ---- Junctions --------------------------------------------------------- */

/* Fills a junction observation for the controller. Returns true exactly once
 * per crossing, when the junction is first confirmed. */
bool line_follow_take_junction(
    line_follow_t *lf,
    junction_observation_t *observation,
    uint32_t now_ms);

/* Probe for the junction shape. While probing, call line_follow_probe_sample
 * instead of line_follow_update (the car is pivoting, not following). */
void line_follow_probe_start(line_follow_t *lf);
void line_follow_probe_sample(line_follow_t *lf, bool left_black, bool right_black);
junction_type_t line_follow_probe_finish(line_follow_t *lf);

/* Direction to take when no barcode was read before this junction. */
route_command_t junction_default_route(junction_type_t type);

/* Junction observation once the shape is known (no-barcode case). */
void junction_observation_for_type(
    junction_observation_t *observation,
    junction_type_t type,
    uint32_t now_ms);

#endif
