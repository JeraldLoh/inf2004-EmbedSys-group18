#ifndef BUDDY5_OBSTACLE_H
#define BUDDY5_OBSTACLE_H

#include <stdbool.h>
#include <stddef.h>

#include "range.h"

typedef enum { SIDE_NONE = 0, SIDE_LEFT, SIDE_RIGHT } side_t;

typedef struct {
    int center_deg;      // servo angle that points straight ahead
    bool left_is_higher; // true if angles above center look to the left
} scan_geometry_t;

typedef struct {
    int angle_deg;
    range_reading_t reading;
} scan_point_t;

// Robot frame at the start of the scan, origin at the sensor: +x to the
// right, +y forward. A "direction" is degrees from straight ahead, + = left;
// it combines the car's own turn and the servo angle.
typedef struct {
    float x_cm;
    float y_cm;
} point2_t;

typedef struct {
    point2_t pt;
    float angle_deg; // direction of the edge (+ = left of straight ahead)
    bool found;      // false: obstacle extends past the widest direction scanned
} obstacle_edge_t;

typedef struct {
    float distance_cm; // straight ahead to the obstacle face
    float width_cm;    // edge-to-edge; a lower bound if either edge wasn't found
    point2_t center;   // midpoint between the edges
    obstacle_edge_t left;
    obstacle_edge_t right;
} obstacle_profile_t;

typedef struct {
    side_t side;          // SIDE_NONE when neither side is usable
    float left_clear_cm;  // free range just past the left edge (0 if not found)
    float right_clear_cm; // free range just past the right edge (0 if not found)
} side_choice_t;

// The three-leg detour around one side, mirrored on the way back:
// turn turn_deg toward the side, drive out_cm, turn back straight, drive
// pass_cm, turn turn_deg toward the line, drive out_cm, turn straight again.
typedef struct {
    side_t side;
    int turn_deg;  // magnitude, 1..89
    float out_cm;  // diagonal leg, used again for the return leg
    float pass_cm; // straight leg alongside the obstacle
    float lateral_cm;
} bypass_plan_t;

// Signed servo offset from center: + = looking left.
float scan_left_offset_deg(const scan_geometry_t *g, float servo_deg);
side_t scan_side_of(const scan_geometry_t *g, float servo_deg);
point2_t scan_polar_to_xy(const scan_geometry_t *g, float servo_deg, float cm);
point2_t obstacle_polar_to_xy(float direction_deg, float cm);

// True when a reading taken in direction_deg lands well behind the obstacle
// face: its forward component must exceed obstacle_cm + margin_cm. Using the
// forward component stops a flat face seen at a wide angle (which reads
// further away) from being mistaken for open space.
bool obstacle_is_clear(range_reading_t r, float direction_deg, float obstacle_cm,
                       float margin_cm, float max_cm);

obstacle_profile_t obstacle_build_profile(float distance_cm, obstacle_edge_t left,
                                          obstacle_edge_t right);

// A side is usable only if its edge was found and the free range just past
// the edge is at least min_clearance_cm. If both are usable: the shorter
// lateral detour wins, then the larger clearance, then LEFT.
side_choice_t obstacle_choose_side(const obstacle_profile_t *profile, float left_clear_cm,
                                   float right_clear_cm, float min_clearance_cm);

// Plans the detour so the car's center passes side_margin_cm outside the
// chosen edge. The turn angle is clamped to min_turn_deg..max_turn_deg.
bypass_plan_t obstacle_plan_bypass(const obstacle_profile_t *profile, side_t side,
                                   float side_margin_cm, float pass_cm, int min_turn_deg,
                                   int max_turn_deg);

const char *side_name(side_t s);

#endif
