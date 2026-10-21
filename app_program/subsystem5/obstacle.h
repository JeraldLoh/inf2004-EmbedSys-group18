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

// Robot frame, origin at the servo pivot: +x to the right, +y forward.
typedef struct {
    float x_cm;
    float y_cm;
} point2_t;

typedef struct {
    point2_t pt;
    float angle_deg;
    bool found; // false: obstacle extends past the scan window on this side
} obstacle_edge_t;

typedef struct {
    float distance_cm; // straight ahead to the obstacle face
    float width_cm;    // edge-to-edge; a lower bound if either edge wasn't found
    point2_t center;   // midpoint between the edges
    obstacle_edge_t left;
    obstacle_edge_t right;
} obstacle_profile_t;

typedef struct {
    side_t side;          // SIDE_NONE when neither side has enough clearance
    float left_clear_cm;  // effective range at the outermost left scan angle
    float right_clear_cm; // effective range at the outermost right scan angle
} side_choice_t;

side_t scan_side_of(const scan_geometry_t *g, float angle_deg);
point2_t scan_polar_to_xy(const scan_geometry_t *g, float angle_deg, float cm);

// True when a reading is far enough past the obstacle face to count as open.
bool obstacle_is_clear(range_reading_t r, float obstacle_cm, float margin_cm, float max_cm);

obstacle_profile_t obstacle_build_profile(float distance_cm, obstacle_edge_t left,
                                          obstacle_edge_t right);

// Picks the bypass side from a coarse scan and the profile. A side is only
// eligible if its outermost reading has at least min_clearance_cm. Among
// eligible sides: prefer the one whose edge was actually found, then the
// shorter lateral detour, then the larger clearance.
side_choice_t obstacle_choose_side(const scan_geometry_t *g, const scan_point_t *coarse,
                                   size_t count, const obstacle_profile_t *profile,
                                   float min_clearance_cm, float max_range_cm);

const char *side_name(side_t s);

#endif
