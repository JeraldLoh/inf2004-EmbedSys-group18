#include "obstacle.h"

#define B5_PI 3.14159265358979323846f
#define DETOUR_TIE_CM 1.0f

// Local float helpers: the micro T-Kernel image is not linked with libm.
static float b5_fabsf(float x) { return x < 0.0f ? -x : x; }
static float b5_fmaxf(float a, float b) { return a > b ? a : b; }

// Taylor series to x^11 after reducing to [-pi/2, pi/2]; error < 1e-7.
static float b5_sinf(float x) {
    while (x > B5_PI) x -= 2.0f * B5_PI;
    while (x < -B5_PI) x += 2.0f * B5_PI;
    if (x > 0.5f * B5_PI) x = B5_PI - x;
    if (x < -0.5f * B5_PI) x = -B5_PI - x;
    float x2 = x * x;
    return x * (1.0f - x2 / 6.0f * (1.0f - x2 / 20.0f * (1.0f - x2 / 42.0f *
           (1.0f - x2 / 72.0f * (1.0f - x2 / 110.0f)))));
}

static float b5_cosf(float x) { return b5_sinf(x + 0.5f * B5_PI); }

static float b5_sqrtf(float x) {
    if (!(x > 0.0f)) return 0.0f;
    float r = x > 1.0f ? x * 0.5f : 1.0f;
    for (int i = 0; i < 30; i++) r = 0.5f * (r + x / r);
    return r;
}

static float left_offset_deg(const scan_geometry_t *g, float angle_deg) {
    float off = angle_deg - (float)g->center_deg;
    return g->left_is_higher ? off : -off;
}

side_t scan_side_of(const scan_geometry_t *g, float angle_deg) {
    float off = left_offset_deg(g, angle_deg);
    if (off > 0.0f) return SIDE_LEFT;
    if (off < 0.0f) return SIDE_RIGHT;
    return SIDE_NONE;
}

point2_t scan_polar_to_xy(const scan_geometry_t *g, float angle_deg, float cm) {
    float rad = left_offset_deg(g, angle_deg) * (B5_PI / 180.0f);
    point2_t p = {-cm * b5_sinf(rad), cm * b5_cosf(rad)};
    return p;
}

bool obstacle_is_clear(range_reading_t r, float obstacle_cm, float margin_cm, float max_cm) {
    return range_effective_cm(r, max_cm) > obstacle_cm + margin_cm;
}

obstacle_profile_t obstacle_build_profile(float distance_cm, obstacle_edge_t left,
                                          obstacle_edge_t right) {
    obstacle_profile_t p;
    p.distance_cm = distance_cm;
    p.left = left;
    p.right = right;
    float dx = left.pt.x_cm - right.pt.x_cm;
    float dy = left.pt.y_cm - right.pt.y_cm;
    p.width_cm = b5_sqrtf(dx * dx + dy * dy);
    p.center.x_cm = 0.5f * (left.pt.x_cm + right.pt.x_cm);
    p.center.y_cm = 0.5f * (left.pt.y_cm + right.pt.y_cm);
    return p;
}

// Effective range at the scan point furthest from center on the given side.
static float outer_clearance(const scan_geometry_t *g, const scan_point_t *pts, size_t n,
                             side_t side, float max_range_cm) {
    float best_off = 0.0f;
    float clear = 0.0f;
    for (size_t i = 0; i < n; i++) {
        float a = (float)pts[i].angle_deg;
        if (scan_side_of(g, a) != side) continue;
        float off = b5_fabsf(left_offset_deg(g, a));
        if (off > best_off) {
            best_off = off;
            clear = range_effective_cm(pts[i].reading, max_range_cm);
        }
    }
    return clear;
}

side_choice_t obstacle_choose_side(const scan_geometry_t *g, const scan_point_t *coarse,
                                   size_t count, const obstacle_profile_t *profile,
                                   float min_clearance_cm, float max_range_cm) {
    side_choice_t c;
    c.left_clear_cm = outer_clearance(g, coarse, count, SIDE_LEFT, max_range_cm);
    c.right_clear_cm = outer_clearance(g, coarse, count, SIDE_RIGHT, max_range_cm);

    bool left_ok = c.left_clear_cm >= min_clearance_cm;
    bool right_ok = c.right_clear_cm >= min_clearance_cm;

    if (!left_ok && !right_ok) {
        c.side = SIDE_NONE;
    } else if (left_ok != right_ok) {
        c.side = left_ok ? SIDE_LEFT : SIDE_RIGHT;
    } else if (profile->left.found != profile->right.found) {
        c.side = profile->left.found ? SIDE_LEFT : SIDE_RIGHT;
    } else {
        float left_detour = b5_fmaxf(0.0f, -profile->left.pt.x_cm);
        float right_detour = b5_fmaxf(0.0f, profile->right.pt.x_cm);
        if (b5_fabsf(left_detour - right_detour) >= DETOUR_TIE_CM) {
            c.side = left_detour < right_detour ? SIDE_LEFT : SIDE_RIGHT;
        } else {
            c.side = c.right_clear_cm > c.left_clear_cm ? SIDE_RIGHT : SIDE_LEFT;
        }
    }
    return c;
}

const char *side_name(side_t s) {
    switch (s) {
        case SIDE_LEFT: return "LEFT";
        case SIDE_RIGHT: return "RIGHT";
        default: return "NONE";
    }
}
