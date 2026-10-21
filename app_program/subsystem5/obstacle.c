#include "obstacle.h"

#define B5_PI 3.14159265358979323846f
#define DEG2RAD (B5_PI / 180.0f)
#define DETOUR_TIE_CM 1.0f

// Local float helpers: the micro T-Kernel image is not linked with libm.
static float b5_fabsf(float x) { return x < 0.0f ? -x : x; }

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

// Minimax polynomial for atan on [-1, 1]; error < 2e-5 rad.
static float atan_unit(float t) {
    float t2 = t * t;
    return t * (0.99997726f + t2 * (-0.33262347f + t2 * (0.19354346f + t2 * (-0.11643287f +
           t2 * (0.05265332f + t2 * -0.01172120f)))));
}

// atan2 for y >= 0, x > 0 (first quadrant), which is all the planner needs.
static float b5_atan2f_q1(float y, float x) {
    return y <= x ? atan_unit(y / x) : 0.5f * B5_PI - atan_unit(x / y);
}

float scan_left_offset_deg(const scan_geometry_t *g, float servo_deg) {
    float off = servo_deg - (float)g->center_deg;
    return g->left_is_higher ? off : -off;
}

side_t scan_side_of(const scan_geometry_t *g, float servo_deg) {
    float off = scan_left_offset_deg(g, servo_deg);
    if (off > 0.0f) return SIDE_LEFT;
    if (off < 0.0f) return SIDE_RIGHT;
    return SIDE_NONE;
}

point2_t obstacle_polar_to_xy(float direction_deg, float cm) {
    float rad = direction_deg * DEG2RAD;
    point2_t p = {-cm * b5_sinf(rad), cm * b5_cosf(rad)};
    return p;
}

point2_t scan_polar_to_xy(const scan_geometry_t *g, float servo_deg, float cm) {
    return obstacle_polar_to_xy(scan_left_offset_deg(g, servo_deg), cm);
}

bool obstacle_is_clear(range_reading_t r, float direction_deg, float obstacle_cm,
                       float margin_cm, float max_cm) {
    float forward = range_effective_cm(r, max_cm) * b5_cosf(direction_deg * DEG2RAD);
    return forward > obstacle_cm + margin_cm;
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

side_choice_t obstacle_choose_side(const obstacle_profile_t *profile, float left_clear_cm,
                                   float right_clear_cm, float min_clearance_cm) {
    side_choice_t c;
    c.left_clear_cm = profile->left.found ? left_clear_cm : 0.0f;
    c.right_clear_cm = profile->right.found ? right_clear_cm : 0.0f;

    bool left_ok = profile->left.found && c.left_clear_cm >= min_clearance_cm;
    bool right_ok = profile->right.found && c.right_clear_cm >= min_clearance_cm;

    if (!left_ok && !right_ok) {
        c.side = SIDE_NONE;
    } else if (left_ok != right_ok) {
        c.side = left_ok ? SIDE_LEFT : SIDE_RIGHT;
    } else {
        float left_detour = b5_fabsf(profile->left.pt.x_cm);
        float right_detour = b5_fabsf(profile->right.pt.x_cm);
        if (b5_fabsf(left_detour - right_detour) >= DETOUR_TIE_CM) {
            c.side = left_detour < right_detour ? SIDE_LEFT : SIDE_RIGHT;
        } else {
            c.side = c.right_clear_cm > c.left_clear_cm ? SIDE_RIGHT : SIDE_LEFT;
        }
    }
    return c;
}

bypass_plan_t obstacle_plan_bypass(const obstacle_profile_t *profile, side_t side,
                                   float side_margin_cm, float pass_cm, int min_turn_deg,
                                   int max_turn_deg) {
    const obstacle_edge_t *e = side == SIDE_RIGHT ? &profile->right : &profile->left;
    float lateral = b5_fabsf(e->pt.x_cm) + side_margin_cm;
    float forward = e->pt.y_cm > 1.0f ? e->pt.y_cm : 1.0f;
    if (lateral < 1.0f) lateral = 1.0f;

    int turn = (int)(b5_atan2f_q1(lateral, forward) / DEG2RAD + 0.5f);
    if (turn < min_turn_deg) turn = min_turn_deg;
    if (turn > max_turn_deg) turn = max_turn_deg;

    bypass_plan_t p;
    p.side = side == SIDE_RIGHT ? SIDE_RIGHT : SIDE_LEFT;
    p.turn_deg = turn;
    p.lateral_cm = lateral;
    p.out_cm = lateral / b5_sinf((float)turn * DEG2RAD);
    p.pass_cm = pass_cm;
    return p;
}

const char *side_name(side_t s) {
    switch (s) {
        case SIDE_LEFT: return "LEFT";
        case SIDE_RIGHT: return "RIGHT";
        default: return "NONE";
    }
}
