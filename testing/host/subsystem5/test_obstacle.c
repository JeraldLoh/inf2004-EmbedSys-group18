#include <stdlib.h>

#include "obstacle.h"
#include "test.h"

static const scan_geometry_t G = {90, true};          // 150 = left, 30 = right
static const scan_geometry_t G_FLIPPED = {90, false}; // 30 = left, 150 = right

static range_reading_t ok(float cm) { return (range_reading_t){RANGE_OK, cm}; }
static range_reading_t st(range_status_t s) { return (range_reading_t){s, 0.0f}; }

// Edge in direction dir (+ = left of straight ahead) at cm.
static obstacle_edge_t edge_dir(float dir, float cm, bool found) {
    obstacle_edge_t e = {obstacle_polar_to_xy(dir, cm), dir, found};
    return e;
}

static void side_classification(void) {
    CHECK_INT(scan_side_of(&G, 150), SIDE_LEFT);
    CHECK_INT(scan_side_of(&G, 30), SIDE_RIGHT);
    CHECK_INT(scan_side_of(&G, 90), SIDE_NONE);
    CHECK_INT(scan_side_of(&G_FLIPPED, 150), SIDE_RIGHT);
    CHECK_INT(scan_side_of(&G_FLIPPED, 30), SIDE_LEFT);
    CHECK_NEAR(scan_left_offset_deg(&G, 102), 12.0f, 1e-6);
    CHECK_NEAR(scan_left_offset_deg(&G_FLIPPED, 102), -12.0f, 1e-6);
}

static void polar_to_xy(void) {
    point2_t c = scan_polar_to_xy(&G, 90, 20.0f);
    CHECK_NEAR(c.x_cm, 0.0f, 1e-4);
    CHECK_NEAR(c.y_cm, 20.0f, 1e-4);

    point2_t l = scan_polar_to_xy(&G, 180, 10.0f);
    CHECK_NEAR(l.x_cm, -10.0f, 1e-4);
    CHECK_NEAR(l.y_cm, 0.0f, 1e-4);

    point2_t r = scan_polar_to_xy(&G, 60, 10.0f);
    CHECK_NEAR(r.x_cm, 5.0f, 1e-4);
    CHECK_NEAR(r.y_cm, 8.6603f, 1e-3);

    point2_t f = scan_polar_to_xy(&G_FLIPPED, 60, 10.0f); // left when flipped
    CHECK_NEAR(f.x_cm, -5.0f, 1e-4);

    point2_t d = obstacle_polar_to_xy(-30.0f, 10.0f); // direction form, 30 deg right
    CHECK_NEAR(d.x_cm, 5.0f, 1e-4);
    CHECK_NEAR(d.y_cm, 8.6603f, 1e-3);
}

static void profile_geometry(void) {
    obstacle_profile_t p = obstacle_build_profile(20.0f, edge_dir(30, 20.0f, true),
                                                  edge_dir(-30, 20.0f, true));
    CHECK_NEAR(p.width_cm, 20.0f, 1e-3);
    CHECK_NEAR(p.center.x_cm, 0.0f, 1e-4);
    CHECK_NEAR(p.center.y_cm, 17.3205f, 1e-3);
    CHECK_NEAR(p.distance_cm, 20.0f, 1e-6);

    obstacle_profile_t off = obstacle_build_profile(10.0f, edge_dir(0, 10.0f, true),
                                                    edge_dir(-60, 10.0f, true));
    CHECK(off.center.x_cm > 0.0f); // obstacle sits to the right
}

static void clear_classification(void) {
    CHECK(obstacle_is_clear(ok(30.0f), 0.0f, 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(ok(23.0f), 0.0f, 15.0f, 8.0f, 400.0f)); // must strictly exceed
    CHECK(!obstacle_is_clear(ok(16.0f), 0.0f, 15.0f, 8.0f, 400.0f));
    CHECK(obstacle_is_clear(st(RANGE_NO_ECHO), 0.0f, 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(st(RANGE_TOO_CLOSE), 0.0f, 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(st(RANGE_FAULT), 0.0f, 15.0f, 8.0f, 400.0f));
}

static void clear_uses_forward_component(void) {
    // A flat face 15 cm ahead reads 30 cm at 60 deg: same face, not open space.
    CHECK(!obstacle_is_clear(ok(30.0f), 60.0f, 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(ok(30.0f), -60.0f, 15.0f, 8.0f, 400.0f));
    CHECK(obstacle_is_clear(ok(60.0f), 60.0f, 15.0f, 8.0f, 400.0f)); // 30 cm forward
    CHECK(obstacle_is_clear(st(RANGE_NO_ECHO), 80.0f, 15.0f, 8.0f, 400.0f));
}

static obstacle_profile_t symmetric_profile(bool lf, bool rf) {
    return obstacle_build_profile(12.0f, edge_dir(12.5f, 12.0f, lf), edge_dir(-12.5f, 12.0f, rf));
}

static void choose_only_usable_side(void) {
    obstacle_profile_t p = symmetric_profile(true, true);
    side_choice_t ch = obstacle_choose_side(&p, 100.0f, 10.0f, 25.0f);
    CHECK_INT(ch.side, SIDE_LEFT);
    CHECK_NEAR(ch.left_clear_cm, 100.0f, 1e-6);
    CHECK_NEAR(ch.right_clear_cm, 10.0f, 1e-6);
    CHECK_INT(obstacle_choose_side(&p, 10.0f, 100.0f, 25.0f).side, SIDE_RIGHT);
}

static void choose_none_when_boxed_in(void) {
    obstacle_profile_t p = symmetric_profile(true, true);
    CHECK_INT(obstacle_choose_side(&p, 24.9f, 10.0f, 25.0f).side, SIDE_NONE);

    // Edges never found (obstacle wider than the turn scan): never usable.
    p = symmetric_profile(false, false);
    side_choice_t ch = obstacle_choose_side(&p, 400.0f, 400.0f, 25.0f);
    CHECK_INT(ch.side, SIDE_NONE);
    CHECK_NEAR(ch.left_clear_cm, 0.0f, 1e-6);
    CHECK_NEAR(ch.right_clear_cm, 0.0f, 1e-6);
}

static void choose_requires_found_edge(void) {
    obstacle_profile_t p = symmetric_profile(false, true);
    CHECK_INT(obstacle_choose_side(&p, 100.0f, 100.0f, 25.0f).side, SIDE_RIGHT);
    p = symmetric_profile(true, false);
    CHECK_INT(obstacle_choose_side(&p, 100.0f, 100.0f, 25.0f).side, SIDE_LEFT);
}

static void choose_shorter_detour(void) {
    // Obstacle mostly on the right: left edge near center, right edge far out.
    obstacle_profile_t p = obstacle_build_profile(12.0f, edge_dir(2.5f, 12.0f, true),
                                                  edge_dir(-32.5f, 12.0f, true));
    CHECK_INT(obstacle_choose_side(&p, 100.0f, 100.0f, 25.0f).side, SIDE_LEFT);

    p = obstacle_build_profile(12.0f, edge_dir(32.5f, 12.0f, true), edge_dir(-2.5f, 12.0f, true));
    CHECK_INT(obstacle_choose_side(&p, 100.0f, 100.0f, 25.0f).side, SIDE_RIGHT);
}

static void choose_tie_breaks_on_clearance(void) {
    obstacle_profile_t p = symmetric_profile(true, true);
    CHECK_INT(obstacle_choose_side(&p, 60.0f, 90.0f, 25.0f).side, SIDE_RIGHT);
    CHECK_INT(obstacle_choose_side(&p, 60.0f, 60.0f, 25.0f).side, SIDE_LEFT);
}

static void plan_clears_the_edge(void) {
    obstacle_profile_t p = obstacle_build_profile(20.0f, edge_dir(20.0f, 20.0f, true),
                                                  edge_dir(-20.0f, 20.0f, true));
    bypass_plan_t l = obstacle_plan_bypass(&p, SIDE_LEFT, 15.0f, 30.0f, 20, 75);
    // Edge at x=-6.84, y=18.79; lateral 21.84 -> atan(21.84/18.79) = 49.3 deg.
    CHECK_INT(l.side, SIDE_LEFT);
    CHECK_INT(l.turn_deg, 49);
    CHECK_NEAR(l.lateral_cm, 21.84f, 0.02);
    CHECK_NEAR(l.out_cm, 21.84f / 0.75471f, 0.05); // lateral / sin(49)
    CHECK_NEAR(l.pass_cm, 30.0f, 1e-6);

    bypass_plan_t r = obstacle_plan_bypass(&p, SIDE_RIGHT, 15.0f, 30.0f, 20, 75);
    CHECK_INT(r.side, SIDE_RIGHT);
    CHECK_INT(r.turn_deg, 49);
    CHECK_NEAR(r.out_cm, l.out_cm, 1e-4);
}

static void plan_clamps_turn(void) {
    // Far, narrow obstacle: geometric angle ~5 deg, clamped up to the minimum.
    obstacle_profile_t far = obstacle_build_profile(200.0f, edge_dir(1.0f, 200.0f, true),
                                                    edge_dir(-1.0f, 200.0f, true));
    bypass_plan_t p = obstacle_plan_bypass(&far, SIDE_LEFT, 15.0f, 30.0f, 20, 75);
    CHECK_INT(p.turn_deg, 20);
    CHECK_NEAR(p.out_cm, p.lateral_cm / 0.34202f, 0.05); // still reaches the lateral offset

    // Edge almost beside the car: clamped down to the maximum.
    obstacle_profile_t wide = obstacle_build_profile(5.0f, edge_dir(80.0f, 5.0f, true),
                                                     edge_dir(-80.0f, 5.0f, true));
    CHECK_INT(obstacle_plan_bypass(&wide, SIDE_RIGHT, 15.0f, 30.0f, 20, 75).turn_deg, 75);
}

static void plan_angle_matches_libm(void) {
    // The planner's own atan (no libm on target) against the host's atan2.
    int worst = 0;
    for (int dir = 1; dir <= 85; dir += 3) {
        for (float cm = 5.0f; cm <= 200.0f; cm *= 1.7f) {
            obstacle_profile_t p = obstacle_build_profile(cm, edge_dir((float)dir, cm, true),
                                                          edge_dir((float)-dir, cm, true));
            bypass_plan_t b = obstacle_plan_bypass(&p, SIDE_LEFT, 15.0f, 30.0f, 1, 89);
            double lat = fabs((double)p.left.pt.x_cm) + 15.0;
            double fwd = p.left.pt.y_cm > 1.0f ? (double)p.left.pt.y_cm : 1.0;
            int ref = (int)(atan2(lat, fwd) * 180.0 / 3.14159265358979 + 0.5);
            if (ref < 1) ref = 1;
            if (ref > 89) ref = 89;
            int diff = abs(b.turn_deg - ref);
            if (diff > worst) worst = diff;
        }
    }
    CHECK(worst <= 1); // rounding at a .5 boundary can differ by one
}

void run_obstacle_tests(void) {
    RUN(side_classification);
    RUN(polar_to_xy);
    RUN(profile_geometry);
    RUN(clear_classification);
    RUN(clear_uses_forward_component);
    RUN(choose_only_usable_side);
    RUN(choose_none_when_boxed_in);
    RUN(choose_requires_found_edge);
    RUN(choose_shorter_detour);
    RUN(choose_tie_breaks_on_clearance);
    RUN(plan_clears_the_edge);
    RUN(plan_clamps_turn);
    RUN(plan_angle_matches_libm);
}
