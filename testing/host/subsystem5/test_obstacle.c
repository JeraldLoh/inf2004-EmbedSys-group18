#include "obstacle.h"
#include "test.h"

static const scan_geometry_t G = {90, true};          // 150 = left, 30 = right
static const scan_geometry_t G_FLIPPED = {90, false}; // 30 = left, 150 = right

static range_reading_t ok(float cm) { return (range_reading_t){RANGE_OK, cm}; }
static range_reading_t st(range_status_t s) { return (range_reading_t){s, 0.0f}; }

static obstacle_edge_t edge_at(const scan_geometry_t *g, float deg, float cm, bool found) {
    obstacle_edge_t e = {scan_polar_to_xy(g, deg, cm), deg, found};
    return e;
}

static void side_classification(void) {
    CHECK_INT(scan_side_of(&G, 150), SIDE_LEFT);
    CHECK_INT(scan_side_of(&G, 30), SIDE_RIGHT);
    CHECK_INT(scan_side_of(&G, 90), SIDE_NONE);
    CHECK_INT(scan_side_of(&G_FLIPPED, 150), SIDE_RIGHT);
    CHECK_INT(scan_side_of(&G_FLIPPED, 30), SIDE_LEFT);
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
}

static void profile_geometry(void) {
    obstacle_profile_t p = obstacle_build_profile(20.0f, edge_at(&G, 120, 20.0f, true),
                                                  edge_at(&G, 60, 20.0f, true));
    CHECK_NEAR(p.width_cm, 20.0f, 1e-3);
    CHECK_NEAR(p.center.x_cm, 0.0f, 1e-4);
    CHECK_NEAR(p.center.y_cm, 17.3205f, 1e-3);
    CHECK_NEAR(p.distance_cm, 20.0f, 1e-6);

    obstacle_profile_t off = obstacle_build_profile(10.0f, edge_at(&G, 90, 10.0f, true),
                                                    edge_at(&G, 30, 10.0f, true));
    CHECK(off.center.x_cm > 0.0f); // obstacle sits to the right
}

static void clear_classification(void) {
    CHECK(obstacle_is_clear(ok(30.0f), 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(ok(23.0f), 15.0f, 8.0f, 400.0f)); // must strictly exceed
    CHECK(!obstacle_is_clear(ok(16.0f), 15.0f, 8.0f, 400.0f));
    CHECK(obstacle_is_clear(st(RANGE_NO_ECHO), 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(st(RANGE_TOO_CLOSE), 15.0f, 8.0f, 400.0f));
    CHECK(!obstacle_is_clear(st(RANGE_FAULT), 15.0f, 8.0f, 400.0f));
}

static void coarse(scan_point_t out[5], range_reading_t r30, range_reading_t r60,
                   range_reading_t r90, range_reading_t r120, range_reading_t r150) {
    out[0] = (scan_point_t){30, r30};
    out[1] = (scan_point_t){60, r60};
    out[2] = (scan_point_t){90, r90};
    out[3] = (scan_point_t){120, r120};
    out[4] = (scan_point_t){150, r150};
}

static obstacle_profile_t symmetric_profile(bool lf, bool rf) {
    return obstacle_build_profile(12.0f, edge_at(&G, 100, 12.0f, lf), edge_at(&G, 80, 12.0f, rf));
}

static void choose_only_open_side(void) {
    scan_point_t c[5];
    obstacle_profile_t p = symmetric_profile(true, true);
    coarse(c, ok(10), ok(50), ok(12), ok(50), ok(100));
    side_choice_t ch = obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f);
    CHECK_INT(ch.side, SIDE_LEFT);
    CHECK_NEAR(ch.left_clear_cm, 100.0f, 1e-6);
    CHECK_NEAR(ch.right_clear_cm, 10.0f, 1e-6);

    coarse(c, ok(100), ok(50), ok(12), ok(50), ok(10));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_RIGHT);
}

static void choose_none_when_boxed_in(void) {
    scan_point_t c[5];
    obstacle_profile_t p = symmetric_profile(true, true);
    coarse(c, ok(24.9f), ok(50), ok(12), ok(50), ok(10));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_NONE);
}

static void choose_uses_outermost_reading(void) {
    // Inner points hit the obstacle itself; only the outer ones show lateral room.
    scan_point_t c[5];
    obstacle_profile_t p = symmetric_profile(true, true);
    coarse(c, ok(10), ok(12), ok(12), ok(12), ok(80));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);
}

static void choose_treats_statuses_conservatively(void) {
    scan_point_t c[5];
    obstacle_profile_t p = symmetric_profile(true, true);
    coarse(c, st(RANGE_NO_ECHO), ok(50), ok(12), ok(50), st(RANGE_TOO_CLOSE));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_RIGHT);
    coarse(c, st(RANGE_FAULT), ok(50), ok(12), ok(50), ok(60));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);
}

static void choose_prefers_found_edge(void) {
    scan_point_t c[5];
    coarse(c, ok(100), ok(50), ok(12), ok(50), ok(100));
    obstacle_profile_t p = symmetric_profile(false, true);
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_RIGHT);
    p = symmetric_profile(true, false);
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);
}

static void choose_shorter_detour(void) {
    scan_point_t c[5];
    coarse(c, ok(100), ok(50), ok(12), ok(50), ok(100));
    // Obstacle mostly on the right: left edge near center, right edge far out.
    obstacle_profile_t p = obstacle_build_profile(12.0f, edge_at(&G, 92.5f, 12.0f, true),
                                                  edge_at(&G, 57.5f, 12.0f, true));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);

    p = obstacle_build_profile(12.0f, edge_at(&G, 122.5f, 12.0f, true),
                               edge_at(&G, 87.5f, 12.0f, true));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_RIGHT);
}

static void choose_tie_breaks_on_clearance(void) {
    scan_point_t c[5];
    obstacle_profile_t p = symmetric_profile(true, true);
    coarse(c, ok(90), ok(50), ok(12), ok(50), ok(60));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_RIGHT);
    coarse(c, ok(60), ok(50), ok(12), ok(50), ok(60));
    CHECK_INT(obstacle_choose_side(&G, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);
}

static void choose_with_flipped_servo(void) {
    scan_point_t c[5];
    obstacle_profile_t p = obstacle_build_profile(12.0f, edge_at(&G_FLIPPED, 80, 12.0f, true),
                                                  edge_at(&G_FLIPPED, 100, 12.0f, true));
    // Flipped: 30 deg looks left, 150 deg looks right. Only 30 is open.
    coarse(c, ok(100), ok(50), ok(12), ok(50), ok(10));
    CHECK_INT(obstacle_choose_side(&G_FLIPPED, c, 5, &p, 25.0f, 400.0f).side, SIDE_LEFT);
}

void run_obstacle_tests(void) {
    RUN(side_classification);
    RUN(polar_to_xy);
    RUN(profile_geometry);
    RUN(clear_classification);
    RUN(choose_only_open_side);
    RUN(choose_none_when_boxed_in);
    RUN(choose_uses_outermost_reading);
    RUN(choose_treats_statuses_conservatively);
    RUN(choose_prefers_found_edge);
    RUN(choose_shorter_detour);
    RUN(choose_tie_breaks_on_clearance);
    RUN(choose_with_flipped_servo);
}
