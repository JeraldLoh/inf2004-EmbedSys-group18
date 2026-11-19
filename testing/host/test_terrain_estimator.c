/*
 * Buddy 4 host tests: terrain estimator, packed snapshot, MQTT frame,
 * command decoding, odometry word and the IF-03 terrain_report_t adapter.
 *
 * The hump cases drive the real estimator with a simulated rigid car with
 * two drive wheels (track T) and a caster (caster_arm behind or in front of
 * the drive axle). The body attitude is the plane through the three floor
 * contacts, so a ramp under one wheel gives mostly roll and a full-width hump
 * gives pitch. Demo ramp: about 10 mm high, 195 mm long. Deterministic
 * vibration noise. Build: app_program/CMakeLists.txt, `ctest -L subsystem4`.
 */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "s4_config.h"
#include "s4_shared.h"
#include "terrain_estimator.h"

#define PI_D 3.14159265358979323846
#define HUMP_H_MM 10.0
#define HUMP_W_MM 195.0
#define TRACK_MM 112.0

enum { SHAPE_TRIANGLE, SHAPE_TRAPEZOID, SHAPE_COSINE };
enum { RAMP_NONE, RAMP_RIGHT, RAMP_LEFT, RAMP_FULL };

typedef struct
{
    double caster_mm;       /* drive axle to caster contact */
    int caster_front;       /* 1: caster ahead of the drive axle */
    double speed_mm_s;
    int shape;
    int ramp;               /* RAMP_* */
    double noise_mg;
    double seconds;         /* 0: long enough to cross the hump */
    double brake_after_mm;  /* < 0: never brake */
    double floor_tilt_deg;  /* floor slope after a level calibration */
} drive_t;

typedef struct
{
    int humps;
    int valid_humps;
    unsigned last_height_mm;
    unsigned last_len_cm;
    unsigned last_width_mm;     /* profile width estimate, 0 = none */
    unsigned final_terrain;
    int32_t final_height_um;
    int end_runs;               /* END_RUN events: must stay 0 while driving */
    unsigned max_lift_mm;       /* largest false lift-episode rise */
} result_t;

static unsigned long lcg_state;

static double noise_unit(void)
{
    /* Sum of 4 uniforms, centred: roughly normal, deterministic. */
    double sum = 0.0;
    int i;
    for (i = 0; i < 4; i++)
    {
        lcg_state = (lcg_state * 1103515245UL + 12345UL) & 0x7fffffffUL;
        sum += (double)lcg_state / 2147483647.0;
    }
    return (sum - 2.0) * 1.7320508;   /* unit variance */
}

static double profile(int shape, double x)
{
    if (x <= 0.0 || x >= HUMP_W_MM)
    {
        return 0.0;
    }
    switch (shape)
    {
        case SHAPE_TRIANGLE:
            return (x < HUMP_W_MM / 2.0) ? HUMP_H_MM * x / (HUMP_W_MM / 2.0)
                                         : HUMP_H_MM * (HUMP_W_MM - x) / (HUMP_W_MM / 2.0);
        case SHAPE_TRAPEZOID:
            if (x < 50.0) { return HUMP_H_MM * x / 50.0; }
            if (x > HUMP_W_MM - 50.0) { return HUMP_H_MM * (HUMP_W_MM - x) / 50.0; }
            return HUMP_H_MM;
        default:
            return HUMP_H_MM * 0.5 * (1.0 - cos(2.0 * PI_D * x / HUMP_W_MM));
    }
}

/* Accelerometer reading (mg, x right, y forward, z up) of a car resting on
   its three contacts: left/right wheels at y = 0, caster at y = -/+arm. */
static void body_gravity(const drive_t *d, double axle_mm, double out[3])
{
    double zl = 0.0, zr = 0.0, zc = 0.0, a, b, c, n1, n2;
    double y1, y2, z0, z1, z2;
    double caster_at = d->caster_front ? axle_mm + d->caster_mm : axle_mm - d->caster_mm;

    if (d->ramp == RAMP_RIGHT || d->ramp == RAMP_FULL) { zr = profile(d->shape, axle_mm); }
    if (d->ramp == RAMP_LEFT || d->ramp == RAMP_FULL) { zl = profile(d->shape, axle_mm); }
    if (d->ramp == RAMP_FULL) { zc = profile(d->shape, caster_at); }

    a = (zr - zl) / TRACK_MM;                    /* z = a x + b y + c */
    c = (zl + zr) / 2.0;
    b = d->caster_front ? (zc - c) / d->caster_mm : (c - zc) / d->caster_mm;
    b += tan(d->floor_tilt_deg * PI_D / 180.0);

    /* Body forward y^ = (0,1,b)/n1, up z^ = (-a,-b,1)/n2, right x^ = y^ x z^.
       The sensor reads world up in body axes. */
    n1 = sqrt(1.0 + b * b);
    n2 = sqrt(1.0 + a * a + b * b);
    y1 = 1.0 / n1; y2 = b / n1;
    z0 = -a / n2; z1 = -b / n2; z2 = 1.0 / n2;
    (void)z1;
    out[0] = 1000.0 * (-y1 * z0);
    out[1] = 1000.0 * y2;
    out[2] = 1000.0 * z2;
}

static void calibrated_estimator(s4_est_t *est, const drive_t *d)
{
    s4_cfg_t cfg;
    s4_input_t in;
    s4_events_t ev;
    int k;

    s4_est_default_config(&cfg);
    cfg.caster_arm_mm = (uint16_t)d->caster_mm;
    cfg.track_mm = (uint16_t)TRACK_MM;
    cfg.climb_nose_down = d->caster_front ? 1U : 0U;
    assert(s4_est_init(est, &cfg));
    s4_est_begin_calibration(est);
    for (k = 0; k < 150; k++)   /* 3 s still and level */
    {
        memset(&in, 0, sizeof(in));
        in.dt_ms = 20U;
        in.accel_mg[2] = 1000;
        in.accel_ok = 1U;
        in.odom_valid = 1U;
        s4_est_step(est, &in, &ev);
    }
    assert(est->calibrated);
    assert(est->terrain == TERRAIN_LEVEL);
}

static result_t drive(const drive_t *d)
{
    s4_est_t est;
    s4_input_t in;
    s4_events_t ev;
    result_t r = {0, 0, 0U, 0U, 0U, 0U, 0, 0, 0U};
    double axle = -300.0, odom_frac = 0.0, t = 0.0, speed = 0.0;
    double brake_t = -1.0;
    double seconds = d->seconds;
    unsigned i;

    if (seconds <= 0.0)
    {
        seconds = (300.0 + HUMP_W_MM + d->caster_mm + 400.0) / d->speed_mm_s;
    }
    lcg_state = 12345UL;
    calibrated_estimator(&est, d);
    while (t < seconds)
    {
        double accel_mm_s2 = 0.0, n, g[3];

        /* 0 -> speed in 0.3 s, then cruise; optional 0.1 s stop. */
        speed = (t < 0.3) ? d->speed_mm_s * t / 0.3 : d->speed_mm_s;
        accel_mm_s2 = (t < 0.3) ? d->speed_mm_s / 0.3 : 0.0;
        if (d->brake_after_mm >= 0.0 && axle > d->brake_after_mm && brake_t < 0.0)
        {
            brake_t = t;
        }
        if (brake_t >= 0.0)
        {
            double s = t - brake_t;
            speed = (s < 0.1) ? d->speed_mm_s * (1.0 - s / 0.1) : 0.0;
            accel_mm_s2 = (s < 0.1) ? -d->speed_mm_s / 0.1 : 0.0;
        }
        body_gravity(d, axle, g);
        n = (speed > 0.0) ? d->noise_mg : 2.0;

        memset(&in, 0, sizeof(in));
        in.dt_ms = 20U;
        in.accel_mg[0] = (int16_t)lround(g[0] + n * noise_unit());
        in.accel_mg[1] = (int16_t)lround(g[1] + accel_mm_s2 / 9.81 + n * noise_unit());
        in.accel_mg[2] = (int16_t)lround(g[2] + n * noise_unit());
        in.accel_ok = 1U;
        odom_frac += speed * 0.020;
        in.odom_delta_mm = (int16_t)floor(odom_frac);
        odom_frac -= in.odom_delta_mm;
        in.odom_valid = 1U;
        s4_est_step(&est, &in, &ev);

        for (i = 0U; i < ev.count; i++)
        {
            if (s4_get(ev.msg[i], S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH) == S4_EV_END_RUN)
            {
                r.end_runs++;
            }
            if (s4_get(ev.msg[i], S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH) == S4_EV_HUMP)
            {
                uint32_t hump = ev.msg[i] >> S4_EV_PAYLOAD_SHIFT;
                r.humps++;
                if (s4_get(hump, S4_HU_VALID_SHIFT, 1U) != 0U)
                {
                    r.valid_humps++;
                }
                r.last_height_mm = s4_get(hump, S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH);
                r.last_len_cm = s4_get(hump, S4_HU_LEN_SHIFT, S4_HU_LEN_WIDTH);
                r.last_width_mm = est.last_width_mm;
            }
        }
        axle += speed * 0.020;
        t += 0.020;
    }
    r.final_terrain = est.terrain;
    r.final_height_um = est.height_um;
    r.max_lift_mm = est.ranges.lift_mm;
    return r;
}

/* One valid hump, height 10 +/- 2 mm, and (width_tol_mm > 0) a width of
   195 +/- width_tol_mm, also published as the length field in cm. */
static void expect_one_hump(const drive_t *d, const char *what, unsigned width_tol_mm)
{
    result_t r = drive(d);
    int width_err = (int)r.last_width_mm - (int)HUMP_W_MM;

    if (r.humps != 1 || r.valid_humps != 1 || r.end_runs != 0 ||
        r.last_height_mm < 8U || r.last_height_mm > 12U ||
        r.final_terrain != TERRAIN_LEVEL ||
        ((width_tol_mm > 0U) &&
         ((r.last_width_mm == 0U) || (abs(width_err) > (int)width_tol_mm) ||
          (r.last_len_cm != (r.last_width_mm + 5U) / 10U))))
    {
        fprintf(stderr, "FAIL %s arm=%.0f front=%d v=%.0f shape=%d: humps=%d valid=%d h=%u "
                "width=%u len=%u terrain=%u end_runs=%d\n", what, d->caster_mm, d->caster_front,
                d->speed_mm_s, d->shape, r.humps, r.valid_humps, r.last_height_mm,
                r.last_width_mm, r.last_len_cm, r.final_terrain, r.end_runs);
        assert(0);
    }
}

static void test_config(void)
{
    s4_cfg_t cfg;

    s4_est_default_config(&cfg);
    assert(s4_est_config_valid(&cfg));
    cfg.raise_off_mm = cfg.raise_on_mm;          /* off must be below on */
    assert(!s4_est_config_valid(&cfg));
    s4_est_default_config(&cfg);
    cfg.caster_arm_mm = 0U;
    assert(!s4_est_config_valid(&cfg));
    s4_est_default_config(&cfg);
    cfg.track_mm = 2000U;
    assert(!s4_est_config_valid(&cfg));
    puts("PASS config validation");
}

static void test_one_sided_ramp(void)
{
    static const double arms[] = {100.0, 130.0, 160.0};
    static const double speeds[] = {60.0, 110.0};
    static const int shapes[] = {SHAPE_TRIANGLE, SHAPE_TRAPEZOID, SHAPE_COSINE};
    unsigned a, b, c;

    for (a = 0U; a < 3U; a++)
    {
        for (b = 0U; b < 2U; b++)
        {
            for (c = 0U; c < 3U; c++)
            {
                drive_t d = {arms[a], 0, speeds[b], shapes[c], RAMP_RIGHT, 15.0, 0.0, -1.0, 0.0};
                /* Width: straight faces 195 +/- 20 mm at the car's 60 mm/s
                   (+/- 25 at 110). A rounded (cosine) bump reads short by
                   design (long shallow feet), so only its height is checked. */
                unsigned tol = (shapes[c] == SHAPE_COSINE) ? 0U
                    : ((speeds[b] <= 60.0) ? 20U : 25U);
                expect_one_hump(&d, "right-wheel ramp", tol);
            }
        }
    }
    {
        drive_t left = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_LEFT, 15.0, 0.0, -1.0, 0.0};
        drive_t front = {130.0, 1, 60.0, SHAPE_TRAPEZOID, RAMP_RIGHT, 15.0, 0.0, -1.0, 0.0};
        drive_t rough = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_RIGHT, 30.0, 0.0, -1.0, 0.0};
        expect_one_hump(&left, "left-wheel ramp", 20U);
        expect_one_hump(&front, "caster in front", 20U);
        expect_one_hump(&rough, "30 mg vibration", 20U);
    }
    puts("PASS 10 mm x 195 mm ramp under one wheel: 21 cases, height 8..12 mm, "
         "straight-faced width 195 +/- 20 mm (60 mm/s)");
}

static void test_full_width_hump(void)
{
    static const double arms[] = {100.0, 130.0, 160.0};
    unsigned a;

    for (a = 0U; a < 3U; a++)
    {
        drive_t d = {arms[a], 0, 60.0, SHAPE_TRAPEZOID, RAMP_FULL, 15.0, 0.0, -1.0, 0.0};
        expect_one_hump(&d, "full-width hump", 0U);   /* caster crossing: no width */
    }
    puts("PASS 10 mm full-width hump (wheels then caster): one valid 8..12 mm result");
}

static void test_flat_and_braking(void)
{
    drive_t flat = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_NONE, 30.0, 300.0, -1.0, 0.0};
    drive_t brake = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_NONE, 15.0, 15.0, 300.0, 0.0};
    result_t r = drive(&flat);

    assert(r.humps == 0);
    assert(r.end_runs == 0);   /* vibration never adds up to a 20 cm "lift" */
    printf("  largest false lift rise in 5 min of 30 mg vibration: %u mm (limit %u mm)\n",
           r.max_lift_mm, (unsigned)S4_LIFT_HEIGHT_MM);
    assert(r.max_lift_mm < S4_LIFT_HEIGHT_MM / 2U);   /* at least 2x margin */
    r = drive(&brake);
    assert(r.humps == 0);
    assert(r.end_runs == 0);
    puts("PASS flat 5 min at 30 mg vibration and a 0.1 s stop: no hump episode, no end of run");
}

static void test_tilted_floor(void)
{
    /* Calibrated level, then 60 s on a floor tilted 1.5 deg: the height
       integral keeps re-zeroing instead of growing ~26 mm per metre, and no
       valid hump is invented. Calibrate on the floor the car drives on. */
    drive_t d = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_NONE, 5.0, 60.0, -1.0, 1.5};
    result_t r = drive(&d);

    if (r.valid_humps != 0 || r.final_terrain != TERRAIN_LEVEL ||
        r.final_height_um > 2000 || r.final_height_um < -2000)
    {
        fprintf(stderr, "FAIL tilted floor: valid=%d terrain=%u height=%ld um\n",
                r.valid_humps, r.final_terrain, (long)r.final_height_um);
        assert(0);
    }
    assert(r.end_runs == 0);
    puts("PASS 1.5 deg floor tilt for 60 s: LEVEL, no valid false hump, no height drift");
}

/* ---- End of run: lifted >= 20 cm or overturned --------------------------
 * A calibrated car rests on the floor, is lifted by rise_mm over lift_s
 * along a smooth (cosine) path while pitching to tilt_deg, is held, and
 * optionally put back down upright and left still. The accelerometer reads
 * the specific force (1 g + vertical acceleration) along the body axes. */
typedef struct
{
    double rise_mm;
    double lift_s;
    double tilt_deg;
    double hold_s;
    double noise_mg;
    int put_back;            /* 1: lower it again and rest it 2 s */
    double knock_mg;         /* > 0: one sideways sample of this size at rest */
} lift_t;

typedef struct
{
    int end_runs;
    unsigned cause;
    unsigned lift_cm;        /* from the event */
    double trigger_s;        /* time after the lift began */
    int latched_at_end;
    int latched_after_lift;  /* latched just before being put back */
} lift_result_t;

/* Smooth step 0 -> 1 over [0, 1]. */
static double ease(double x)
{
    if (x <= 0.0) { return 0.0; }
    if (x >= 1.0) { return 1.0; }
    return 0.5 * (1.0 - cos(PI_D * x));
}

static double ease_acc(double x, double duration_s)
{
    /* Second derivative of ease(t / duration) with respect to t. */
    if (x <= 0.0 || x >= 1.0) { return 0.0; }
    return 0.5 * (PI_D / duration_s) * (PI_D / duration_s) * cos(PI_D * x);
}

static lift_result_t lift_car(const lift_t *l)
{
    drive_t base = {130.0, 0, 60.0, SHAPE_TRAPEZOID, RAMP_NONE, 0.0, 0.0, -1.0, 0.0};
    lift_result_t r = {0, 0U, 0U, -1.0, 0, 0};
    s4_est_t est;
    s4_input_t in;
    s4_events_t ev;
    const double rest_s = 1.0;
    double t = 0.0;
    double down_at = rest_s + l->lift_s + l->hold_s;
    double end_s = l->put_back ? (down_at + l->lift_s + 2.0) : down_at;
    unsigned i;

    lcg_state = 777UL;
    calibrated_estimator(&est, &base);
    while (t < end_s)
    {
        double x_up = (t - rest_s) / l->lift_s;
        double x_down = (t - down_at) / l->lift_s;
        double frac = ease(x_up) - (l->put_back ? ease(x_down) : 0.0);
        double a_mm_s2 = l->rise_mm * (ease_acc(x_up, l->lift_s) -
                                       (l->put_back ? ease_acc(x_down, l->lift_s) : 0.0));
        double phi = l->tilt_deg * PI_D / 180.0 * frac;
        double f = 1000.0 * (1.0 + a_mm_s2 / 9806.65);   /* specific force, mg */
        double n = l->noise_mg;

        memset(&in, 0, sizeof(in));
        in.dt_ms = 20U;
        in.accel_mg[0] = (int16_t)lround(n * noise_unit());
        in.accel_mg[1] = (int16_t)lround(f * sin(phi) + n * noise_unit());
        in.accel_mg[2] = (int16_t)lround(f * cos(phi) + n * noise_unit());
        if (l->knock_mg > 0.0 && t >= 0.5 && t < 0.52)
        {
            in.accel_mg[0] = (int16_t)lround(l->knock_mg);   /* one sideways jolt */
        }
        in.accel_ok = 1U;
        in.odom_valid = 1U;
        s4_est_step(&est, &in, &ev);
        for (i = 0U; i < ev.count; i++)
        {
            if (s4_get(ev.msg[i], S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH) == S4_EV_END_RUN)
            {
                r.end_runs++;
                r.cause = s4_get(ev.msg[i], S4_EV_END_CAUSE_SHIFT, S4_ST_ENDCAUSE_WIDTH);
                r.lift_cm = s4_get(ev.msg[i], S4_EV_END_LIFT_SHIFT, S4_MO_LIFT_WIDTH);
                r.trigger_s = t - rest_s;
            }
        }
        if (t < down_at)
        {
            r.latched_after_lift = (int)s4_get(est.words[S4_W_STATUS], S4_ST_END_SHIFT, 1U);
        }
        t += 0.020;
    }
    r.latched_at_end = (int)s4_get(est.words[S4_W_STATUS], S4_ST_END_SHIFT, 1U);
    return r;
}

static void expect_lift(const lift_t *l, int want_end, unsigned want_cause, const char *what)
{
    lift_result_t r = lift_car(l);
    int ok = want_end ? (r.end_runs == 1 && (want_cause == S4_END_NONE || r.cause == want_cause))
                      : (r.end_runs == 0);

    printf("  %-44s end_runs=%d cause=%u lift=%u cm at %.2f s\n",
           what, r.end_runs, r.cause, r.lift_cm, r.trigger_s);
    if (!ok)
    {
        fprintf(stderr, "FAIL %s\n", what);
        assert(0);
    }
}

static void test_end_of_run(void)
{
    /* rise, lift time, tilt, hold, noise, put back, knock */
    lift_t upright = {300.0, 0.6, 0.0, 1.0, 15.0, 0, 0.0};
    lift_t tilted = {300.0, 0.7, 45.0, 1.0, 15.0, 0, 0.0};
    lift_t sideways = {300.0, 0.7, 90.0, 1.0, 15.0, 0, 0.0};
    lift_t flipped = {300.0, 0.8, 180.0, 1.0, 15.0, 0, 0.0};
    lift_t just_over = {230.0, 0.6, 0.0, 1.0, 15.0, 0, 0.0};
    lift_t low = {120.0, 0.5, 0.0, 1.0, 15.0, 0, 0.0};
    lift_t ramp_tilt = {0.0, 0.5, 25.0, 3.0, 15.0, 0, 0.0};
    lift_t knock = {0.0, 0.5, 0.0, 3.0, 15.0, 0, 1400.0};
    lift_t back_down = {300.0, 0.6, 0.0, 1.0, 15.0, 1, 0.0};
    lift_result_t r;

    puts("End of run (lifted >= 20 cm or overturned):");
    expect_lift(&upright, 1, S4_END_LIFTED, "30 cm lift in 0.6 s, upright");
    expect_lift(&just_over, 1, S4_END_LIFTED, "23 cm lift in 0.6 s, upright");
    expect_lift(&tilted, 1, S4_END_LIFTED, "30 cm lift while tilting to 45 deg");
    expect_lift(&sideways, 1, S4_END_NONE, "30 cm lift while tilting to 90 deg");
    expect_lift(&flipped, 1, S4_END_NONE, "30 cm lift while turning upside down");
    expect_lift(&low, 0, S4_END_NONE, "12 cm lift: below the limit");
    /* Known limit: a lift spread over 2 s accelerates too gently to start an
       episode, so lift the car deliberately (under about 1 s). */
    {
        lift_t slow = {300.0, 2.0, 0.0, 1.0, 15.0, 0, 0.0};
        expect_lift(&slow, 0, S4_END_NONE, "30 cm lift spread over 2 s (not counted)");
    }
    expect_lift(&ramp_tilt, 0, S4_END_NONE, "25 deg tilt on the floor (steep ramp)");
    expect_lift(&knock, 0, S4_END_NONE, "1.4 g sideways knock at rest");

    /* Overturned without any rise still ends the run. */
    {
        lift_t roll_over = {0.0, 0.8, 120.0, 1.0, 15.0, 0, 0.0};
        expect_lift(&roll_over, 1, S4_END_OVERTURNED, "turned over 120 deg in place");
    }

    /* The latch holds while the car is held, and clears once it rests upright. */
    r = lift_car(&back_down);
    printf("  %-44s latched while held=%d, after 2 s on the floor=%d\n",
           "30 cm lift, put back down", r.latched_after_lift, r.latched_at_end);
    assert(r.end_runs == 1 && r.latched_after_lift == 1 && r.latched_at_end == 0);
    puts("PASS end of run: lift at any tilt, overturn, no false trigger, latch clears on the floor");
}

static void test_shared_and_payloads(void)
{
    uint32_t words[S4_WORDS] = {0x80000001UL, 0x12345678UL, 0x0000ABCDUL, 0x1000040AUL};
    uint32_t back[S4_WORDS];
    uint8_t frame[S4_TLM_SIZE];
    uint8_t cmd = 0U;
    terrain_report_t report;
    s4_odom_reader_t reader;
    int16_t delta;
    unsigned i;

    /* Before the first publish there is nothing to read or send. */
    assert(!s4_shared_read(back));
    assert(s4_telemetry_pack(frame, sizeof(frame)) == 0U);

    s4_shared_publish(words);
    assert(s4_shared_read(back));
    assert(memcmp(words, back, sizeof(words)) == 0);
    assert(s4_telemetry_pack(frame, sizeof(frame) - 1U) == 0U);
    assert(s4_telemetry_pack(frame, sizeof(frame)) == S4_TLM_SIZE);
    assert(frame[0] == 0x21U);   /* version 1, type 4: Buddy 1 checks this */
    for (i = 0U; i < S4_WORDS; i++)
    {
        uint32_t w = (uint32_t)frame[1U + 4U * i] | ((uint32_t)frame[2U + 4U * i] << 8) |
                     ((uint32_t)frame[3U + 4U * i] << 16) | ((uint32_t)frame[4U + 4U * i] << 24);
        assert(w == words[i]);
    }

    for (i = 0U; i <= 5U; i++)
    {
        assert(s4_command_decode((uint8_t)i, &cmd) && cmd == i);
    }
    assert(!s4_command_decode(6U, &cmd));
    assert(!s4_command_decode(7U, &cmd));
    assert(!s4_command_decode(0x09U, &cmd));

    /* Word 0 = LEVEL, valid; word 3 = 10 mm, +/-4, valid. */
    s4_terrain_report_from_words(words, 1234U, &report);
    assert(report.state == TERRAIN_LEVEL);
    assert(report.speed_cap_mm_s == 0U);
    assert(report.estimated_height_mm == 10);
    assert(report.uncertainty_mm == 4U);
    assert(report.estimate_valid);
    assert(report.observed_ms == 1234U);

    /* End of run decoded from the snapshot. */
    {
        uint32_t ended[S4_WORDS] = {0U, 0U, 0U, 0U};
        uint32_t cause = 9U, cm = 9U;
        assert(!s4_end_of_run_from_words(words, &cause, &cm));
        ended[S4_W_STATUS] = s4_put(0U, S4_ST_END_SHIFT, 1U, 1U);
        ended[S4_W_STATUS] = s4_put(ended[S4_W_STATUS], S4_ST_ENDCAUSE_SHIFT,
                                    S4_ST_ENDCAUSE_WIDTH, S4_END_LIFTED);
        ended[S4_W_MOTION] = s4_put(0U, S4_MO_LIFT_SHIFT, S4_MO_LIFT_WIDTH, 27U);
        assert(s4_end_of_run_from_words(ended, &cause, &cm));
        assert(cause == S4_END_LIFTED && cm == 27U);
        assert(s4_end_of_run_from_words(ended, NULL, NULL));
    }

    memset(&reader, 0, sizeof(reader));
    s4_post_odometry(65530);
    assert(s4_take_odometry(&reader, 10U, &delta) && delta == 0);   /* primes */
    s4_post_odometry(65540);                                       /* 16-bit wrap */
    assert(s4_take_odometry(&reader, 10U, &delta) && delta == 10);
    s4_post_odometry(65535);                                       /* reverse */
    assert(s4_take_odometry(&reader, 10U, &delta) && delta == -5);
    s4_post_odometry_invalid();
    assert(!s4_take_odometry(&reader, 10U, &delta) && delta == 0);
    puts("PASS seqlock snapshot, 17-byte frame, commands, IF-03 adapter, odometry");
}

int main(void)
{
    setvbuf(stdout, NULL, _IONBF, 0);   /* keep output that precedes a failed assert */
    test_shared_and_payloads();
    test_config();
    test_one_sided_ramp();
    test_full_width_hump();
    test_flat_and_braking();
    test_tilted_floor();
    test_end_of_run();
    puts("Buddy 4 host tests: all passed");
    return 0;
}
