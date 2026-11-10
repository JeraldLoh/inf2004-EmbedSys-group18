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
    result_t r = {0, 0, 0U, 0U, 0U, 0U, 0};
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
    return r;
}

/* One valid hump, height 10 +/- 2 mm, and (width_tol_mm > 0) a width of
   195 +/- width_tol_mm, also published as the length field in cm. */
static void expect_one_hump(const drive_t *d, const char *what, unsigned width_tol_mm)
{
    result_t r = drive(d);
    int width_err = (int)r.last_width_mm - (int)HUMP_W_MM;

    if (r.humps != 1 || r.valid_humps != 1 ||
        r.last_height_mm < 8U || r.last_height_mm > 12U ||
        r.final_terrain != TERRAIN_LEVEL ||
        ((width_tol_mm > 0U) &&
         ((r.last_width_mm == 0U) || (abs(width_err) > (int)width_tol_mm) ||
          (r.last_len_cm != (r.last_width_mm + 5U) / 10U))))
    {
        fprintf(stderr, "FAIL %s arm=%.0f front=%d v=%.0f shape=%d: humps=%d valid=%d h=%u "
                "width=%u len=%u terrain=%u\n", what, d->caster_mm, d->caster_front,
                d->speed_mm_s, d->shape, r.humps, r.valid_humps, r.last_height_mm,
                r.last_width_mm, r.last_len_cm, r.final_terrain);
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
    r = drive(&brake);
    assert(r.humps == 0);
    puts("PASS flat 5 min at 30 mg vibration and a 0.1 s stop: no hump episode");
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
    puts("PASS 1.5 deg floor tilt for 60 s: LEVEL, no valid false hump, no height drift");
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
    test_shared_and_payloads();
    test_config();
    test_one_sided_ramp();
    test_full_width_hump();
    test_flat_and_braking();
    test_tilted_floor();
    puts("Buddy 4 host tests: all passed");
    return 0;
}
