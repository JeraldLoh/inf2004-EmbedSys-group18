/*
 * Host checks for Buddy 4 (subsystem4) portable code: fixed-point IMU math,
 * minimum-bit packing, seqlock snapshot, odometry word, MQTT frame, and the
 * terrain estimator driven by a simulated car. No kernel, no hardware.
 */
#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "../../app_program/subsystem4/imu_math.h"
#include "../../app_program/subsystem4/s4_packed.h"
#include "../../app_program/subsystem4/s4_shared.h"
#include "../../app_program/subsystem4/terrain_estimator.h"

#define PI_D 3.14159265358979323846
#define DEG(rad) ((rad) * 180.0 / PI_D)
#define RAD(deg) ((deg) * PI_D / 180.0)
#define DT_MS 20U

/* ------------------------------------------------------------------ */
/*  Simulated car                                                      */
/* ------------------------------------------------------------------ */

typedef struct
{
    double mount_tilt_rad;   /* sensor pitched up relative to the car */
    double heading_rad;      /* clockwise from magnetic north */
    double path_mm;          /* odometry truth */
    long posted_mm;
    unsigned noise_mg;
    uint32_t rng;
} car_t;

typedef struct
{
    unsigned terrain_seen;   /* bit per terrain_state_t visited */
    unsigned max_cap_units;
    int humps;
    uint32_t last_hump;      /* hump event payload (word bits 0..28) */
    int collisions;
    uint32_t last_collision;
    int turns;
    uint32_t last_turn;
    int cals;
    uint32_t last_cal;
    int faults;
} sim_log_t;

static int noise(car_t *car)
{
    if (car->noise_mg == 0U)
    {
        return 0;
    }
    car->rng = (car->rng * 1103515245U) + 12345U;
    return (int)((car->rng >> 16) % (2U * car->noise_mg + 1U)) - (int)car->noise_mg;
}

/* Rotate a vehicle-frame vector into the tilted sensor frame. */
static void to_sensor(const car_t *car, const double v[3], double s[3])
{
    double c = cos(car->mount_tilt_rad);
    double n = sin(car->mount_tilt_rad);

    s[0] = v[0];                     /* X = right */
    s[1] = (v[1] * c) + (v[2] * n);  /* Y = forward */
    s[2] = (-v[1] * n) + (v[2] * c); /* Z = up */
}

static s4_input_t car_sample(
    car_t *car,
    double pitch_rad,
    double extra_fwd_mg,
    double extra_right_mg,
    bool odom_valid,
    int16_t odom_delta)
{
    /* Earth field (east, north, up) in raw magnetometer LSB. */
    const double b_east = 0.0;
    const double b_north = 220.0;
    const double b_up = -420.0;
    double sp = sin(pitch_rad);
    double cp = cos(pitch_rad);
    double sh = sin(car->heading_rad);
    double ch = cos(car->heading_rad);
    double right[3] = {ch, -sh, 0.0};
    double fwd[3] = {sh * cp, ch * cp, sp};
    double up[3] = {-sh * sp, -ch * sp, cp};
    double accel_v[3];
    double mag_v[3];
    double accel_s[3];
    double mag_s[3];
    s4_input_t in = {0};
    unsigned axis;

    accel_v[0] = extra_right_mg;
    accel_v[1] = (1000.0 * sp) + extra_fwd_mg;
    accel_v[2] = 1000.0 * cp;
    mag_v[0] = (b_east * right[0]) + (b_north * right[1]) + (b_up * right[2]);
    mag_v[1] = (b_east * fwd[0]) + (b_north * fwd[1]) + (b_up * fwd[2]);
    mag_v[2] = (b_east * up[0]) + (b_north * up[1]) + (b_up * up[2]);
    to_sensor(car, accel_v, accel_s);
    to_sensor(car, mag_v, mag_s);

    for (axis = 0U; axis < 3U; axis++)
    {
        in.accel_mg[axis] = (int16_t)lround(accel_s[axis] + noise(car));
        in.mag[axis] = (int16_t)lround(mag_s[axis]);
    }
    in.dt_ms = DT_MS;
    in.accel_ok = 1U;
    in.mag_ok = 1U;
    in.odom_valid = odom_valid ? 1U : 0U;
    in.odom_delta_mm = odom_delta;
    return in;
}

static void log_step(s4_est_t *est, const s4_input_t *in, sim_log_t *log)
{
    s4_events_t events;
    uint8_t i;
    unsigned cap;

    s4_est_step(est, in, &events);
    log->terrain_seen |= 1U << est->terrain;
    cap = s4_get(est->words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH);
    if (cap > log->max_cap_units)
    {
        log->max_cap_units = cap;
    }
    for (i = 0U; i < events.count; i++)
    {
        uint32_t msg = events.msg[i];
        uint32_t payload = msg >> S4_EV_PAYLOAD_SHIFT;

        switch (s4_get(msg, S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH))
        {
            case S4_EV_HUMP:
                log->humps++;
                log->last_hump = payload;
                break;
            case S4_EV_COLLISION:
                log->collisions++;
                log->last_collision = msg;
                break;
            case S4_EV_TURN:
                log->turns++;
                log->last_turn = msg;
                break;
            case S4_EV_CAL:
                log->cals++;
                log->last_cal = msg;
                break;
            case S4_EV_FAULT:
                log->faults++;
                break;
            default:
                break;
        }
    }
}

/* Drive straight on level ground for n samples at speed_mm_s. */
static void drive_level(
    s4_est_t *est, car_t *car, sim_log_t *log, unsigned n,
    double speed_mm_s, bool odom)
{
    unsigned i;

    for (i = 0U; i < n; i++)
    {
        long now_mm;
        s4_input_t in;

        car->path_mm += speed_mm_s * (DT_MS / 1000.0);
        now_mm = lround(car->path_mm);
        in = car_sample(car, 0.0, 0.0, 0.0, odom, (int16_t)(now_mm - car->posted_mm));
        car->posted_mm = now_mm;
        log_step(est, &in, log);
    }
}

/*
 * Raised-cosine hump: h(x) = H/2 (1 - cos(2 pi x / L)), 0 <= x <= L.
 * The car moves at speed_mm_s along the surface; odometry reports path length.
 */
static void drive_hump(
    s4_est_t *est, car_t *car, sim_log_t *log,
    double height_mm, double length_mm, double speed_mm_s, bool odom)
{
    double x = 0.0;

    while (x < length_mm)
    {
        double slope = (height_mm * PI_D / length_mm) * sin(2.0 * PI_D * x / length_mm);
        double ds = speed_mm_s * (DT_MS / 1000.0);
        long now_mm;
        s4_input_t in;

        x += ds / sqrt(1.0 + (slope * slope));
        car->path_mm += ds;
        now_mm = lround(car->path_mm);
        in = car_sample(car, atan(slope), 0.0, 0.0, odom,
                        (int16_t)(now_mm - car->posted_mm));
        car->posted_mm = now_mm;
        log_step(est, &in, log);
    }
}

static void calibrated_estimator(s4_est_t *est, car_t *car, sim_log_t *log)
{
    s4_cfg_t cfg;
    sim_log_t empty = {0};

    s4_est_default_config(&cfg);
    assert(s4_est_init(est, &cfg));
    s4_est_begin_calibration(est);
    car->mount_tilt_rad = RAD(2.0);
    car->noise_mg = 6U;
    car->rng = 1U;
    *log = empty;
    drive_level(est, car, log, 80U, 0.0, true);
    assert(log->cals == 1);
    assert(s4_get(log->last_cal, S4_EV_CAL_OK_SHIFT, 1U) == 1U);
    assert(est->calibrated);
    assert(est->terrain == TERRAIN_LEVEL);
    *log = empty;
}

/* ------------------------------------------------------------------ */
/*  Math                                                               */
/* ------------------------------------------------------------------ */

static void test_atan2_accuracy(void)
{
    double worst = 0.0;
    int deg10;

    for (deg10 = -1799; deg10 <= 1800; deg10 += 7)
    {
        double angle = RAD(deg10 / 10.0);
        int32_t y = (int32_t)lround(10000.0 * sin(angle));
        int32_t x = (int32_t)lround(10000.0 * cos(angle));
        double got = imu_atan2_ddeg(y, x) / 10.0;
        double want = DEG(atan2((double)y, (double)x));
        double err = fabs(got - want);

        if (err > 180.0)
        {
            err = 360.0 - err;
        }
        if (err > worst)
        {
            worst = err;
        }
    }
    printf("  atan2 worst error %.3f deg\n", worst);
    assert(worst <= 0.15);
    assert(imu_atan2_ddeg(0, 0) == 0);
    assert(imu_atan2_ddeg(INT32_MIN, 1) == -900);
}

static void test_isqrt_and_sin(void)
{
    int a;

    assert(imu_isqrt_u64(0U) == 0U);
    assert(imu_isqrt_u64(15U) == 3U);
    assert(imu_isqrt_u64(16U) == 4U);
    assert(imu_isqrt_u64(1000000000000ULL) == 1000000U);
    assert(imu_isqrt_u64(UINT64_MAX) == 0xFFFFFFFFU);

    for (a = -900; a <= 900; a++)
    {
        double want = 32767.0 * sin(RAD(a / 10.0));
        assert(fabs(imu_sin_q15(a) - want) <= 2.0);
    }
    assert(imu_sin_q15(1200) == 32767);
    assert(imu_wrap_ddeg(1900) == -1700);
    assert(imu_wrap_ddeg(-1900) == 1700);
    assert(imu_wrap_ddeg(3600) == 0);
}

static void test_decode_and_axes(void)
{
    /* +1000 mg, -1000 mg, +2000 mg at +/-4 g (2 mg/LSB). */
    const uint8_t accel[6] = {0x40U, 0x1FU, 0xC0U, 0xE0U, 0x80U, 0x3EU};
    const uint8_t clip[6] = {0xF0U, 0x7FU, 0x00U, 0x00U, 0x00U, 0x00U};
    /* X = 300, Z = 980 (-> 1100 after gain match), Y = -200. Order X, Z, Y. */
    const uint8_t mag[6] = {0x01U, 0x2CU, 0x03U, 0xD4U, 0xFFU, 0x38U};
    const uint8_t mag_ovf[6] = {0xF0U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U};
    int16_t out[3];
    int32_t sensor[3] = {10, 20, 30};
    int32_t vehicle[3];
    bool flag = true;

    assert(imu_decode_accel_mg(accel, IMU_ACCEL_FS_4G, out, &flag));
    assert((out[0] == 1000) && (out[1] == -1000) && (out[2] == 2000) && !flag);
    assert(imu_decode_accel_mg(clip, IMU_ACCEL_FS_4G, out, &flag) && flag);
    assert(imu_decode_mag(mag, out, &flag));
    assert((out[0] == 300) && (out[1] == -200) && (out[2] == 1100) && !flag);
    assert(imu_decode_mag(mag_ovf, out, &flag) && flag);
    assert(!imu_decode_accel_mg(NULL, 0U, out, &flag));

    assert(imu_axis_codes_valid(IMU_AXIS_CODE(0U, 0), IMU_AXIS_CODE(1U, 0),
                                IMU_AXIS_CODE(2U, 0)));
    assert(!imu_axis_codes_valid(IMU_AXIS_CODE(0U, 0), IMU_AXIS_CODE(0U, 1),
                                 IMU_AXIS_CODE(2U, 0)));
    assert(!imu_axis_codes_valid(3U, 1U, 2U));

    /* Board mounted rotated: sensor -Y is right, X forward, Z up. */
    imu_map_axes(sensor, IMU_AXIS_CODE(1U, 1), IMU_AXIS_CODE(0U, 0),
                 IMU_AXIS_CODE(2U, 0), vehicle);
    assert((vehicle[IMU_RIGHT] == -20) && (vehicle[IMU_FWD] == 10) &&
           (vehicle[IMU_UP] == 30));
}

static void test_heading(void)
{
    car_t car = {0};
    double worst = 0.0;
    int heading_deg;
    int pitch_deg;

    for (pitch_deg = -15; pitch_deg <= 15; pitch_deg += 15)
    {
        for (heading_deg = 0; heading_deg < 360; heading_deg += 10)
        {
            s4_input_t in;
            int32_t a[3];
            int32_t m[3];
            uint16_t got;
            double err;
            unsigned axis;

            car.heading_rad = RAD(heading_deg);
            in = car_sample(&car, RAD(pitch_deg), 0.0, 0.0, false, 0);
            for (axis = 0U; axis < 3U; axis++)
            {
                a[axis] = in.accel_mg[axis];
                m[axis] = in.mag[axis];
            }
            assert(imu_heading_ddeg(a, m, &got));
            err = fabs((got / 10.0) - heading_deg);
            if (err > 180.0)
            {
                err = 360.0 - err;
            }
            if (err > worst)
            {
                worst = err;
            }
        }
    }
    printf("  heading worst error %.2f deg (pitch -15..15)\n", worst);
    assert(worst <= 1.0);
}

/* ------------------------------------------------------------------ */
/*  Packing, shared data and payloads                                  */
/* ------------------------------------------------------------------ */

static void test_packing(void)
{
    uint32_t w = 0U;

    w = s4_put(w, S4_AT_PITCH_SHIFT, S4_AT_PITCH_WIDTH, (uint32_t)-180);
    w = s4_put(w, S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH, 179U);
    w = s4_put(w, S4_AT_HEADING_SHIFT, S4_AT_HEADING_WIDTH, 359U);
    w = s4_put(w, S4_AT_SEQ_SHIFT, S4_AT_SEQ_WIDTH, 31U);
    assert(s4_get_signed(w, S4_AT_PITCH_SHIFT, S4_AT_PITCH_WIDTH) == -180);
    assert(s4_get_signed(w, S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH) == 179);
    assert(s4_get(w, S4_AT_HEADING_SHIFT, S4_AT_HEADING_WIDTH) == 359U);
    assert(s4_get(w, S4_AT_SEQ_SHIFT, S4_AT_SEQ_WIDTH) == 31U);

    /* Writing one field never disturbs its neighbours. */
    w = s4_put(w, S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH, (uint32_t)-1);
    assert(s4_get_signed(w, S4_AT_PITCH_SHIFT, S4_AT_PITCH_WIDTH) == -180);
    assert(s4_get_signed(w, S4_AT_ROLL_SHIFT, S4_AT_ROLL_WIDTH) == -1);
    assert(s4_get(w, S4_AT_HEADING_SHIFT, S4_AT_HEADING_WIDTH) == 359U);

    /* Values wider than the field are truncated to it, not spilled. */
    assert(s4_put(0U, S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH, 0xFFU) == 0x7U);
    assert(s4_sext(0x1FFU, 9U) == -1);
    assert(s4_sext(0x0FFU, 9U) == 255);
}

static void test_shared_and_payloads(void)
{
    uint32_t words[S4_WORDS] = {0x11223344U, 0x55667788U, 0x99AABBCCU, 0xDDEEFF00U};
    uint32_t copy[S4_WORDS];
    uint8_t frame[S4_TLM_SIZE];
    uint8_t command;
    s4_odom_reader_t reader = {0};
    terrain_report_t report;
    uint32_t status = 0U;
    uint32_t hump = 0U;
    int16_t delta;
    int i;

    /* Nothing published yet. */
    assert(!s4_shared_read(copy));
    assert(s4_telemetry_pack(frame, sizeof(frame)) == 0U);

    s4_shared_publish(words);
    assert(s4_shared_read(copy));
    for (i = 0; i < (int)S4_WORDS; i++)
    {
        assert(copy[i] == words[i]);
    }

    assert(s4_telemetry_pack(frame, sizeof(frame) - 1U) == 0U);
    assert(s4_telemetry_pack(frame, sizeof(frame)) == 17U);
    assert(frame[0] == (S4_TLM_VERSION | (S4_TLM_TYPE << 3)));
    assert((frame[1] == 0x44U) && (frame[4] == 0x11U) && (frame[16] == 0xDDU));

    assert(s4_command_decode(S4_CMD_RECALIBRATE, &command) &&
           (command == S4_CMD_RECALIBRATE));
    assert(s4_command_decode(S4_CMD_NONE, &command) && (command == S4_CMD_NONE));
    assert(s4_command_decode(S4_CMD_REPORT_RANGES, &command) &&
           (command == S4_CMD_REPORT_RANGES));
    assert(!s4_command_decode(6U, &command));
    assert(!s4_command_decode(0x09U, &command));

    /* Odometry: first read primes, then modular deltas across the wrap. */
    assert(!s4_take_odometry(&reader, 10U, &delta));
    s4_post_odometry(65530);
    assert(s4_take_odometry(&reader, 10U, &delta) && (delta == 0));
    s4_post_odometry(65540);
    assert(s4_take_odometry(&reader, 10U, &delta) && (delta == 10));
    s4_post_odometry(65500);
    assert(s4_take_odometry(&reader, 10U, &delta) && (delta == -40));
    for (i = 0; i < 9; i++)
    {
        assert(s4_take_odometry(&reader, 10U, &delta) && (delta == 0));
    }
    assert(!s4_take_odometry(&reader, 10U, &delta));   /* stale */
    s4_post_odometry(65503);
    assert(s4_take_odometry(&reader, 10U, &delta) && (delta == 3));
    s4_post_odometry_invalid();
    assert(!s4_take_odometry(&reader, 10U, &delta));

    /* Contract adapter. */
    status = s4_put(status, S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH, TERRAIN_CREST);
    status = s4_put(status, S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH, 15U);
    hump = s4_put(hump, S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH, 42U);
    hump = s4_put(hump, S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH, 6U);
    hump = s4_put(hump, S4_HU_VALID_SHIFT, 1U, 1U);
    words[S4_W_STATUS] = status;
    words[S4_W_HUMP] = hump;
    s4_terrain_report_from_words(words, 1234U, &report);
    assert(report.state == TERRAIN_CREST);
    assert(report.speed_cap_mm_s == 150U);
    assert(report.estimated_height_mm == 42);
    assert(report.uncertainty_mm == 6U);
    assert(report.estimate_valid);
    assert(report.observed_ms == 1234U);
}

/* ------------------------------------------------------------------ */
/*  Estimator scenarios                                                */
/* ------------------------------------------------------------------ */

static void test_config_validation(void)
{
    s4_cfg_t cfg;
    s4_est_t est;

    s4_est_default_config(&cfg);
    assert(s4_est_config_valid(&cfg));
    cfg.ascend_ddeg = cfg.level_ddeg;
    assert(!s4_est_config_valid(&cfg));
    assert(!s4_est_init(&est, &cfg));
    s4_est_default_config(&cfg);
    cfg.axis_fwd = cfg.axis_right;
    assert(!s4_est_config_valid(&cfg));
}

static void test_calibration_rejects_motion(void)
{
    s4_cfg_t cfg;
    s4_est_t est;
    car_t car = {0};
    sim_log_t log = {0};

    s4_est_default_config(&cfg);
    assert(s4_est_init(&est, &cfg));
    s4_est_begin_calibration(&est);
    car.noise_mg = 120U;    /* being carried around, not still */
    car.rng = 7U;
    drive_level(&est, &car, &log, 200U, 0.0, true);
    assert(log.cals == 1);
    assert(s4_get(log.last_cal, S4_EV_CAL_OK_SHIFT, 1U) == 0U);
    assert(!est.calibrated);
    assert((est.faults & S4_FAULT_UNCAL) != 0U);
    assert(est.terrain == TERRAIN_UNKNOWN);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) == 10U);
}

static void test_level_after_calibration(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    int32_t pitch_half;

    calibrated_estimator(&est, &car, &log);
    drive_level(&est, &car, &log, 100U, 150.0, true);
    pitch_half = s4_get_signed(est.words[S4_W_ATTITUDE], S4_AT_PITCH_SHIFT,
                               S4_AT_PITCH_WIDTH);
    /* 2 deg mounting tilt removed by calibration. */
    assert((pitch_half >= -1) && (pitch_half <= 1));
    assert(est.terrain == TERRAIN_LEVEL);
    assert(log.humps == 0);
    assert(log.max_cap_units == 0U);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH) !=
           S4_MC_IMPACT);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_ODOM_SHIFT, 1U) == 1U);
}

static void check_hump(const sim_log_t *log, double height_mm, double tolerance_mm,
                       const char *label)
{
    uint32_t h = s4_get(log->last_hump, S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH);
    uint32_t unc = s4_get(log->last_hump, S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH);
    uint32_t pitch = s4_get(log->last_hump, S4_HU_PITCH_SHIFT, S4_HU_PITCH_WIDTH);
    uint32_t len = s4_get(log->last_hump, S4_HU_LEN_SHIFT, S4_HU_LEN_WIDTH);
    uint32_t valid = s4_get(log->last_hump, S4_HU_VALID_SHIFT, 1U);

    printf("  %s: height %u mm (true %.0f) +/-%u, peak pitch %u deg, %u cm, %s\n",
           label, (unsigned)h, height_mm, (unsigned)unc, (unsigned)pitch,
           (unsigned)len, valid ? "valid" : "invalid");
    assert(log->humps == 1);
    assert(valid == 1U);
    assert(fabs((double)h - height_mm) <= tolerance_mm);
    assert(fabs((double)h - height_mm) <= (double)unc + 1.0);
}

static void test_hump_with_odometry(void)
{
    const unsigned visited = (1U << TERRAIN_LEVEL) | (1U << TERRAIN_ASCENDING) |
        (1U << TERRAIN_CREST) | (1U << TERRAIN_DESCENDING) | (1U << TERRAIN_SETTLING);
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;

    calibrated_estimator(&est, &car, &log);
    drive_level(&est, &car, &log, 50U, 150.0, true);
    drive_hump(&est, &car, &log, 40.0, 400.0, 150.0, true);
    drive_level(&est, &car, &log, 60U, 150.0, true);

    assert((log.terrain_seen & visited) == visited);
    assert(log.max_cap_units == 15U);          /* 150 mm/s on the hump */
    assert(est.terrain == TERRAIN_LEVEL);
    check_hump(&log, 40.0, 4.0, "40 mm hump, odometry");
    assert(est.words[S4_W_HUMP] != 0U);
}

static void test_small_and_large_humps(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;

    calibrated_estimator(&est, &car, &log);
    drive_level(&est, &car, &log, 50U, 120.0, true);
    drive_hump(&est, &car, &log, 20.0, 300.0, 120.0, true);
    drive_level(&est, &car, &log, 60U, 120.0, true);
    check_hump(&log, 20.0, 3.0, "20 mm hump, odometry");

    calibrated_estimator(&est, &car, &log);
    drive_level(&est, &car, &log, 50U, 100.0, true);
    drive_hump(&est, &car, &log, 70.0, 500.0, 100.0, true);
    drive_level(&est, &car, &log, 60U, 100.0, true);
    check_hump(&log, 70.0, 6.0, "70 mm hump, odometry");
}

static void test_hump_without_odometry(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    uint32_t unc;

    calibrated_estimator(&est, &car, &log);
    car.noise_mg = 20U;   /* motors running: vibration marks it as moving */
    drive_level(&est, &car, &log, 50U, 150.0, false);
    drive_hump(&est, &car, &log, 40.0, 400.0, 150.0, false);
    drive_level(&est, &car, &log, 60U, 150.0, false);
    check_hump(&log, 40.0, 8.0, "40 mm hump, NO odometry");
    unc = s4_get(log.last_hump, S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH);
    assert(unc >= 20U);   /* fallback distance widens the uncertainty */
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_ODOM_SHIFT, 1U) == 0U);
}

static void test_collision(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    s4_input_t in;

    calibrated_estimator(&est, &car, &log);
    drive_level(&est, &car, &log, 50U, 150.0, true);

    /* Head-on hit: strong deceleration on one sample. */
    in = car_sample(&car, 0.0, -2200.0, 150.0, true, 3);
    log_step(&est, &in, &log);
    assert(log.collisions == 1);
    assert(s4_get(log.last_collision, S4_EV_COLL_DIR_SHIFT, S4_ST_COLLDIR_WIDTH) ==
           S4_HIT_FRONT);
    assert(s4_get(log.last_collision, S4_EV_COLL_MAG_SHIFT, S4_MO_SHOCK_WIDTH) >=
           2000U / S4_SHOCK_UNIT_MG);
    assert(est.motion == S4_MOTION_IMPACT);
    assert((est.faults & S4_FAULT_COLLISION) != 0U);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) == 10U);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_COLLCNT_SHIFT, S4_ST_COLLCNT_WIDTH) == 1U);

    /* The jolt must not start a false hump. */
    drive_level(&est, &car, &log, 150U, 150.0, true);
    assert(log.humps == 0);
    assert(log.collisions == 1);
    assert(est.terrain == TERRAIN_LEVEL);
    assert(est.motion != S4_MOTION_IMPACT);
    assert((est.faults & S4_FAULT_COLLISION) != 0U);   /* latched */
    s4_est_clear_latched(&est);
    assert((est.faults & S4_FAULT_COLLISION) == 0U);

    /* Side hit from the left pushes the car right. */
    in = car_sample(&car, 0.0, 0.0, 1800.0, true, 3);
    log_step(&est, &in, &log);
    assert(log.collisions == 2);
    assert(s4_get(log.last_collision, S4_EV_COLL_DIR_SHIFT, S4_ST_COLLDIR_WIDTH) ==
           S4_HIT_LEFT);
}

static void test_turn(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    unsigned i;
    int32_t rate_dps = 0;
    uint32_t angle;

    calibrated_estimator(&est, &car, &log);
    car.heading_rad = 0.0;
    drive_level(&est, &car, &log, 20U, 0.0, true);

    /* Pivot right at 90 deg/s for 1 s. */
    for (i = 0U; i < 50U; i++)
    {
        s4_input_t in;

        car.heading_rad += RAD(90.0 * DT_MS / 1000.0);
        in = car_sample(&car, 0.0, 0.0, 0.0, true, 0);
        log_step(&est, &in, &log);
        if (i == 40U)
        {
            /* 8-bit field in 2 deg/s units (tribunal decision). */
            rate_dps = s4_get_signed(est.words[S4_W_MOTION], S4_MO_TURNRATE_SHIFT,
                                     S4_MO_TURNRATE_WIDTH) * (int32_t)S4_TURNRATE_UNIT_DPS;
            assert(s4_get(est.words[S4_W_STATUS], S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH) ==
                   S4_MC_TURN_RIGHT);
        }
    }
    drive_level(&est, &car, &log, 30U, 0.0, true);

    angle = s4_get(log.last_turn, S4_EV_TURN_ANGLE_SHIFT, S4_EV_TURN_ANGLE_WIDTH);
    printf("  turn: rate %d deg/s mid-turn, event right %u deg (true 90)\n",
           (int)rate_dps, (unsigned)angle);
    assert((rate_dps >= 80) && (rate_dps <= 100));
    assert(log.turns == 1);
    assert(s4_get(log.last_turn, S4_EV_TURN_RIGHT_SHIFT, 1U) == 1U);
    assert((angle >= 82U) && (angle <= 98U));
    /* Heading is meaningful: message valid and no MAG fault. */
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_VALID_SHIFT, 1U) == 1U);
    assert((s4_get(est.words[S4_W_STATUS], S4_ST_FAULT_SHIFT, S4_ST_FAULT_WIDTH) &
            S4_FAULT_MAG) == 0U);
    assert(abs((int)s4_get(est.words[S4_W_ATTITUDE], S4_AT_HEADING_SHIFT,
                           S4_AT_HEADING_WIDTH) - 90) <= 2);
}

/* Week 8 tribunal decisions: validity bit, flags, motion class, cal status,
   turn-rate width with explicit clamping, and the observed-range log. */
static void test_tribunal_fields(void)
{
    s4_cfg_t cfg;
    s4_est_t est;
    car_t car = {0};
    sim_log_t log = {0};
    s4_input_t in;
    s4_input_t lost = {0};
    s4_events_t events = {0};
    bool hump_flag_seen = false;
    uint32_t status;
    unsigned i;

    /* Calibration status: RUNNING while calibrating, data not yet valid. */
    s4_est_default_config(&cfg);
    assert(s4_est_init(&est, &cfg));
    s4_est_begin_calibration(&est);
    car.noise_mg = 4U;
    car.rng = 3U;
    drive_level(&est, &car, &log, 10U, 0.0, true);
    status = est.words[S4_W_STATUS];
    assert(s4_get(status, S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH) == S4_CAL_RUNNING);
    assert(s4_get(status, S4_ST_VALID_SHIFT, 1U) == 0U);

    drive_level(&est, &car, &log, 80U, 0.0, true);
    status = est.words[S4_W_STATUS];
    assert(s4_get(status, S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH) == S4_CAL_LEVEL);
    assert(s4_get(status, S4_ST_VALID_SHIFT, 1U) == 1U);
    assert(s4_get(status, S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH) == S4_MC_STATIONARY);
    assert(s4_get(status, S4_ST_CLAMPED_SHIFT, 1U) == 0U);

    /* One bus error drops the single validity bit for the whole message. */
    lost.dt_ms = DT_MS;
    lost.bus_fault = 1U;
    log_step(&est, &lost, &log);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_VALID_SHIFT, 1U) == 0U);
    drive_level(&est, &car, &log, 10U, 0.0, true);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_VALID_SHIFT, 1U) == 1U);

    /* Hump flag is raised on the hump (Buddy 3 pauses sensing) and dropped
       once level again. */
    s4_est_reset_ranges(&est);
    drive_level(&est, &car, &log, 30U, 150.0, true);
    for (i = 0U; i < 1U; i++)
    {
        double x = 0.0;

        while (x < 400.0)
        {
            double slope = (40.0 * PI_D / 400.0) * sin(2.0 * PI_D * x / 400.0);
            long now_mm;

            x += 3.0 / sqrt(1.0 + (slope * slope));
            car.path_mm += 3.0;
            now_mm = lround(car.path_mm);
            in = car_sample(&car, atan(slope), 0.0, 0.0, true,
                            (int16_t)(now_mm - car.posted_mm));
            car.posted_mm = now_mm;
            log_step(&est, &in, &log);
            if (s4_get(est.words[S4_W_STATUS], S4_ST_HUMP_SHIFT, 1U) != 0U)
            {
                hump_flag_seen = true;
            }
        }
    }
    drive_level(&est, &car, &log, 60U, 150.0, true);
    assert(hump_flag_seen);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_HUMP_SHIFT, 1U) == 0U);
    assert((est.ranges.hump_height_mm >= 36U) && (est.ranges.hump_height_mm <= 44U));
    assert(est.ranges.pitch_ddeg >= 160U);   /* true peak 17.4 deg */

    /* Collision flag set during the impact hold, motion class IMPACT. */
    in = car_sample(&car, 0.0, -2200.0, 0.0, true, 3);
    log_step(&est, &in, &log);
    status = est.words[S4_W_STATUS];
    assert(s4_get(status, S4_ST_COLL_SHIFT, 1U) == 1U);
    assert(s4_get(status, S4_ST_MOTION_SHIFT, S4_ST_MOTION_WIDTH) == S4_MC_IMPACT);
    assert(est.ranges.shock_mg >= 2000U);
    drive_level(&est, &car, &log, 40U, 150.0, true);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_COLL_SHIFT, 1U) == 0U);

    /* Turn rate beyond +/-254 deg/s: field saturates, CLAMPED is set, and the
       range log keeps the true value (evidence for widening the field). */
    for (i = 0U; i < 15U; i++)
    {
        car.heading_rad += RAD(400.0 * DT_MS / 1000.0);
        in = car_sample(&car, 0.0, 0.0, 0.0, true, 0);
        log_step(&est, &in, &log);
    }
    status = est.words[S4_W_STATUS];
    assert(s4_get(status, S4_ST_CLAMPED_SHIFT, 1U) == 1U);
    assert(s4_get_signed(est.words[S4_W_MOTION], S4_MO_TURNRATE_SHIFT,
                         S4_MO_TURNRATE_WIDTH) == 127);
    assert(est.ranges.turn_rate_dps > 300U);
    assert(est.ranges.clamp_count > 0U);
    printf("  ranges: turn %u dps (field limit 254), hump %u mm, pitch %u.%u deg,"
           " shock %u mg, clamped %u\n",
           (unsigned)est.ranges.turn_rate_dps, (unsigned)est.ranges.hump_height_mm,
           (unsigned)(est.ranges.pitch_ddeg / 10U), (unsigned)(est.ranges.pitch_ddeg % 10U),
           (unsigned)est.ranges.shock_mg, (unsigned)est.ranges.clamp_count);
    drive_level(&est, &car, &log, 40U, 0.0, true);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CLAMPED_SHIFT, 1U) == 0U);

    /* Magnetometer calibration lifts the 2-bit status to LEVEL_MAG. */
    car.mount_tilt_rad = 0.0;
    s4_est_mag_calibration(&est, true, &events);
    for (i = 0U; i < 180U; i++)
    {
        car.heading_rad += RAD(2.0);
        in = car_sample(&car, 0.0, 0.0, 0.0, true, 0);
        log_step(&est, &in, &log);
    }
    events.count = 0U;
    s4_est_mag_calibration(&est, false, &events);
    drive_level(&est, &car, &log, 5U, 0.0, true);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH) ==
           S4_CAL_LEVEL_MAG);

    /* Failed calibration reads NONE. */
    assert(s4_est_init(&est, &cfg));
    s4_est_begin_calibration(&est);
    car.noise_mg = 120U;
    drive_level(&est, &car, &log, 200U, 0.0, true);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH) == S4_CAL_NONE);
}

static void test_faults(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    s4_input_t lost = {0};
    unsigned i;

    calibrated_estimator(&est, &car, &log);

    /* Bus error: flagged on the first failed read. */
    lost.dt_ms = DT_MS;
    lost.bus_fault = 1U;
    log_step(&est, &lost, &log);
    assert((est.faults & S4_FAULT_I2C) != 0U);
    assert(est.terrain == TERRAIN_INVALID);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) == 10U);

    /* No data for longer than stale_ms. */
    lost.bus_fault = 0U;
    for (i = 0U; i < 12U; i++)
    {
        log_step(&est, &lost, &log);
    }
    assert((est.faults & S4_FAULT_STALE) != 0U);
    assert((est.faults & S4_FAULT_MAG) != 0U);
    assert(log.faults >= 2);

    /* Recovery. */
    drive_level(&est, &car, &log, 30U, 0.0, true);
    assert((est.faults & (S4_FAULT_I2C | S4_FAULT_STALE | S4_FAULT_MAG)) == 0U);
    assert(est.terrain == TERRAIN_LEVEL);

    /* A sensor dropout during a hump aborts it as an invalid result. */
    drive_level(&est, &car, &log, 20U, 150.0, true);
    log.humps = 0;
    drive_hump(&est, &car, &log, 40.0, 200.0, 150.0, true);
    lost.bus_fault = 1U;
    log_step(&est, &lost, &log);
    assert(log.humps == 1);
    assert(s4_get(log.last_hump, S4_HU_VALID_SHIFT, 1U) == 0U);
}

static void test_tilt_and_mag_calibration(void)
{
    s4_est_t est;
    car_t car = {0};
    sim_log_t log;
    s4_events_t events = {0};
    unsigned i;

    calibrated_estimator(&est, &car, &log);

    /* Steady 30 deg slope: tilt fault, degraded cap. */
    for (i = 0U; i < 40U; i++)
    {
        s4_input_t in = car_sample(&car, RAD(30.0), 0.0, 0.0, true, 0);
        log_step(&est, &in, &log);
    }
    assert((est.faults & S4_FAULT_TILT) != 0U);
    assert(s4_get(est.words[S4_W_STATUS], S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) == 10U);

    /* Hard-iron calibration: spin with a constant offset added. Mounted flat:
       a tilted sensor leaks the vertical field into X/Y, which a flat spin
       cannot tell apart from hard iron (documented limitation). */
    calibrated_estimator(&est, &car, &log);
    car.mount_tilt_rad = 0.0;
    s4_est_mag_calibration(&est, true, &events);
    for (i = 0U; i < 180U; i++)
    {
        s4_input_t in;

        car.heading_rad += RAD(2.0);
        in = car_sample(&car, 0.0, 0.0, 0.0, true, 0);
        in.mag[0] = (int16_t)(in.mag[0] + 80);
        in.mag[1] = (int16_t)(in.mag[1] - 50);
        log_step(&est, &in, &log);
    }
    events.count = 0U;   /* the caller owns and resets the event list */
    s4_est_mag_calibration(&est, false, &events);
    assert(events.count == 1U);
    assert(s4_get(events.msg[0], S4_EV_CAL_KIND_SHIFT, 1U) == 1U);
    assert(s4_get(events.msg[0], S4_EV_CAL_OK_SHIFT, 1U) == 1U);
    assert(abs(est.mag_off[0] - 80) <= 3);
    assert(abs(est.mag_off[1] + 50) <= 3);
    assert(est.mag_calibrated);
}

/* ------------------------------------------------------------------ */
/*  Change Requirement 1: representation sizes (host ABI)              */
/* ------------------------------------------------------------------ */

/* Conventional "before" layouts: one natural C type per field. */
typedef struct
{
    terrain_state_t terrain;
    uint32_t speed_cap_mm_s;
    int32_t motion_class;
    int32_t longitudinal;
    bool fault_i2c, fault_mag, fault_stale, fault_uncal;
    bool fault_saturated, fault_tilt, fault_collision;
    bool calibrated, mag_calibrated, distance_from_odometry, heading_valid;
    uint32_t collision_count;
    int32_t collision_dir;
    int32_t turn_state;
    float pitch_deg, roll_deg, heading_deg;
    uint32_t sample_seq;
    float turn_rate_dps;
    int32_t live_height_mm;
    uint32_t shock_mg;
    uint32_t vibration_mg;
    int32_t hump_height_mm;
    uint32_t hump_uncertainty_mm;
    int32_t hump_peak_pitch_deg;
    uint32_t hump_length_mm;
    bool hump_valid;
    uint32_t hump_seq;
} baseline_snapshot_t;

typedef struct
{
    int32_t type;
    terrain_state_t state;
    uint32_t speed_cap_mm_s;
    int32_t height_mm;
    uint32_t uncertainty_mm;
    int32_t peak_pitch_deg;
    uint32_t length_mm;
    bool valid;
    int32_t collision_dir;
    uint32_t magnitude_mg;
    int32_t turn_angle_deg;
    uint32_t fault_mask;
} baseline_event_t;

static void print_sizes(void)
{
    printf("  sizes on this host: snapshot %u -> %u B, event %u -> %u B,"
           " terrain_report_t %u B, cfg %u B, estimator %u B\n",
           (unsigned)sizeof(baseline_snapshot_t),
           (unsigned)(sizeof(uint32_t) * S4_WORDS),
           (unsigned)sizeof(baseline_event_t),
           (unsigned)sizeof(s4_event_msg_t),
           (unsigned)sizeof(terrain_report_t),
           (unsigned)sizeof(s4_cfg_t),
           (unsigned)sizeof(s4_est_t));
}

int main(void)
{
    /* Unbuffered so diagnostics survive a failed assert. */
    (void)setvbuf(stdout, NULL, _IONBF, 0);
    test_atan2_accuracy();
    test_isqrt_and_sin();
    test_decode_and_axes();
    test_heading();
    test_packing();
    test_shared_and_payloads();
    test_config_validation();
    test_calibration_rejects_motion();
    test_level_after_calibration();
    test_hump_with_odometry();
    test_small_and_large_humps();
    test_hump_without_odometry();
    test_collision();
    test_turn();
    test_faults();
    test_tilt_and_mag_calibration();
    test_tribunal_fields();
    print_sizes();
    (void)puts("terrain estimator host tests passed");
    return 0;
}
