/* Buddy 3 line PID (line_pid.c): calibration, error scaling, steering sign,
 * P/D/I terms, anti-windup and gap reset. Default build: black reads high,
 * sensor on the LEFT tape edge, so more black means steer left (negative). */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem3/line_pid.h"

#define WHITE 500U
#define BLACK 3500U
#define EDGE ((WHITE + BLACK) / 2U)

static void calibrated(line_pid_t *pid)
{
    line_pid_init(pid);
    line_pid_calibrate_start(pid);
    line_pid_calibrate_sample(pid, WHITE);
    line_pid_calibrate_sample(pid, BLACK);
    line_pid_calibrate_sample(pid, EDGE);
    assert(line_pid_calibrate_finish(pid));
}

static void test_calibration(void)
{
    line_pid_t pid;

    /* Uncalibrated: no steering, no error. */
    line_pid_init(&pid);
    assert(!pid.calibrated);
    assert(line_pid_step(&pid, BLACK, 10U) == 0);
    assert(line_pid_error(&pid, BLACK) == 0);

    /* Too little contrast (sensor too high, or not swept) is refused. */
    line_pid_calibrate_start(&pid);
    line_pid_calibrate_sample(&pid, 1000U);
    line_pid_calibrate_sample(&pid, 1200U);
    assert(!line_pid_calibrate_finish(&pid));
    assert(line_pid_step(&pid, BLACK, 20U) == 0);

    calibrated(&pid);
    assert(pid.white_level == WHITE);
    assert(pid.black_level == BLACK);

    /* Out-of-range ADC values clamp instead of wrapping. */
    assert(line_pid_darkness(5000U) == LINE_PID_ADC_MAX);
}

/* The tests pin their gains (CMake sets them for line_pid.c too), so
 * retuning the car's defaults in line_pid.h does not break them. */
#if (LINE_PID_KP_X100 != 80) || ((LINE_PID_KI_X100 != 0) && (LINE_PID_KI_X100 != 100)) || \
    ((LINE_PID_KD_X100 != 300) && (LINE_PID_KD_X100 != 0))
#error "build test_line_pid.c with -DLINE_PID_KP_X100=80 and KD 300 (or KI 100, KD 0)"
#endif

#if LINE_PID_KI_X100 != 100
/* Gains: Kp 0.80, Ki 0, Kd 3.00. */
static void test_error_scaling(void)
{
    line_pid_t pid;

    calibrated(&pid);
    assert(line_pid_error(&pid, WHITE) == -100);
    assert(line_pid_error(&pid, BLACK) == 100);
    assert(line_pid_error(&pid, EDGE) == 0);
    assert(line_pid_error(&pid, EDGE + 750U) == 50);   /* quarter span */
    assert(line_pid_error(&pid, 0U) == -100);          /* past white clamps */
    assert(line_pid_error(&pid, 4095U) == 100);        /* past black clamps */
}

static void test_proportional_sign(void)
{
    line_pid_t pid;
    int8_t steer;

    calibrated(&pid);
    /* On the edge: straight. */
    assert(line_pid_step(&pid, EDGE, 100U) == 0);

    /* First step after a reset has no D term: pure P, Kp = 0.80. */
    line_pid_reset(&pid);
    steer = line_pid_step(&pid, EDGE + 750U, 200U);   /* +50 % error */
    assert(steer == -40);                              /* drifted right: steer left */

    line_pid_reset(&pid);
    steer = line_pid_step(&pid, EDGE - 750U, 300U);   /* -50 % error */
    assert(steer == 40);                               /* drifted left: steer right */

    /* Output never exceeds the -100..+100 contract. */
    line_pid_reset(&pid);
    assert(line_pid_step(&pid, BLACK, 400U) >= -100);
}

static void test_derivative_damps(void)
{
    line_pid_t pid;
    uint32_t now_ms;
    int8_t steer = 0;

    /* Error jumps 0 -> 50 %. D is held at 0 until the first 10 ms window
     * closes, then adds extra left steer on top of P (40). */
    calibrated(&pid);
    (void)line_pid_step(&pid, EDGE, 1000U);
    for (now_ms = 1001U; now_ms < (1000U + LINE_PID_D_WINDOW_MS); now_ms++)
    {
        assert(line_pid_step(&pid, EDGE + 750U, now_ms) == -40);
    }
    steer = line_pid_step(&pid, EDGE + 750U, now_ms);
    assert(steer <= -(40 + 10));

    /* Error held: the smoothed error settles and D fades back to 0. */
    for (now_ms = 1011U; now_ms <= 1040U; now_ms++)
    {
        steer = line_pid_step(&pid, EDGE + 750U, now_ms);
    }
    assert(steer == -40);
}

static void test_derivative_ignores_noise(void)
{
    line_pid_t pid;
    uint32_t now_ms;
    int8_t steer;

    /* +/-2 % sample-to-sample noise on the edge. Sample-to-sample D would
     * give +/-12; the smoothed window keeps the output near P (+/-1). */
    calibrated(&pid);
    (void)line_pid_step(&pid, EDGE, 1000U);
    for (now_ms = 1001U; now_ms <= 1100U; now_ms++)
    {
        steer = line_pid_step(&pid, ((now_ms & 1U) != 0U) ? (EDGE + 30U) : (EDGE - 30U), now_ms);
        assert((steer >= -3) && (steer <= 3));
    }
}

static void test_gap_resets_history(void)
{
    line_pid_t pid;

    calibrated(&pid);
    (void)line_pid_step(&pid, EDGE - 1500U, 1000U);    /* -100 % */
    /* After a long gap (probe, bypass) the old error must not create a
     * huge D kick: only P on the new error. */
    assert(line_pid_step(&pid, EDGE + 150U, 1000U + LINE_PID_GAP_MS + 1U) == -8);
}

static void test_saturation_and_blend(void)
{
    line_pid_t pid;

    calibrated(&pid);
    assert(!line_pid_saturated(&pid, EDGE));
    assert(!line_pid_saturated(&pid, EDGE + 1300U));   /* 86 % */
    assert(line_pid_saturated(&pid, EDGE + 1400U));    /* 93 % */
    assert(line_pid_saturated(&pid, WHITE));           /* -100 % */

    /* Not saturated: always the PID, however hard digital steers. */
    assert(line_pid_blend(-20, -90, false) == -20);
    /* Saturated on a bend: digital ramp steers harder the same way. */
    assert(line_pid_blend(-35, -80, true) == -80);
    assert(line_pid_blend(35, 80, true) == 80);
    /* Digital weaker, or the other way (stale hold): keep the PID. */
    assert(line_pid_blend(35, 20, true) == 35);
    assert(line_pid_blend(35, -20, true) == 35);
    assert(line_pid_blend(-35, 60, true) == -35);
}

static void test_reset_keeps_calibration(void)
{
    line_pid_t pid;

    calibrated(&pid);
    line_pid_reset(&pid);
    assert(pid.calibrated);
    assert(!pid.primed);
    assert(pid.integral == 0);
}

#else
/* Built a second time with -DLINE_PID_KI_X100=100 -DLINE_PID_KD_X100=0. */
static void test_integral_windup(void)
{
    line_pid_t pid;
    uint32_t now_ms;
    int8_t steer = 0;

    calibrated(&pid);
    /* Hold +50 % error for 2 s at 1 ms steps. P = 40; I would reach 100
     * without the limit, but stops at LINE_PID_I_LIMIT = 30. */
    for (now_ms = 1000U; now_ms <= 3000U; now_ms++)
    {
        steer = line_pid_step(&pid, EDGE + 750U, now_ms);
    }
    assert(steer == -(40 + LINE_PID_I_LIMIT));

    /* Back on the edge: P = 0 but the stored I still pulls until the error
     * crosses to the other side. */
    steer = line_pid_step(&pid, EDGE, now_ms);
    assert(steer == -LINE_PID_I_LIMIT);
}
#endif

int main(void)
{
    test_calibration();
#if LINE_PID_KI_X100 == 100
    test_integral_windup();
#else
    test_error_scaling();
    test_proportional_sign();
    test_derivative_damps();
    test_derivative_ignores_noise();
    test_gap_resets_history();
    test_saturation_and_blend();
    test_reset_keeps_calibration();
#endif
    (void)printf("line_pid host tests passed\n");
    return 0;
}
