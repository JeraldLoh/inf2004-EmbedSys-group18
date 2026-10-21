/*
 * Buddy 4 on-target checks. Built only with: make TEST=1
 * Preconditions: IMU wired on GP2 (SDA) / GP3 (SCL), I2C1 enabled,
 * car powered, wheels still, on a level floor, motors not driven.
 * IDs match testing/subsystem4/TEST_LOG.md.
 */
#include "s4_imu.h"

#include "../contracts/fault_mask.h"
#include "../test_report.h"
#include "s4_config.h"
#include "terrain_estimator.h"

#if APP_TEST

#define S4_TIMING_STEPS 200U
#define S4_STEP_BUDGET_US 1000U   /* 5% of the 20 ms period */

/* Private estimator for timing, so the running task is not disturbed. */
LOCAL s4_est_t s4_timing_est;

/* Worst and mean s4_est_step() time on this core, from the 1 MHz timer. */
LOCAL void s4_time_estimator(UW *mean_us, UW *worst_us)
{
    s4_cfg_t cfg;
    s4_input_t in = {0};
    s4_events_t events;
    UW total = 0U;
    UW worst = 0U;
    UW i;

    s4_est_default_config(&cfg);
    (void)s4_est_init(&s4_timing_est, &cfg);
    s4_est_begin_calibration(&s4_timing_est);
    in.accel_mg[2] = 1000;
    in.mag[1] = 220;
    in.mag[2] = -420;
    in.dt_ms = S4_PERIOD_MS;
    in.accel_ok = 1U;
    in.mag_ok = 1U;
    in.odom_valid = 1U;
    in.odom_delta_mm = 3;
    for (i = 0U; i < S4_TIMING_STEPS; i++)
    {
        UW start = in_w(TIMER_TIMERAWL);
        UW spent;

        in.accel_mg[1] = (int16_t)((i & 7U) * 4U);   /* keep it busy */
        s4_est_step(&s4_timing_est, &in, &events);
        spent = in_w(TIMER_TIMERAWL) - start;
        total += spent;
        if (spent > worst)
        {
            worst = spent;
        }
    }
    *mean_us = total / S4_TIMING_STEPS;
    *worst_us = worst;
}

EXPORT void s4_imu_selftest(void)
{
    uint32_t first[S4_WORDS];
    uint32_t words[S4_WORDS];
    uint32_t status;
    T_RFLG rflg;
    T_RMBF rmbf;
    BOOL have_first;
    BOOL have_now;
    INT pitch_half;
    INT roll_half;
    UB frame[S4_TLM_SIZE];
    UW mean_us;
    UW worst_us;

    TEST_INIT();

    TEST_CHECK("S4-T01",
               (tk_ref_flg(s4_flg_request, &rflg) == E_OK) &&
               (tk_ref_mbf(s4_mbf_events, &rmbf) == E_OK) &&
               (rmbf.maxmsz == (INT)sizeof(s4_event_msg_t)),
               "Buddy 4 kernel objects exist, events are 4 bytes");

    /* Calibration takes S4_CAL_SAMPLES periods once the bus is up. */
    have_first = s4_shared_read(first) ? TRUE : FALSE;
    (void)tk_dly_tsk((S4_CAL_SAMPLES * S4_PERIOD_MS) + 1500U);
    have_now = s4_shared_read(words) ? TRUE : FALSE;
    status = words[S4_W_STATUS];

    TEST_CHECK("S4-T02",
               have_first && have_now &&
               (s4_get(first[S4_W_ATTITUDE], S4_AT_SEQ_SHIFT, S4_AT_SEQ_WIDTH) !=
                s4_get(words[S4_W_ATTITUDE], S4_AT_SEQ_SHIFT, S4_AT_SEQ_WIDTH)),
               "IMU task publishes fresh samples");

    TEST_CHECK("S4-T03",
               (s4_get(status, S4_ST_FAULT_SHIFT, S4_ST_FAULT_WIDTH) &
                (S4_FAULT_I2C | S4_FAULT_STALE)) == 0U,
               "LSM303 answers on I2C1 with no bus or stale fault");

    TEST_CHECK("S4-T04",
               s4_get(status, S4_ST_CAL_SHIFT, S4_ST_CAL_WIDTH) >= S4_CAL_LEVEL,
               "level calibration completed (2-bit status LEVEL or LEVEL_MAG)");

    TEST_CHECK("S4-T05",
               s4_get(status, S4_ST_TERRAIN_SHIFT, S4_ST_TERRAIN_WIDTH) == TERRAIN_LEVEL,
               "terrain reports LEVEL when still on a level floor");

    pitch_half = (INT)s4_get_signed(words[S4_W_ATTITUDE], S4_AT_PITCH_SHIFT,
                                    S4_AT_PITCH_WIDTH);
    roll_half = (INT)s4_get_signed(words[S4_W_ATTITUDE], S4_AT_ROLL_SHIFT,
                                   S4_AT_ROLL_WIDTH);
    TEST_CHECK("S4-T06",
               (pitch_half >= -4) && (pitch_half <= 4) &&
               (roll_half >= -4) && (roll_half <= 4),
               "pitch and roll within +/-2 deg after calibration");

    TEST_CHECK("S4-T07",
               s4_get(status, S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) == 0U,
               "no terrain speed cap on level ground");

    TEST_CHECK("S4-T08",
               (s4_get(status, S4_ST_FAULT_SHIFT, S4_ST_FAULT_WIDTH) &
                S4_FAULT_MAG) == 0U,
               "magnetometer present (heading available)");

    TEST_CHECK("S4-T11",
               s4_get(status, S4_ST_VALID_SHIFT, 1U) != 0U,
               "message data-valid bit set once calibrated with no bus fault");

    TEST_CHECK("S4-T12",
               (fault_mask_read() & (FAULT_B4_IMU | FAULT_B4_IMU_UNCAL)) == 0U,
               "shared fault word: Buddy 4 IMU and calibration bits clear");

    s4_time_estimator(&mean_us, &worst_us);
    tm_printf((UB *)"[TEST] S4 info: estimator step mean %d us, worst %d us\n",
              (INT)mean_us, (INT)worst_us);
    TEST_CHECK("S4-T09", worst_us < S4_STEP_BUDGET_US,
               "estimator step worst case under 1 ms (5% of period)");

    TEST_CHECK("S4-T10",
               (sizeof(s4_event_msg_t) == 4U) &&
               (s4_telemetry_pack(frame, sizeof(frame)) == 17U) &&
               (frame[0] == (S4_TLM_VERSION | (S4_TLM_TYPE << S4_TLM_HDR_TYPE_SHIFT))),
               "event is 4 bytes, MQTT telemetry frame is 17 bytes");

    tm_printf((UB *)"[TEST] S4 info: sample time worst %d us (I2C + estimate)\n",
              (INT)s4_sample_time_worst_us);
    tm_printf((UB *)"[TEST] S4 info: status=0x%x attitude=0x%x bus_errors=%d dropped=%d\n",
              (UINT)status, (UINT)words[S4_W_ATTITUDE],
              (INT)s4_bus_errors, (INT)s4_events_dropped);
    TEST_SUMMARY("subsystem4");
}

#endif
