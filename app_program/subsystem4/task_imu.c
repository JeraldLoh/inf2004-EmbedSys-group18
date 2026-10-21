/*
 * Buddy 4 IMU task: the only owner of I2C1 and of the estimator state.
 *
 * Every S4_PERIOD_MS a cyclic handler sets a tick bit in s4_flg_request. The
 * task wakes on that bit or on a request bit, reads the LSM303DLHC, runs one
 * estimator step, publishes the packed snapshot and forwards events.
 *
 * Kernel objects (4): event flag, message buffer, cyclic handler, task.
 */
#include "s4_imu.h"

#include "../contracts/fault_mask.h"
#include "drv_lsm303.h"
#include "s4_config.h"
#include "terrain_estimator.h"

#define S4_FLG_TICK 0x8000U              /* private: set by the cyclic handler */
#define S4_WAIT_TIMEOUT_MS (S4_PERIOD_MS * 5U)
#define S4_MBF_MSG_BYTES ((SZ)sizeof(s4_event_msg_t))
#define S4_MBF_SLOT_BYTES (S4_MBF_MSG_BYTES + (SZ)sizeof(INT)) /* + kernel header */

_Static_assert((S4_REQ_ALL & S4_FLG_TICK) == 0U, "tick bit overlaps requests");

EXPORT ID s4_flg_request = 0;
EXPORT ID s4_mbf_events = 0;
EXPORT UW s4_events_dropped = 0U;
EXPORT UW s4_bus_errors = 0U;
EXPORT UW s4_sample_time_us = 0U;
EXPORT UW s4_sample_time_worst_us = 0U;

LOCAL ID s4_cyc_tick = 0;
LOCAL ID s4_tid_imu = 0;

/* Task-private state: touched only by s4_imu_task. */
LOCAL s4_est_t s4_est;
LOCAL drv_lsm303_t s4_dev;
LOCAL s4_odom_reader_t s4_odom;
LOCAL fault_mask_t s4_published_faults = 0U;

/*
 * Publish Buddy 4's field of the shared fault word, only when it changes.
 * fault_mask_update() is atomic, so other buddies' bits are never lost.
 */
LOCAL void s4_publish_faults(const s4_est_t *est)
{
    fault_mask_t bits = 0U;

    if ((est->faults & (S4_FAULT_I2C | S4_FAULT_STALE)) != 0U)
    {
        bits |= FAULT_B4_IMU;
    }
    if ((est->faults & S4_FAULT_UNCAL) != 0U)
    {
        bits |= FAULT_B4_IMU_UNCAL;
    }
    if ((est->faults & S4_FAULT_MAG) != 0U)
    {
        bits |= FAULT_B4_MAG;
    }
    if ((est->faults & S4_FAULT_COLLISION) != 0U)
    {
        bits |= FAULT_B4_COLLISION;
    }
    if (bits != s4_published_faults)
    {
        fault_mask_update(FAULT_MASK_B4, bits);
        s4_published_faults = bits;
    }
}

/* Print the observed ranges and sample timing (tribunal evidence). */
LOCAL void s4_report_ranges(const s4_range_log_t *r)
{
    tm_printf((UB *)"S4 ranges: turn %d dps (field +/-%d), pitch %d.%d deg, roll %d.%d deg\n",
              (INT)r->turn_rate_dps,
              (INT)(S4_MASK(S4_MO_TURNRATE_WIDTH - 1U) * S4_TURNRATE_UNIT_DPS),
              (INT)(r->pitch_ddeg / 10U), (INT)(r->pitch_ddeg % 10U),
              (INT)(r->roll_ddeg / 10U), (INT)(r->roll_ddeg % 10U));
    tm_printf((UB *)"S4 ranges: hump %d mm / %d mm long, shock %d mg, vibration %d mg, clamped %d\n",
              (INT)r->hump_height_mm, (INT)r->hump_length_mm, (INT)r->shock_mg,
              (INT)r->vibration_mg, (INT)r->clamp_count);
    tm_printf((UB *)"S4 sample time: last %d us, worst %d us (period %d ms)\n",
              (INT)s4_sample_time_us, (INT)s4_sample_time_worst_us, (INT)S4_PERIOD_MS);
}

/* Cyclic handler: interrupt context, so only a non-blocking flag set. */
LOCAL void s4_tick_handler(void *exinf)
{
    (void)exinf;
    (void)tk_set_flg(s4_flg_request, S4_FLG_TICK);
}

#if S4_LOG_EVENTS
LOCAL const char *s4_terrain_name(UINT state)
{
    static const char *const names[] = {
        "UNKNOWN", "LEVEL", "ASCENDING", "CREST", "DESCENDING", "SETTLING", "INVALID"
    };
    return (state < 7U) ? names[state] : "?";
}

LOCAL void s4_log_event(s4_event_msg_t msg)
{
    static const char *const sides[] = {"front", "rear", "left", "right"};

    switch (s4_get(msg, S4_EV_TYPE_SHIFT, S4_EV_TYPE_WIDTH))
    {
        case S4_EV_TERRAIN:
            tm_printf((UB *)"S4 terrain %s, cap %d mm/s\n",
                      s4_terrain_name(s4_get(msg, S4_EV_TERRAIN_STATE_SHIFT,
                                             S4_ST_TERRAIN_WIDTH)),
                      (INT)(s4_get(msg, S4_EV_TERRAIN_CAP_SHIFT, S4_ST_CAP_WIDTH) *
                            S4_CAP_UNIT_MM_S));
            break;
        case S4_EV_HUMP:
            tm_printf((UB *)"S4 hump %d mm +/-%d, peak %d deg, %d cm, %s\n",
                      (INT)s4_get(msg, S4_EV_PAYLOAD_SHIFT + S4_HU_HEIGHT_SHIFT,
                                  S4_HU_HEIGHT_WIDTH),
                      (INT)s4_get(msg, S4_EV_PAYLOAD_SHIFT + S4_HU_UNC_SHIFT,
                                  S4_HU_UNC_WIDTH),
                      (INT)s4_get(msg, S4_EV_PAYLOAD_SHIFT + S4_HU_PITCH_SHIFT,
                                  S4_HU_PITCH_WIDTH),
                      (INT)s4_get(msg, S4_EV_PAYLOAD_SHIFT + S4_HU_LEN_SHIFT,
                                  S4_HU_LEN_WIDTH),
                      s4_get(msg, S4_EV_PAYLOAD_SHIFT + S4_HU_VALID_SHIFT, 1U)
                          ? "valid" : "INVALID");
            break;
        case S4_EV_COLLISION:
            tm_printf((UB *)"S4 collision on %s side, %d mg\n",
                      sides[s4_get(msg, S4_EV_COLL_DIR_SHIFT, S4_ST_COLLDIR_WIDTH)],
                      (INT)(s4_get(msg, S4_EV_COLL_MAG_SHIFT, S4_MO_SHOCK_WIDTH) *
                            S4_SHOCK_UNIT_MG));
            break;
        case S4_EV_TURN:
            tm_printf((UB *)"S4 turned %s %d deg\n",
                      s4_get(msg, S4_EV_TURN_RIGHT_SHIFT, 1U) ? "right" : "left",
                      (INT)s4_get(msg, S4_EV_TURN_ANGLE_SHIFT, S4_EV_TURN_ANGLE_WIDTH));
            break;
        case S4_EV_FAULT:
            tm_printf((UB *)"S4 faults 0x%x\n",
                      (UINT)s4_get(msg, S4_EV_FAULT_SHIFT, S4_FAULT_WIDTH));
            break;
        case S4_EV_CAL:
            tm_printf((UB *)"S4 %s calibration %s\n",
                      s4_get(msg, S4_EV_CAL_KIND_SHIFT, 1U) ? "magnetometer" : "level",
                      s4_get(msg, S4_EV_CAL_OK_SHIFT, 1U) ? "done" : "FAILED");
            break;
        default:
            break;
    }
}
#endif

/* Queue full: drop the OLDEST event so the controller sees recent ones. */
LOCAL void s4_dispatch(const s4_events_t *events)
{
    s4_event_msg_t discard;
    UB i;

    for (i = 0U; i < events->count; i++)
    {
        s4_event_msg_t msg = events->msg[i];

        if (tk_snd_mbf(s4_mbf_events, &msg, S4_MBF_MSG_BYTES, TMO_POL) < E_OK)
        {
            s4_events_dropped++;
            (void)tk_rcv_mbf(s4_mbf_events, &discard, TMO_POL);
            (void)tk_snd_mbf(s4_mbf_events, &msg, S4_MBF_MSG_BYTES, TMO_POL);
        }
#if S4_LOG_EVENTS
        s4_log_event(msg);
#endif
    }
}

/* Read the sensor into an estimator input; handles bus faults and re-open. */
LOCAL void s4_acquire(s4_input_t *in)
{
    static UW reopen_wait = 0U;
    static UW error_run = 0U;
    UB raw[6];
    BOOL fresh = FALSE;
    bool flag = false;
    int16_t odom_delta = 0;
    ER er;

    if (!s4_dev.open)
    {
        if (reopen_wait > 0U)
        {
            reopen_wait--;
            in->bus_fault = 1U;
        }
        else if (drv_lsm303_open(&s4_dev, S4_ACCEL_FS_CODE) < E_OK)
        {
            reopen_wait = S4_REOPEN_TICKS;
            in->bus_fault = 1U;
            s4_bus_errors++;
        }
        else
        {
            error_run = 0U;
        }
    }

    if (s4_dev.open)
    {
        er = drv_lsm303_read_accel(&s4_dev, raw, &fresh);
        if (er < E_OK)
        {
            in->bus_fault = 1U;
            s4_bus_errors++;
            if (++error_run >= S4_MAX_BUS_ERRORS)
            {
                drv_lsm303_close(&s4_dev);
                reopen_wait = S4_REOPEN_TICKS;
            }
        }
        else
        {
            error_run = 0U;
            if (fresh && imu_decode_accel_mg(raw, S4_ACCEL_FS_CODE, in->accel_mg, &flag))
            {
                in->accel_ok = 1U;
                in->accel_saturated = flag ? 1U : 0U;
            }
        }
    }

    if (s4_dev.open && s4_dev.mag_present &&
        (drv_lsm303_read_mag(&s4_dev, raw) >= E_OK) &&
        imu_decode_mag(raw, in->mag, &flag) && !flag)
    {
        in->mag_ok = 1U;
    }

    in->odom_valid = s4_take_odometry(&s4_odom, S4_ODOM_STALE_TICKS, &odom_delta) ? 1U : 0U;
    in->odom_delta_mm = odom_delta;
}

LOCAL void s4_imu_task(INT stacd, void *exinf)
{
    static const s4_input_t no_input = {0};
    s4_cfg_t cfg;
    s4_input_t in;
    s4_events_t events;
    SYSTIM now;
    SYSTIM last;
    UINT pattern;
    UW elapsed;
    UW sample_start;
    ER er;

    (void)stacd;
    (void)exinf;

    s4_est_default_config(&cfg);
    if (!s4_est_init(&s4_est, &cfg))
    {
        tm_printf((UB *)"S4 invalid s4_config.h; IMU task stopped\n");
        tk_ext_tsk();
    }
    s4_est_begin_calibration(&s4_est);
    tm_printf((UB *)"S4 IMU task started; keep the car still and level\n");
    (void)tk_get_tim(&last);

    for (;;)
    {
        er = tk_wai_flg(s4_flg_request, S4_REQ_ALL | S4_FLG_TICK,
                        TWF_ORW | TWF_BITCLR, &pattern, S4_WAIT_TIMEOUT_MS);
        if (er == E_TMOUT)
        {
            pattern = S4_FLG_TICK;   /* keep reporting staleness */
        }
        else if (er < E_OK)
        {
            (void)tk_dly_tsk(S4_PERIOD_MS);
            continue;
        }

        events.count = 0U;
        if ((pattern & S4_REQ_RECALIBRATE) != 0U)
        {
            s4_est_begin_calibration(&s4_est);
        }
        if ((pattern & S4_REQ_CLEAR_LATCHED) != 0U)
        {
            s4_est_clear_latched(&s4_est);
        }
        if ((pattern & S4_REQ_MAG_CAL_START) != 0U)
        {
            s4_est_mag_calibration(&s4_est, true, &events);
        }
        if ((pattern & S4_REQ_MAG_CAL_STOP) != 0U)
        {
            s4_est_mag_calibration(&s4_est, false, &events);
        }
        if ((pattern & S4_REQ_REPORT_RANGES) != 0U)
        {
            s4_report_ranges(&s4_est.ranges);
            s4_est_reset_ranges(&s4_est);
            s4_sample_time_worst_us = 0U;
        }
        s4_dispatch(&events);

        if ((pattern & S4_FLG_TICK) == 0U)
        {
            continue;
        }

        (void)tk_get_tim(&now);
        elapsed = now.lo - last.lo;
        last = now;

        in = no_input;   /* <string.h> clashes with the kernel size_t */
        in.dt_ms = (uint16_t)((elapsed > 1000U) ? 1000U : elapsed);

        /* Sampling time = I2C reads + estimation + publish, 1 MHz timer. */
        sample_start = in_w(TIMER_TIMERAWL);
        s4_acquire(&in);
        s4_est_step(&s4_est, &in, &events);
        s4_shared_publish(s4_est.words);
        s4_publish_faults(&s4_est);
        s4_sample_time_us = in_w(TIMER_TIMERAWL) - sample_start;
        if (s4_sample_time_us > s4_sample_time_worst_us)
        {
            s4_sample_time_worst_us = s4_sample_time_us;
        }
        s4_dispatch(&events);
    }
}

LOCAL void s4_delete_objects(void)
{
    if (s4_cyc_tick > 0)
    {
        (void)tk_del_cyc(s4_cyc_tick);
        s4_cyc_tick = 0;
    }
    if (s4_mbf_events > 0)
    {
        (void)tk_del_mbf(s4_mbf_events);
        s4_mbf_events = 0;
    }
    if (s4_flg_request > 0)
    {
        (void)tk_del_flg(s4_flg_request);
        s4_flg_request = 0;
    }
}

EXPORT ER s4_imu_init(void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};
    T_CCYC ccyc = {0};
    ID id;

    /* Single waiter: the IMU task. Requests come from any task/handler. */
    cflg.flgatr = TA_TFIFO | TA_WSGL;
    cflg.iflgptn = 0U;
    id = tk_cre_flg(&cflg);
    if (id < E_OK)
    {
        return (ER)id;
    }
    s4_flg_request = id;

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = (SZ)(S4_EVENT_QUEUE_DEPTH * S4_MBF_SLOT_BYTES);
    cmbf.maxmsz = (INT)S4_MBF_MSG_BYTES;
    id = tk_cre_mbf(&cmbf);
    if (id < E_OK)
    {
        s4_delete_objects();
        return (ER)id;
    }
    s4_mbf_events = id;

    /* Created stopped; s4_imu_start() starts it after the task exists. */
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP)s4_tick_handler;
    ccyc.cyctim = S4_PERIOD_MS;
    ccyc.cycphs = S4_PERIOD_MS;
    id = tk_cre_cyc(&ccyc);
    if (id < E_OK)
    {
        s4_delete_objects();
        return (ER)id;
    }
    s4_cyc_tick = id;
    return E_OK;
}

EXPORT ER s4_imu_start(void)
{
    T_CTSK ctsk = {0};
    ID tid;
    ER er;

    if ((s4_flg_request <= 0) || (s4_mbf_events <= 0) || (s4_cyc_tick <= 0))
    {
        return E_OBJ;
    }

    ctsk.task = (FP)s4_imu_task;
    ctsk.itskpri = S4_TASK_PRIORITY;
    ctsk.stksz = S4_TASK_STACK_BYTES;
    ctsk.tskatr = TA_HLNG | TA_RNG3;
    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        return (ER)tid;
    }
    er = tk_sta_tsk(tid, 0);
    if (er < E_OK)
    {
        (void)tk_del_tsk(tid);
        return er;
    }
    s4_tid_imu = tid;
    return tk_sta_cyc(s4_cyc_tick);
}

EXPORT ER s4_request(UINT bits)
{
    if (s4_flg_request <= 0)
    {
        return E_OBJ;
    }
    bits &= S4_REQ_ALL;
    return (bits == 0U) ? E_OK : tk_set_flg(s4_flg_request, bits);
}

EXPORT ER s4_command(UB byte)
{
    static const UINT request_for[] = {
        0U,                      /* S4_CMD_NONE */
        S4_REQ_RECALIBRATE,
        S4_REQ_CLEAR_LATCHED,
        S4_REQ_MAG_CAL_START,
        S4_REQ_MAG_CAL_STOP,
        S4_REQ_REPORT_RANGES
    };
    uint8_t command;

    if (!s4_command_decode(byte, &command))
    {
        return E_PAR;
    }
    return s4_request(request_for[command]);
}
