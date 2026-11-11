/* Buddy 3: line + barcode acquisition task. DRAFT, not yet run on the car.
 *
 * Data flow, once per sample period:
 *
 *   cyclic handler --wake--> s3_task
 *     |- apply requests from the controller (probe, reacquire)
 *     |- drv_ir_read_line()    -> line_follow_update()  -> steering -100..100
 *     |- barcode: ADC sample   -> barcode_sensor_is_black() -> decoder
 *     '- publish line status, route, junction
 *
 * JUNCTIONS: a barcode before a junction decides the direction. With no
 * barcode the default is STRAIGHT at a "+" and RIGHT at a T. The two shapes
 * look the same to straddling sensors, so the no-barcode case runs a probe:
 *   1. crossbar confirmed, no barcode -> subsystem3_probe_requested() is TRUE
 *   2. controller stops the car just past the bar and calls
 *      subsystem3_probe_begin(), then has Buddy 2 pivot ~10 deg left and back
 *   3. controller calls subsystem3_probe_end()
 *   4. S3 publishes the default route (STRAIGHT or RIGHT) and then the
 *      confirmed junction, so the controller executes it as usual
 * Each cycle the controller must take the route BEFORE the junction.
 *
 * BARCODE SPEED: bar widths are measured in time, so the speed must be steady
 * while a barcode is read. line_status.barcode_active is 1 during a read; the
 * controller should hold a constant (slower) speed until it clears.
 *
 * TIMING: the kernel tick is CNF_TIMER_PERIOD (config/config.h), currently
 * 10 ms. ADC polling needs a 1 ms period (CNF_TIMER_PERIOD = 1, a shared
 * setting; CAR_RUN builds already use it).
 *
 * BARCODE THRESHOLDS: by default (S3_BARCODE_FIXED = 1) fixed thresholds
 * apply from power-on, with no calibration sweep. LINE_PID=1 builds (or
 * S3_BARCODE_FIXED = 0) still sweep when subsystem3_calibrate_barcode() is
 * called: 3 s of samples set the thresholds while line following carries on,
 * and until that succeeds every sample reads white, so no barcode is decoded.
 *
 * LINE PID (LINE_PID=1 builds only): the left line module's A0 output, wired
 * to GP28, steers with a PID (line_pid.c) along the tape's left edge. It is
 * calibrated in the same sweep as the barcode: slide the car sideways across
 * the tape. If that calibration fails, the digital steering below is used.
 *
 * Started only by the CAR_RUN startup (vehicle_controller/car_run_target.c).
 */
#include "subsystem3.h"
#include "drv_ir.h"
#include "barcode_sensor.h"
#include "barcode_decoder.h"

#ifndef APP_S3_LINE_PID
#define APP_S3_LINE_PID 0
#endif
#if APP_S3_LINE_PID
#include "line_pid.h"
LOCAL line_pid_t s3_pid;          /* written only by s3_task */
LOCAL uint16_t s3_pid_raw;
#endif
#if APP_CAR_MQTT
#include "../subsystem1/buddy1_bridge.h"
static b1_s3_telemetry s3_wifi;
static uint32_t s3_wifi_at;
static uint32_t s3_scan_state,s3_scan_character,s3_scan_route,s3_scan_sequence,s3_scan_failures,s3_scan_event_ms;
static uint32_t s3_kernel_ms(void) { SYSTIM t; (void)tk_get_otm(&t); return t.lo; }
#endif

#ifndef S3_DEBUG_RAW
#define S3_DEBUG_RAW 0           /* 1 = print raw sensor values every 100 ms */
#endif

#define S3_SAMPLE_MS 1U
#define S3_TASK_PRIORITY 10      /* TBD: agree the schedule as a team */
#define S3_TASK_STACK 1024
#define S3_CALIBRATION_MS 3000U  /* sweep the barcode sensor during this */
/* 1 = no calibration sweep: fixed darkness thresholds, measured on the car
 * 2026-11-10, sensor lowered to ~2-3 mm, mounted 30+ mm left of the line:
 * white paper/mat ~120-350; the ~1.5-2 mm thin bars of the course print peak
 * only ~540-1850 at 70 mm/s; wide bars ~3000-3900. */
#ifndef S3_BARCODE_FIXED
#define S3_BARCODE_FIXED 1
#endif
#define S3_BARCODE_FIXED_LOW 350U
#define S3_BARCODE_FIXED_HIGH 450U
#define S3_ROUTE_VALID_MS 10000U /* TBD: measure barcode-to-junction time */
#define S3_DEBUG_PERIOD_MS 100U

/* Where the task is, for the [S3 TASK] line printed by another task. 0 = asleep,
 * 1..8 = the step it last reached. A frozen counter with a frozen stage shows
 * where the task is stuck. */
static volatile uint32_t s3_dbg_loops;
static volatile uint32_t s3_dbg_stage;
EXPORT void subsystem3_debug(uint32_t *loops, uint32_t *stage)
{
    if (loops != NULL) { *loops = s3_dbg_loops; }
    if (stage != NULL) { *stage = s3_dbg_stage; }
}

LOCAL uint32_t s3_cap_ok = 0U;     /* decoded captures since boot */
LOCAL uint32_t s3_cap_fail = 0U;   /* captures that did not decode */
LOCAL ID s3_tid = E_NOEXS;
LOCAL ID s3_cyc = E_NOEXS;
LOCAL ID s3_mtx = E_NOEXS;

/* Written only by s3_task. Line status and the packed steering sample each
 * use a single aligned store. Events
 * that are read-and-cleared by another task (a read-modify-write) and the
 * request handshakes are protected by s3_mtx. */
LOCAL volatile line_status_t s3_line;
#define S3_STEER_OFFSET 100
#define S3_STEER_ANALOGUE 0x100U
LOCAL volatile uint16_t s3_steering = S3_STEER_OFFSET;
LOCAL route_instruction_t s3_route;
LOCAL BOOL s3_route_pending = FALSE;
LOCAL junction_observation_t s3_junction;
LOCAL BOOL s3_junction_pending = FALSE;
LOCAL command_id_t s3_next_command_id = 1U;

/* Probe handshake with the controller task. */
typedef enum
{
    S3_PROBE_IDLE = 0,
    S3_PROBE_REQUESTED,   /* S3 -> controller: please probe this junction */
    S3_PROBE_BEGIN,       /* controller -> S3: pivot is starting */
    S3_PROBE_RUNNING,
    S3_PROBE_END          /* controller -> S3: pivot is finished */
} s3_probe_state_t;

LOCAL s3_probe_state_t s3_probe = S3_PROBE_IDLE;

/* Reacquire request from the controller (e.g. after an obstacle bypass). */
LOCAL BOOL s3_reacquire_request = FALSE;
LOCAL line_side_t s3_reacquire_hint = LINE_SIDE_NONE;

/* Barcode calibration: requested by startup, run by s3_task alongside line
 * following. s3_barcode_ready is one word written only by s3_task. */
LOCAL BOOL s3_calibrate_request = FALSE;
LOCAL volatile BOOL s3_barcode_ready = FALSE;
/* Capture diagnostics (darkest reading per element, sample gaps, 2 ms
 * waveform dump) cost ~6 KB of RAM: S3_DEBUG_RAW=1 builds only. */
#define S3_CAPTURE_DIAG S3_DEBUG_RAW
#if S3_CAPTURE_DIAG
/* Diagnostics only (s3_task): darkest reading per element of a capture. */
LOCAL uint16_t s3_peak[BARCODE_DECODER_MAX_RUNS];
LOCAL bool s3_peak_busy = false;
LOCAL uint32_t s3_prev_sample_us = 0U;
LOCAL uint32_t s3_max_gap_us = 0U;     /* largest sample gap in a capture */
LOCAL uint32_t s3_capture_samples = 0U;
/* Waveform recorder: darkness every 2 ms of a capture (6 s max). Printed
 * slowly after a failed read so the USB ring never overflows. */
#define S3_WAVE_MAX 3000U
LOCAL uint16_t s3_wave[S3_WAVE_MAX];
LOCAL UINT s3_wave_n = 0U;
LOCAL UINT s3_wave_next = 0U;
LOCAL bool s3_wave_dumping = false;
LOCAL uint32_t s3_wave_last_ms = 0U;
#endif
#if APP_CAR_MQTT
/* Observation only: 0 not started, 1 running, 2 passed, 3 failed, 4 not reported. */
static uint32_t s3_wifi_calibration_state,s3_wifi_calibration_reason;
#endif


LOCAL void s3_cyclic_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(s3_tid);   /* interrupt context: just wake the task */
}

/* One sample of the barcode calibration sweep. Starts on a request from
 * subsystem3_calibrate_barcode() and ends S3_CALIBRATION_MS later; line
 * following carries on meanwhile. Returns true while calibrating: the sample
 * is used for calibration, not decoded. */
LOCAL bool s3_calibrate_step(barcode_sensor_t *bar, barcode_decoder_t *decoder,
                             bool have_raw, uint16_t raw, uint32_t now_ms)
{
    static bool active = false;
    static uint32_t start_ms = 0U;
    BOOL requested;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    requested = s3_calibrate_request;
    s3_calibrate_request = FALSE;
    (void)tk_unl_mtx(s3_mtx);

    if (requested)
    {
        tm_printf((UB *)"[S3] Calibrating: sweep barcode sensor over black and white for %d ms\n",
                  (INT)S3_CALIBRATION_MS);
        barcode_sensor_reset(bar);   /* barcodes ignored until it succeeds */
        barcode_decoder_reset(decoder);
#if APP_S3_LINE_PID
        tm_printf((UB *)"[S3] Line PID: also slide the car sideways across the tape\n");
        line_pid_calibrate_start(&s3_pid);   /* digital steering until it succeeds */
#endif
        s3_barcode_ready = FALSE;
        active = true;
#if APP_CAR_MQTT
        s3_wifi_calibration_state=1U;s3_wifi_calibration_reason=0U;
#endif
        start_ms = now_ms;
    }
    if (!active)
    {
        return false;
    }
    if (have_raw)
    {
        barcode_sensor_calibrate_sample(bar, raw);
    }
#if APP_S3_LINE_PID
    line_pid_calibrate_sample(&s3_pid, s3_pid_raw);
#endif
    if ((now_ms - start_ms) < S3_CALIBRATION_MS)
    {
        return true;
    }

    active = false;
#if APP_S3_LINE_PID
    if (line_pid_calibrate_finish(&s3_pid))
    {
        tm_printf((UB *)"[S3] Line PID OK: white %d black %d\n",
                  (INT)s3_pid.white_level, (INT)s3_pid.black_level);
    }
    else
    {
        tm_printf((UB *)"[S3] Line PID calibration FAILED: white %d black %d, using digital steering\n",
                  (INT)s3_pid.white_level, (INT)s3_pid.black_level);
    }
#endif
    if (barcode_sensor_calibrate_finish(bar))
    {
        s3_barcode_ready = TRUE;
#if APP_CAR_MQTT
        s3_wifi_calibration_state=2U;
#endif
        tm_printf((UB *)"[S3] Barcode OK: white %d black %d, thresholds %d/%d\n",
                  (INT)bar->white_level, (INT)bar->black_level,
                  (INT)bar->threshold_low, (INT)bar->threshold_high);
    }
    else
    {
#if APP_CAR_MQTT
        s3_wifi_calibration_state=3U;s3_wifi_calibration_reason=1U;
#endif
        tm_printf((UB *)"[S3] Barcode calibration FAILED: white %d black %d (contrast too low)\n",
                  (INT)bar->white_level, (INT)bar->black_level);
    }
    return true;
}

LOCAL void s3_publish_junction(const junction_observation_t *junction)
{
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
#if APP_CAR_MQTT
    s3_wifi.junction_counter=(s3_wifi.junction_counter+1U)&65535U;
#endif
    s3_junction = *junction;
    s3_junction_pending = TRUE;
    (void)tk_unl_mtx(s3_mtx);
}

LOCAL void s3_publish_route(route_command_t command, char character, uint32_t now_ms)
{
    route_instruction_t route;

    route.command_id = s3_next_command_id++;
    route.command = command;
    route.decoded_character = character;
    route.decoded_ms = now_ms;
    route.expires_ms = now_ms + S3_ROUTE_VALID_MS;
    route.valid = true;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
#if APP_CAR_MQTT
    s3_wifi.route=(uint32_t)command;
    s3_wifi.barcode_source=character=='-' ? 1U : 0U;
    s3_wifi.scan_valid=1U;
    s3_wifi.route_counter=(s3_wifi.route_counter+1U)&65535U;
#endif
    s3_route = route;
    s3_route_pending = TRUE;
    (void)tk_unl_mtx(s3_mtx);

    tm_printf((UB *)"[S3] Route '%c' -> command %d\n", (INT)character, (INT)command);
}

/* Returns the probe state, applying the controller's begin/end requests. */
LOCAL s3_probe_state_t s3_probe_step(line_follow_t *line, uint32_t now_ms)
{
    s3_probe_state_t state;
    junction_observation_t junction;
    junction_type_t type;
    route_command_t command;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    state = s3_probe;
    if (state == S3_PROBE_BEGIN)
    {
        s3_probe = S3_PROBE_RUNNING;
    }
    else if (state == S3_PROBE_END)
    {
        s3_probe = S3_PROBE_IDLE;
    }
    (void)tk_unl_mtx(s3_mtx);

    if (state == S3_PROBE_BEGIN)
    {
        line_follow_probe_start(line);
        return S3_PROBE_RUNNING;
    }
    if (state == S3_PROBE_END)
    {
        type = line_follow_probe_finish(line);
        command = junction_default_route(type);
        tm_printf((UB *)"[S3] No barcode: junction is %s\n",
                  (type == JUNCTION_TYPE_CROSS) ? "CROSS" : "T");
        if (command != ROUTE_COMMAND_NONE)
        {
            s3_publish_route(command, '-', now_ms);   /* '-' = default, no barcode */
        }
        junction_observation_for_type(&junction, type, now_ms);
        s3_publish_junction(&junction);
        line_follow_reset(line);
        return S3_PROBE_IDLE;
    }
    return state;
}

LOCAL void s3_reacquire_step(line_follow_t *line, uint32_t now_ms)
{
    BOOL requested;
    line_side_t hint;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    requested = s3_reacquire_request;
    hint = s3_reacquire_hint;
    s3_reacquire_request = FALSE;
    (void)tk_unl_mtx(s3_mtx);

    if (requested)
    {
        line_follow_reacquire_start(line, hint, now_ms);
        tm_printf((UB *)"[S3] Reacquiring line, searching side %d\n",
                  (INT)line->status.last_side);
    }
}

LOCAL void s3_task(INT stacd, void *exinf)
{
    line_follow_t line;
    barcode_decoder_t decoder;
    junction_observation_t junction;
    route_command_t command;
    char character;
    bool barcode_seen = false;   /* barcode read since the last junction */
    bool barcode_black = false;
    bool left_black;
    bool right_black;
    uint32_t now_us;
    uint32_t now_ms;
    barcode_sensor_t bar;
    uint16_t raw = 0U;
#if S3_DEBUG_RAW
    uint32_t last_debug_ms = 0U;
#endif
#if APP_S3_LINE_PID
    bool pid_raw_ok = false;
#endif

    (void)stacd;
    (void)exinf;

    line_follow_reset(&line);
#if APP_S3_LINE_PID
    line_pid_init(&s3_pid);   /* digital steering until calibrated */
#endif
    barcode_decoder_reset(&decoder);
    decoder.fail_pending = false;
    decoder.fail_count = 0U;
    decoder.trace_pending = false;
    decoder.trace_count = 0U;
    decoder.trace_id = 0U;
    barcode_sensor_reset(&bar);   /* reads white until calibrated */
#if S3_BARCODE_FIXED && !APP_S3_LINE_PID
    /* No calibration sweep: the fixed thresholds apply from power-on. */
    barcode_sensor_set_fixed(&bar, S3_BARCODE_FIXED_LOW, S3_BARCODE_FIXED_HIGH);
    s3_barcode_ready = TRUE;
#if APP_CAR_MQTT
    s3_wifi_calibration_state = 2U;
#endif
    tm_printf((UB *)"[S3] Barcode ready: fixed thresholds %d/%d (no calibration)\n",
              (INT)bar.threshold_low, (INT)bar.threshold_high);
#endif

    for (;;)
    {
        int8_t steer = 0;
        bool analogue_steering = false;
        s3_dbg_stage = 0U;            /* asleep, waiting for the 1 ms tick */
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);   /* drop missed wake-ups, no catch-up burst */
        s3_dbg_loops++;
        s3_dbg_stage = 1U;

        now_us = drv_ir_now_us();
        now_ms = now_us / 1000U;

        /* ---- Requests from the controller ---- */
        s3_reacquire_step(&line, now_ms);

        /* ---- Line ---- */
        s3_dbg_stage = 2U;            /* line sensors, steering */
        drv_ir_read_line(&left_black, &right_black);
#if APP_S3_LINE_PID
        /* A failed read uses digital steering for this sample only. */
        pid_raw_ok = (drv_ir_read_line_analogue(&s3_pid_raw) == E_OK);
#endif

        if (s3_probe_step(&line, now_ms) == S3_PROBE_RUNNING)
        {
            /* Car is pivoting for the probe: watch, do not steer. */
            line_follow_probe_sample(&line, left_black, right_black);
        }
        else
        {
            (void)line_follow_update(&line, left_black, right_black, now_ms);
            steer = line_follow_steer(&line, now_ms);
#if APP_S3_LINE_PID
            /* PID steers only while plainly following. Junction, reacquire
             * and line loss keep the digital behaviour. */
            switch ((line_state_t)line.status.state)
            {
                case LINE_STATE_CENTRED:
                case LINE_STATE_STEER_LEFT:
                case LINE_STATE_STEER_RIGHT:
                    if (s3_pid.calibrated && pid_raw_ok)
                    {
                        /* On a bend the sensor runs off the edge; the
                         * digital ramp then takes over if it steers harder. */
                        int8_t pid_steer = line_pid_step(&s3_pid, s3_pid_raw, now_ms);
                        steer = line_pid_blend(pid_steer, steer,
                            line_pid_saturated(&s3_pid, s3_pid_raw));
                        analogue_steering = steer == pid_steer;
                    }
                    break;
                default:
                    line_pid_reset(&s3_pid);
                    break;
            }
#endif
        }

        s3_dbg_stage = 3U;            /* junction handling */
        if (line_follow_take_junction(&line, &junction, now_ms))
        {
            if (barcode_seen)
            {
                s3_publish_junction(&junction);   /* barcode route decides */
            }
            else
            {
                /* No barcode: report "junction ahead" (not confirmed yet) and
                 * ask the controller to probe so we know "+" or T. */
                junction_observation_for_type(&junction, JUNCTION_TYPE_UNKNOWN, now_ms);
                s3_publish_junction(&junction);
                (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
                s3_probe = S3_PROBE_REQUESTED;
                (void)tk_unl_mtx(s3_mtx);
            }
            barcode_seen = false;
        }

        /* ---- Barcode ---- */
        {
            s3_dbg_stage = 5U;        /* reading the barcode input (ADC) */
            bool have_raw = (drv_ir_read_barcode(&raw) == E_OK);
            s3_dbg_stage = 6U;        /* classify and decode */
#if APP_CAR_MQTT
            s3_wifi.adc_valid=have_raw;
            s3_wifi.adc=have_raw ? raw : 0U;
#endif

            if (!s3_calibrate_step(&bar, &decoder, have_raw, raw, now_ms) &&
                have_raw)
            {
                barcode_black = barcode_sensor_is_black(&bar, raw);
                barcode_decoder_feed(&decoder, barcode_black, now_us);
#if S3_CAPTURE_DIAG
                /* Diagnostics: darkest reading inside each element of the
                 * capture, printed if it fails. A "white" element hiding a
                 * missed bar shows as a raised peak. */
                if (barcode_decoder_busy(&decoder))
                {
                    uint16_t dark = barcode_sensor_darkness(raw);
                    if (!s3_peak_busy)
                    {
                        UINT k;
                        for (k = 0U; k < BARCODE_DECODER_MAX_RUNS; k++)
                        {
                            s3_peak[k] = 0U;
                        }
                        s3_peak_busy = true;
                        if (!s3_wave_dumping)
                        {
                            s3_wave_n = 0U;
                        }
                        s3_max_gap_us = 0U;
                        s3_capture_samples = 0U;
                    }
                    else if ((now_us - s3_prev_sample_us) > s3_max_gap_us)
                    {
                        s3_max_gap_us = now_us - s3_prev_sample_us;
                    }
                    if (((s3_capture_samples & 1U) == 0U) && !s3_wave_dumping &&
                        (s3_wave_n < S3_WAVE_MAX))
                    {
                        s3_wave[s3_wave_n++] = dark;
                    }
                    s3_capture_samples++;
                    if ((decoder.run_count < BARCODE_DECODER_MAX_RUNS) &&
                        (dark > s3_peak[decoder.run_count]))
                    {
                        s3_peak[decoder.run_count] = dark;
                    }
                }
                else
                {
                    s3_peak_busy = false;
                }
                s3_prev_sample_us = now_us;
#endif
            }
        }
#if APP_CAR_MQTT
        /* Duration is copied from the actual most recent completed run. */
        if(decoder.run_count>0U){
            uint32_t ms=decoder.run_us[decoder.run_count-1U]/1000U;
            if(ms<=6000U) s3_wifi.bar_ms=ms;
        }
        /* take() resets the decoder; retain direction before that reset. */
        if (decoder.state == BARCODE_DECODER_COMPLETE) {
            s3_wifi.reversed=barcode_decoder_was_reversed(&decoder);
            s3_wifi.scan_counter=(s3_wifi.scan_counter+1U)&65535U;
        }
#endif
        s3_dbg_stage = 7U;            /* results and trace */
        if (barcode_decoder_take(&decoder, &command, &character))
        {
            s3_cap_ok++;
#if APP_CAR_MQTT
            /* 2 decoded and mapped, 4 decoded but no action assigned. */
            s3_scan_state = command == ROUTE_COMMAND_NONE ? 4U : 2U;
            s3_scan_character = (uint8_t)character;
            s3_scan_route = command;
            s3_scan_sequence++;
            s3_scan_event_ms = s3_kernel_ms();
#endif
            if (decoder.reversed)
            {
                tm_printf((UB *)"[S3] (barcode read in reverse)\n");
            }
            if (command == ROUTE_COMMAND_NONE)
            {
                /* Good read, but nobody mapped this letter: no route. The
                 * junction is then handled as if there was no barcode. */
                tm_printf((UB *)"[S3] Barcode '%c' read, but it is not mapped (send MAP over MQTT)\n",
                          (INT)character);
            }
            else
            {
                s3_publish_route(command, character, now_ms);
                barcode_seen = true;
            }
        }

        /* One line per finished capture, microseconds, for tools/barcode_replay.py.
         * ok=1 means the decoder accepted it (see the "[S3] Barcode" lines for
         * the letter). Printed after the capture so it cannot delay sampling. */
        if (decoder.trace_pending)
        {
            UINT t;
            decoder.trace_pending = false;
            if (!decoder.trace_ok) { s3_cap_fail++; }
            tm_printf((UB *)"[S3 CAP] id=%d ok=%d n=%d pass=%d fail=%d us=",
                      (INT)decoder.trace_id, (INT)decoder.trace_ok,
                      (INT)decoder.trace_count, (INT)s3_cap_ok, (INT)s3_cap_fail);
            for (t = 0U; t < decoder.trace_count; t++)
            {
                tm_printf((UB *)(t == 0U ? "%d" : ",%d"),
                          (INT)decoder.trace_run_us[t]);
            }
            tm_printf((UB *)"\n");
        }

        /* A capture that did not decode: print its element widths (ms) so
         * the cause is visible. Few runs = a tape crossing, not a barcode. */
        if (decoder.fail_pending)
        {
            decoder.fail_pending = false;
#if APP_CAR_MQTT
            /* Every capture gets a visible outcome, including a short tape
             * crossing. This is a failed attempt, not proof of a barcode. */
            s3_scan_state = 3U;
            s3_scan_character = 0U;
            s3_scan_route = 0U;
            s3_scan_sequence++;
            s3_scan_failures++;
            s3_scan_event_ms = s3_kernel_ms();
#endif
            if (decoder.fail_count >= 5U)
            {
                UINT i;
                tm_printf((UB *)"[S3] Barcode NOT decoded: %d elements (need 29), ms:",
                          (INT)decoder.fail_count);
                for (i = 0U; i < decoder.fail_count; i++)
                {
                    tm_printf((UB *)" %d", (INT)(decoder.fail_run_us[i] / 1000U));
                }
                tm_printf((UB *)"\n");
#if S3_CAPTURE_DIAG
                tm_printf((UB *)"[S3]   darkest per element:");
                for (i = 0U; i < decoder.fail_count; i++)
                {
                    tm_printf((UB *)" %d", (INT)s3_peak[i]);
                }
                tm_printf((UB *)"\n[S3]   samples %lu, largest gap between samples %lu us\n",
                          (unsigned long)s3_capture_samples,
                          (unsigned long)s3_max_gap_us);
                if (!s3_wave_dumping && (s3_wave_n > 0U))
                {
                    tm_printf((UB *)"[S3 WAVE] begin n=%d step_ms=2\n", (INT)s3_wave_n);
                    s3_wave_next = 0U;
                    s3_wave_dumping = true;
                    s3_wave_last_ms = now_ms;
                }
#endif
            }
        }
#if S3_CAPTURE_DIAG
        /* Drip the recorded waveform out, 20 values every 20 ms. */
        if (s3_wave_dumping && ((now_ms - s3_wave_last_ms) >= 20U))
        {
            UINT j;
            s3_wave_last_ms = now_ms;
            tm_printf((UB *)"[S3 WAVE] %d:", (INT)s3_wave_next);
            for (j = 0U; (j < 20U) && (s3_wave_next < s3_wave_n); j++)
            {
                tm_printf((UB *)" %d", (INT)s3_wave[s3_wave_next++]);
            }
            tm_printf((UB *)"\n");
            if (s3_wave_next >= s3_wave_n)
            {
                tm_printf((UB *)"[S3 WAVE] end\n");
                s3_wave_dumping = false;
            }
        }
#endif

        s3_dbg_stage = 8U;            /* publish status */
        /* ---- Publish line status and steering ---- */
#if APP_CAR_MQTT
        /* Show "Reading" from the first captured bar. It may be a barcode
         * or a tape crossing; only a successful decode confirms a letter. */
        if (barcode_decoder_busy(&decoder))
        {
            if (s3_scan_state != 1U)
            {
                s3_scan_state = 1U;
                s3_scan_event_ms = s3_kernel_ms();
            }
        }
        else if (s3_scan_state == 1U)
        {
            /* The decoder dropped the capture silently (black longer than any
             * bar = tape). No result was produced, so go back to "ready"
             * instead of showing "Reading" forever. */
            s3_scan_state = 0U;
            s3_scan_event_ms = s3_kernel_ms();
        }
#endif
        line_follow_set_barcode_active(&line, barcode_decoder_busy(&decoder));
        s3_line = line.status;
        s3_steering = (uint16_t)((unsigned)(steer + S3_STEER_OFFSET) |
            (analogue_steering ? S3_STEER_ANALOGUE : 0U));
#if APP_CAR_MQTT
        {
            uint32_t at=s3_kernel_ms();
            if ((uint32_t)(at-s3_wifi_at)>=100U) {
                s3_wifi.left_black=line.status.left_black;
                s3_wifi.right_black=line.status.right_black;
                s3_wifi.line_state=line.status.state;
                s3_wifi.last_side=line.status.last_side;
                s3_wifi.steering=steer;
                s3_wifi.black=barcode_black;
                s3_wifi.calibrated=s3_barcode_ready!=FALSE;
                s3_wifi.decoder_state=decoder.state;
                s3_wifi.calibration_low=bar.threshold_low;
                s3_wifi.calibration_high=bar.threshold_high;
                /* Pattern is unavailable: do not invent a decoded mask. */
                b1_live_line report={.at_ms=at,.calibration_state=s3_wifi_calibration_state,
                    .calibration_reason=s3_wifi_calibration_reason,
                    .barcode_state=s3_scan_state,.barcode_character=s3_scan_character,
                    .barcode_route=s3_scan_route,.barcode_sequence=s3_scan_sequence,
                    .barcode_failures=s3_scan_failures,.barcode_event_ms=s3_scan_event_ms,
                    .barcode_digital=0};
                (void)b1_submit_live_status(3,&report);
                if(b1_submit_section(3,&s3_wifi,at)) s3_wifi_at=at;
            }
        }
#endif

#if S3_DEBUG_RAW
        if ((now_ms - last_debug_ms) >= S3_DEBUG_PERIOD_MS)
        {
            last_debug_ms = now_ms;
            tm_printf((UB *)"[S3 RAW] L=%d R=%d steer=%d adc=%d dark=%d thr=%d/%d\n",
                      (INT)left_black, (INT)right_black, (INT)steer,
                      (INT)raw, (INT)barcode_sensor_darkness(raw),
                      (INT)bar.threshold_low, (INT)bar.threshold_high);
#if APP_S3_LINE_PID
            tm_printf((UB *)"[S3 PID] adc=%d dark=%d err=%d cal=%d\n",
                      (INT)s3_pid_raw, (INT)line_pid_darkness(s3_pid_raw),
                      (INT)line_pid_error(&s3_pid, s3_pid_raw),
                      (INT)s3_pid.calibrated);
#endif
        }
#endif
    }
}

EXPORT ER subsystem3_init(void)
{
    T_CMTX cmtx = {0};
    T_CTSK ctsk = {0};
    T_CCYC ccyc = {0};
    ER er;

    er = drv_ir_init();
    if (er < E_OK)
    {
        return er;
    }

    cmtx.mtxatr = TA_INHERIT;   /* priority inheritance avoids priority inversion */
    s3_mtx = tk_cre_mtx(&cmtx);
    if (s3_mtx < E_OK)
    {
        return s3_mtx;
    }

    ctsk.tskatr = TA_HLNG | TA_RNG0;
    ctsk.task = (FP)s3_task;
    ctsk.itskpri = S3_TASK_PRIORITY;
    ctsk.stksz = S3_TASK_STACK;
    s3_tid = tk_cre_tsk(&ctsk);
    if (s3_tid < E_OK)
    {
        return s3_tid;
    }

    ccyc.cycatr = TA_HLNG;          /* created stopped; started in start() */
    ccyc.cychdr = (FP)s3_cyclic_handler;
    ccyc.cyctim = S3_SAMPLE_MS;
    ccyc.cycphs = S3_SAMPLE_MS;
    s3_cyc = tk_cre_cyc(&ccyc);
    return (s3_cyc < E_OK) ? s3_cyc : E_OK;
}

EXPORT ER subsystem3_start(void)
{
    ER er = tk_sta_tsk(s3_tid, 0);
    if (er < E_OK)
    {
        return er;
    }
    return tk_sta_cyc(s3_cyc);
}

/* ---- Outputs ---------------------------------------------------------- */

EXPORT line_status_t subsystem3_line_status(void)
{
    return s3_line;
}

EXPORT int8_t subsystem3_steer(bool *analogue)
{
    uint16_t sample = s3_steering;
    if (analogue != NULL) { *analogue = (sample & S3_STEER_ANALOGUE) != 0U; }
    return (int8_t)((int)(sample & 0xffU) - S3_STEER_OFFSET);
}

EXPORT BOOL subsystem3_take_route(route_instruction_t *route)
{
    BOOL taken = FALSE;

    if (route == NULL)
    {
        return FALSE;
    }
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    if (s3_route_pending)
    {
        *route = s3_route;
        s3_route_pending = FALSE;
        taken = TRUE;
    }
    (void)tk_unl_mtx(s3_mtx);
    return taken;
}

EXPORT BOOL subsystem3_take_junction(junction_observation_t *junction)
{
    BOOL taken = FALSE;

    if (junction == NULL)
    {
        return FALSE;
    }
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    if (s3_junction_pending)
    {
        *junction = s3_junction;
        s3_junction_pending = FALSE;
        taken = TRUE;
    }
    (void)tk_unl_mtx(s3_mtx);
    return taken;
}

/* ---- Requests from the controller ------------------------------------- */

EXPORT BOOL subsystem3_probe_requested(void)
{
    BOOL requested;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    requested = (s3_probe == S3_PROBE_REQUESTED) ? TRUE : FALSE;
    (void)tk_unl_mtx(s3_mtx);
    return requested;
}

EXPORT ER subsystem3_probe_begin(void)
{
    ER er = E_OBJ;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    if (s3_probe == S3_PROBE_REQUESTED)
    {
        s3_probe = S3_PROBE_BEGIN;
        er = E_OK;
    }
    (void)tk_unl_mtx(s3_mtx);
    return er;
}

EXPORT ER subsystem3_probe_end(void)
{
    ER er = E_OBJ;

    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    if (s3_probe == S3_PROBE_RUNNING)
    {
        s3_probe = S3_PROBE_END;
        er = E_OK;
    }
    (void)tk_unl_mtx(s3_mtx);
    return er;
}

EXPORT void subsystem3_reacquire(line_side_t side_hint)
{
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    s3_reacquire_request = TRUE;
    s3_reacquire_hint = side_hint;
    (void)tk_unl_mtx(s3_mtx);
}

EXPORT void subsystem3_calibrate_barcode(void)
{
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    s3_calibrate_request = TRUE;
    (void)tk_unl_mtx(s3_mtx);
}

EXPORT BOOL subsystem3_barcode_calibrated(void)
{
    return s3_barcode_ready;
}

