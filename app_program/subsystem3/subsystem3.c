/* Buddy 3: line + barcode acquisition task. DRAFT, not yet run on the car.
 *
 * Data flow, once per sample period:
 *
 *   cyclic handler --wake--> s3_task
 *     |- apply requests from the controller (probe, reacquire)
 *     |- drv_ir_read_line()    -> line_follow_update()  -> steering -100..100
 *     |- barcode: ADC sample   -> barcode_sensor_is_black() -> decoder
 *     |       or: edge buffer  (S3_BARCODE_USE_EDGES)        -> decoder
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
 * setting). With S3_BARCODE_USE_EDGES = 1 the barcode is timed by GPIO edge
 * interrupts instead and a 10 ms period is enough.
 *
 * BARCODE CALIBRATION runs only when asked (subsystem3_calibrate_barcode(),
 * called by the CAR_RUN startup at its "sweep now" prompt): 3 s of samples
 * set the black/white thresholds, while line following carries on. Until a
 * calibration succeeds every sample reads white, so no barcode is decoded.
 *
 * Started only by the CAR_RUN startup (vehicle_controller/car_run_target.c).
 */
#include "subsystem3.h"
#include "drv_ir.h"
#include "barcode_sensor.h"
#include "barcode_decoder.h"
#include "barcode_edges.h"
#if APP_CAR_MQTT
#include "../subsystem1/buddy1_bridge.h"
static b1_s3_telemetry s3_wifi;
static uint32_t s3_wifi_at;
static uint32_t s3_kernel_ms(void) { SYSTIM t; (void)tk_get_otm(&t); return t.lo; }
#endif

#ifndef S3_BARCODE_USE_EDGES
#define S3_BARCODE_USE_EDGES 0   /* 1 = digital D0 + GPIO edge interrupts */
#endif
#ifndef S3_DEBUG_RAW
#define S3_DEBUG_RAW 0           /* 1 = print raw sensor values every 100 ms */
#endif

#if S3_BARCODE_USE_EDGES
#define S3_SAMPLE_MS 10U
#else
#define S3_SAMPLE_MS 1U
#endif
#define S3_TASK_PRIORITY 10      /* TBD: agree the schedule as a team */
#define S3_TASK_STACK 1024
#define S3_CALIBRATION_MS 3000U  /* sweep the barcode sensor during this */
#define S3_ROUTE_VALID_MS 10000U /* TBD: measure barcode-to-junction time */
#define S3_DEBUG_PERIOD_MS 100U

LOCAL ID s3_tid = E_NOEXS;
LOCAL ID s3_cyc = E_NOEXS;
LOCAL ID s3_mtx = E_NOEXS;

/* Written only by s3_task. line_status is one byte and the steering value is
 * one byte, so each is stored and read atomically without a lock. Events
 * that are read-and-cleared by another task (a read-modify-write) and the
 * request handshakes are protected by s3_mtx. */
LOCAL volatile line_status_t s3_line;
LOCAL volatile int8_t s3_steer_value;
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
LOCAL volatile uint32_t s3_raw_sample; /* bit16 valid, bits11..0 ADC; one store */

bool subsystem3_barcode_raw(uint16_t *raw)
{
    uint32_t sample = s3_raw_sample;
    if (raw == NULL || (sample & 0x10000U) == 0U) { return false; }
    *raw = (uint16_t)(sample & 0xFFFU);
    return true;
}

#if S3_BARCODE_USE_EDGES
LOCAL barcode_edges_t s3_edges;
#endif

LOCAL void s3_cyclic_handler(void *exinf)
{
    (void)exinf;
    (void)tk_wup_tsk(s3_tid);   /* interrupt context: just wake the task */
}

#if !S3_BARCODE_USE_EDGES
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
        s3_barcode_ready = FALSE;
        active = true;
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
    if ((now_ms - start_ms) < S3_CALIBRATION_MS)
    {
        return true;
    }

    active = false;
    if (barcode_sensor_calibrate_finish(bar))
    {
        s3_barcode_ready = TRUE;
        tm_printf((UB *)"[S3] Barcode OK: white %d black %d, thresholds %d/%d\n",
                  (INT)bar->white_level, (INT)bar->black_level,
                  (INT)bar->threshold_low, (INT)bar->threshold_high);
    }
    else
    {
        tm_printf((UB *)"[S3] Barcode calibration FAILED: white %d black %d (contrast too low)\n",
                  (INT)bar->white_level, (INT)bar->black_level);
    }
    return true;
}
#endif

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
#if S3_BARCODE_USE_EDGES
    uint32_t edge_us;
#else
    barcode_sensor_t bar;
    uint16_t raw = 0U;
#endif
#if S3_DEBUG_RAW
    uint32_t last_debug_ms = 0U;
#endif

    (void)stacd;
    (void)exinf;

    line_follow_reset(&line);
    barcode_decoder_reset(&decoder);
#if !S3_BARCODE_USE_EDGES
    barcode_sensor_reset(&bar);   /* reads white until calibrated */
#endif

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        (void)tk_can_wup(TSK_SELF);   /* drop missed wake-ups, no catch-up burst */

        now_us = drv_ir_now_us();
        now_ms = now_us / 1000U;

        /* ---- Requests from the controller ---- */
        s3_reacquire_step(&line, now_ms);

        /* ---- Line ---- */
        drv_ir_read_line(&left_black, &right_black);

        if (s3_probe_step(&line, now_ms) == S3_PROBE_RUNNING)
        {
            /* Car is pivoting for the probe: watch, do not steer. */
            line_follow_probe_sample(&line, left_black, right_black);
            s3_steer_value = 0;
        }
        else
        {
            (void)line_follow_update(&line, left_black, right_black, now_ms);
            s3_steer_value = line_follow_steer(&line, now_ms);
        }

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
#if S3_BARCODE_USE_EDGES
        /* Exact edge times from the ISR, then one call at "now" so the
         * decoder can notice the quiet zone that ends the barcode. */
        while (barcode_edges_pop(&s3_edges, &barcode_black, &edge_us))
        {
            barcode_decoder_feed(&decoder, barcode_black, edge_us);
        }
        if (s3_edges.overflow != 0U)
        {
            s3_edges.overflow = 0U;
            barcode_decoder_reset(&decoder);   /* lost edges: discard read */
        }
        barcode_decoder_feed(&decoder, barcode_black, now_us);
#else
        {
            bool have_raw = (drv_ir_read_barcode(&raw) == E_OK);
            s3_raw_sample = have_raw ? (0x10000U | raw) : 0U;
#if APP_CAR_MQTT
            s3_wifi.adc_valid=have_raw;
            s3_wifi.adc=have_raw ? raw : 0U;
#endif

            if (!s3_calibrate_step(&bar, &decoder, have_raw, raw, now_ms) &&
                have_raw)
            {
                barcode_black = barcode_sensor_is_black(&bar, raw);
                barcode_decoder_feed(&decoder, barcode_black, now_us);
            }
        }
#endif
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
        if (barcode_decoder_take(&decoder, &command, &character))
        {
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

        /* ---- Publish: build locally, store once (atomic byte) ---- */
        line_follow_set_barcode_active(&line, barcode_decoder_busy(&decoder));
        s3_line = line.status;
#if APP_CAR_MQTT
        {
            uint32_t at=s3_kernel_ms();
            if ((uint32_t)(at-s3_wifi_at)>=100U) {
                s3_wifi.left_black=line.status.left_black;
                s3_wifi.right_black=line.status.right_black;
                s3_wifi.line_state=line.status.state;
                s3_wifi.last_side=line.status.last_side;
                s3_wifi.steering=s3_steer_value;
                s3_wifi.black=barcode_black;
                s3_wifi.calibrated=s3_barcode_ready!=FALSE;
                s3_wifi.decoder_state=decoder.state;
#if !S3_BARCODE_USE_EDGES
                s3_wifi.calibration_low=bar.threshold_low;
                s3_wifi.calibration_high=bar.threshold_high;
#endif
                /* Pattern is unavailable: do not invent a decoded mask. */
                if(b1_submit_section(3,&s3_wifi,at)) s3_wifi_at=at;
            }
        }
#endif

#if S3_DEBUG_RAW
        if ((now_ms - last_debug_ms) >= S3_DEBUG_PERIOD_MS)
        {
            last_debug_ms = now_ms;
#if S3_BARCODE_USE_EDGES
            tm_printf((UB *)"[S3 RAW] L=%d R=%d steer=%d barcode=%d\n",
                      (INT)left_black, (INT)right_black,
                      (INT)s3_steer_value, (INT)barcode_black);
#else
            tm_printf((UB *)"[S3 RAW] L=%d R=%d steer=%d adc=%d dark=%d thr=%d/%d\n",
                      (INT)left_black, (INT)right_black, (INT)s3_steer_value,
                      (INT)raw, (INT)barcode_sensor_darkness(raw),
                      (INT)bar.threshold_low, (INT)bar.threshold_high);
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
#if S3_BARCODE_USE_EDGES
    barcode_edges_reset(&s3_edges);
#endif

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

EXPORT int8_t subsystem3_steer(void)
{
    return s3_steer_value;
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
#if !S3_BARCODE_USE_EDGES
    (void)tk_loc_mtx(s3_mtx, TMO_FEVR);
    s3_calibrate_request = TRUE;
    (void)tk_unl_mtx(s3_mtx);
#endif
}

EXPORT BOOL subsystem3_barcode_calibrated(void)
{
#if S3_BARCODE_USE_EDGES
    return TRUE;   /* digital D0: thresholded by the sensor itself */
#else
    return s3_barcode_ready;
#endif
}

#if S3_BARCODE_USE_EDGES
/* Called by the team's shared GPIO interrupt dispatcher on every edge of
 * the barcode D0 pin. Interrupt context: no waiting, no printing. */
EXPORT void subsystem3_barcode_edge_isr(void)
{
    (void)barcode_edges_push(&s3_edges, drv_ir_read_barcode_digital(),
                             drv_ir_now_us());
}
#endif
