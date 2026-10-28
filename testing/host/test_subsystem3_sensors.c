/* Host tests for Buddy 3 (S3-H01 .. S3-H14, see testing/subsystem3/TEST_LOG.md).
 * No hardware: sensor readings and time are simulated.
 */
#include <assert.h>
#include <stdio.h>

#include "../../app_program/subsystem3/line_follow.h"
#include "../../app_program/subsystem3/barcode_sensor.h"
#include "../../app_program/subsystem3/barcode_decoder.h"
#include "../../app_program/subsystem3/barcode_edges.h"
#include "../../app_program/subsystem3/barcode_map.h"
#include <string.h>
#include "../../app_program/vehicle_controller/vehicle_controller.h"

/* ---- helpers ---------------------------------------------------------- */

/* Feed the same reading for "ms" milliseconds, one sample per ms. */
static uint32_t hold(line_follow_t *lf, bool l, bool r, uint32_t t, uint32_t ms)
{
    uint32_t end = t + ms;
    for (; t < end; t++)
    {
        (void)line_follow_update(lf, l, r, t);
    }
    return t;
}

static const uint16_t CODE39[4] = {0x109U, 0x049U, 0x148U, 0x019U};  /* A B C D */
static const route_command_t COMMANDS[4] = {
    ROUTE_COMMAND_LEFT, ROUTE_COMMAND_RIGHT,
    ROUTE_COMMAND_STRAIGHT, ROUTE_COMMAND_UTURN};

/* Element widths (us) of *X*, black first, 29 elements. */
static void build_symbol(uint16_t data, uint32_t narrow, uint32_t wide, uint32_t *w)
{
    const uint16_t p[3] = {0x094U, data, 0x094U};
    uint8_t k = 0U, s, b;
    for (s = 0U; s < 3U; s++)
    {
        for (b = 0U; b < 9U; b++)
        {
            w[k++] = ((p[s] >> (8U - b)) & 1U) ? wide : narrow;
        }
        if (s < 2U)
        {
            w[k++] = narrow;
        }
    }
}

/* Drive simulated ADC readings over a barcode, sampled every 500 us. */
static bool scan(const uint32_t *w, route_command_t *cmd, char *ch, bool *reversed)
{
    barcode_sensor_t s;
    barcode_decoder_t d;
    uint32_t now = 1U, end;
    uint8_t i;

    barcode_sensor_reset(&s);
    barcode_sensor_calibrate_sample(&s, 800U);
    barcode_sensor_calibrate_sample(&s, 3200U);
    assert(barcode_sensor_calibrate_finish(&s));
    barcode_decoder_reset(&d);

    for (end = now + 200000U; now < end; now += 500U)
    {
        barcode_decoder_feed(&d, barcode_sensor_is_black(&s, 800U), now);
    }
    for (i = 0U; i < 29U; i++)
    {
        uint16_t raw = ((i % 2U) == 0U) ? 3200U : 800U;
        for (end = now + w[i]; now < end; now += 500U)
        {
            barcode_decoder_feed(&d, barcode_sensor_is_black(&s, raw), now);
            if (i == 14U)
            {
                assert(barcode_decoder_busy(&d));   /* mid-barcode: busy */
            }
        }
    }
    for (end = now + 400000U; now < end; now += 500U)
    {
        barcode_decoder_feed(&d, barcode_sensor_is_black(&s, 800U), now);
    }
    if (reversed != NULL)
    {
        *reversed = d.reversed;
    }
    return barcode_decoder_take(&d, cmd, ch);
}

static route_instruction_t make_route(route_command_t cmd, char ch, uint32_t now)
{
    route_instruction_t r = {0};
    r.command_id = 1U;
    r.command = cmd;
    r.decoded_character = ch;
    r.decoded_ms = now;
    r.expires_ms = now + 10000U;
    r.valid = true;
    return r;
}

/* ---- S3-H01 ----------------------------------------------------------- */
static void test_status_is_one_byte(void)
{
    assert(sizeof(line_status_t) == 1U);
}

/* ---- S3-H02 ----------------------------------------------------------- */
static void test_majority_filter(void)
{
    line_follow_t lf;
    uint32_t t;

    line_follow_reset(&lf);
    t = hold(&lf, false, false, 0U, 10U);
    (void)line_follow_update(&lf, true, false, t++);     /* 1-sample glitch */
    t = hold(&lf, false, false, t, 5U);
    assert(lf.status.state == LINE_STATE_CENTRED);
    assert(lf.status.last_side == LINE_SIDE_NONE);

    t = hold(&lf, true, false, t, 2U);                  /* 2 samples: real */
    assert(lf.status.state == LINE_STATE_STEER_LEFT);
    assert(lf.status.left_black == 1U);
}

/* ---- S3-H03 ----------------------------------------------------------- */
static void test_steering_ramp_and_damping(void)
{
    line_follow_t lf;
    uint32_t t, start;
    int8_t early, later;

    line_follow_reset(&lf);
    t = hold(&lf, false, false, 0U, 10U);
    assert(line_follow_steer(&lf, t) == 0);

    t = hold(&lf, true, false, t, 2U);                  /* drift: steer left */
    start = lf.state_since_ms;
    early = line_follow_steer(&lf, start);
    t = hold(&lf, true, false, t, 60U);
    later = line_follow_steer(&lf, t);
    assert(early == -LINE_STEER_BASE);
    assert(later < early);                              /* stronger left */
    t = hold(&lf, true, false, t, 500U);
    assert(line_follow_steer(&lf, t) == -LINE_STEER_MAX);   /* capped */

    t = hold(&lf, false, false, t, 2U);                 /* back on centre */
    assert(line_follow_steer(&lf, t) == LINE_DAMP_STEER);   /* counter-steer */
    t = hold(&lf, false, false, t, LINE_DAMP_MS + 5U);
    assert(line_follow_steer(&lf, t) == 0);             /* damping over */

    t = hold(&lf, false, true, t, 2U);                  /* mirror: right */
    assert(line_follow_steer(&lf, t) > 0);
}

/* ---- S3-H04 ----------------------------------------------------------- */
static void test_junction_debounce_and_single_report(void)
{
    line_follow_t lf;
    junction_observation_t obs;
    uint32_t t;

    line_follow_reset(&lf);
    t = hold(&lf, false, false, 0U, 10U);
    t = hold(&lf, true, true, t, 5U);                   /* short blip */
    t = hold(&lf, false, false, t, 10U);
    assert(!line_follow_take_junction(&lf, &obs, t));

    t = hold(&lf, true, true, t, 100U);                 /* real crossbar */
    assert(lf.status.state == LINE_STATE_JUNCTION);
    assert(line_follow_take_junction(&lf, &obs, t));
    assert(obs.confirmed && obs.left_available && obs.right_available);
    assert(obs.straight_available);
    assert(!line_follow_take_junction(&lf, &obs, t));   /* once only */

    t = hold(&lf, false, false, t, 50U);
    t = hold(&lf, true, true, t, 40U);                  /* next junction */
    assert(line_follow_take_junction(&lf, &obs, t));
}

/* ---- S3-H05 ----------------------------------------------------------- */
static void test_barcode_calibration_and_hysteresis(void)
{
    barcode_sensor_t s;

    barcode_sensor_reset(&s);
    assert(!barcode_sensor_is_black(&s, 4000U));        /* not calibrated */
    barcode_sensor_calibrate_sample(&s, 1000U);
    barcode_sensor_calibrate_sample(&s, 3000U);
    assert(barcode_sensor_calibrate_finish(&s));
    assert(s.threshold_low == 1800U && s.threshold_high == 2200U);

    assert(!barcode_sensor_is_black(&s, 2100U));        /* in band: white */
    assert(barcode_sensor_is_black(&s, 2300U));         /* above high */
    assert(barcode_sensor_is_black(&s, 1900U));         /* in band: black */
    assert(!barcode_sensor_is_black(&s, 1700U));        /* below low */

    barcode_sensor_reset(&s);
    barcode_sensor_calibrate_sample(&s, 2000U);
    barcode_sensor_calibrate_sample(&s, 2100U);
    assert(!barcode_sensor_calibrate_finish(&s));       /* low contrast */
}

/* ---- S3-H06 ----------------------------------------------------------- */
static void test_barcode_adaptation(void)
{
    barcode_sensor_t s;
    uint16_t i;
    uint16_t first_low;

    barcode_sensor_reset(&s);
    barcode_sensor_calibrate_sample(&s, 1000U);
    barcode_sensor_calibrate_sample(&s, 3000U);
    assert(barcode_sensor_calibrate_finish(&s));
    first_low = s.threshold_low;

    /* Room gets brighter: white foam now reads 600 instead of 1000. */
    for (i = 0U; i < 1000U; i++)
    {
        (void)barcode_sensor_is_black(&s, 600U);
    }
    assert(s.white_level < 700U);                       /* followed the drift */
    assert(s.threshold_low < first_low);
    assert(barcode_sensor_is_black(&s, 2900U));         /* black still works */

    /* Readings collapsing towards black must not destroy the contrast. */
    for (i = 0U; i < 5000U; i++)
    {
        (void)barcode_sensor_is_black(&s, 2900U);
    }
    assert((uint32_t)(s.black_level - s.white_level) >= BARCODE_MIN_CONTRAST);
}

/* ---- S3-H07 ----------------------------------------------------------- */
static void test_decode_all_letters_both_directions_all_speeds(void)
{
    static const uint32_t speeds[5] = {50U, 100U, 150U, 200U, 300U};   /* mm/s */
    uint32_t w[29], r[29];
    route_command_t cmd;
    char ch;
    bool reversed;
    uint8_t letter, sp, k;

    for (sp = 0U; sp < 5U; sp++)
    {
        uint32_t narrow = 3000000U / speeds[sp];        /* 3 mm bar, in us */
        for (letter = 0U; letter < 4U; letter++)
        {
            build_symbol(CODE39[letter], narrow, narrow * 3U, w);
            assert(scan(w, &cmd, &ch, &reversed));
            assert((ch == (char)('A' + letter)) && (cmd == COMMANDS[letter]));
            assert(!reversed);

            for (k = 0U; k < 29U; k++)
            {
                r[k] = w[28U - k];                       /* crossed backwards */
            }
            assert(scan(r, &cmd, &ch, &reversed));
            assert((ch == (char)('A' + letter)) && (cmd == COMMANDS[letter]));
            assert(reversed);
        }
    }
}

/* ---- S3-H08 ----------------------------------------------------------- */
static void test_malformed_reads_rejected(void)
{
    uint32_t w[29];
    route_command_t cmd;
    char ch;

    build_symbol(CODE39[1], 20000U, 60000U, w);
    w[11] = 60000U;                                      /* 4 wide in "B" */
    assert(!scan(w, &cmd, &ch, NULL));

    build_symbol(CODE39[1], 20000U, 60000U, w);
    w[9] = 60000U;                                       /* wide gap */
    assert(!scan(w, &cmd, &ch, NULL));
}

/* ---- S3-H09 ----------------------------------------------------------- */
static void test_barcode_then_junction_executes_route(void)
{
    uint32_t w[29];
    uint8_t i;

    for (i = 0U; i < 4U; i++)
    {
        vehicle_controller_t vc;
        line_follow_t lf;
        junction_observation_t obs;
        route_instruction_t route;
        controller_action_t action;
        route_command_t cmd = ROUTE_COMMAND_NONE;
        char ch = '\0';
        uint32_t t;

        build_symbol(CODE39[i], 20000U, 60000U, w);
        assert(scan(w, &cmd, &ch, NULL));

        vehicle_controller_init(&vc);
        (void)vehicle_controller_set_mode(&vc, SYSTEM_MODE_AUTO);
        route = make_route(cmd, ch, 1000U);
        assert(vehicle_controller_receive_route(&vc, &route, 1000U));

        line_follow_reset(&lf);
        t = hold(&lf, true, true, 2000U, 100U);          /* onto the "+" */
        assert(line_follow_take_junction(&lf, &obs, t));
        action = vehicle_controller_on_junction(&vc, &obs, t);
        assert(action.type == CONTROLLER_ACTION_EXECUTE_ROUTE);
        assert(action.route_command == COMMANDS[i]);
    }
}

/* ---- S3-H10 ----------------------------------------------------------- */
static void run_no_barcode_case(bool line_ahead, route_command_t expected)
{
    vehicle_controller_t vc;
    line_follow_t lf;
    junction_observation_t obs;
    route_instruction_t route;
    controller_action_t action;
    junction_type_t type;

    vehicle_controller_init(&vc);
    (void)vehicle_controller_set_mode(&vc, SYSTEM_MODE_AUTO);
    line_follow_reset(&lf);

    junction_observation_for_type(&obs, JUNCTION_TYPE_UNKNOWN, 100U);
    action = vehicle_controller_on_junction(&vc, &obs, 100U);
    assert(action.type == CONTROLLER_ACTION_NONE);       /* only approach */
    assert(vc.mission == MISSION_JUNCTION_APPROACH);

    line_follow_probe_start(&lf);
    line_follow_probe_sample(&lf, false, false);
    line_follow_probe_sample(&lf, false, line_ahead);
    type = line_follow_probe_finish(&lf);
    assert(type == (line_ahead ? JUNCTION_TYPE_CROSS : JUNCTION_TYPE_T));

    route = make_route(junction_default_route(type), '-', 200U);
    assert(vehicle_controller_receive_route(&vc, &route, 200U));
    junction_observation_for_type(&obs, type, 200U);
    action = vehicle_controller_on_junction(&vc, &obs, 200U);
    assert(action.type == CONTROLLER_ACTION_EXECUTE_ROUTE);
    assert(action.route_command == expected);
}

static void test_no_barcode_defaults_and_override(void)
{
    vehicle_controller_t vc;
    junction_observation_t obs;
    route_instruction_t route;
    controller_action_t action;
    line_follow_t lf;

    line_follow_reset(&lf);
    assert(line_follow_probe_finish(&lf) == JUNCTION_TYPE_UNKNOWN);
    assert(junction_default_route(JUNCTION_TYPE_UNKNOWN) == ROUTE_COMMAND_NONE);

    run_no_barcode_case(true, ROUTE_COMMAND_STRAIGHT);   /* "+" */
    run_no_barcode_case(false, ROUTE_COMMAND_RIGHT);     /* T   */

    /* Barcode "A" at a T still turns LEFT. */
    vehicle_controller_init(&vc);
    (void)vehicle_controller_set_mode(&vc, SYSTEM_MODE_AUTO);
    route = make_route(ROUTE_COMMAND_LEFT, 'A', 0U);
    assert(vehicle_controller_receive_route(&vc, &route, 0U));
    junction_observation_for_type(&obs, JUNCTION_TYPE_T, 50U);
    action = vehicle_controller_on_junction(&vc, &obs, 50U);
    assert(action.route_command == ROUTE_COMMAND_LEFT);
}

/* ---- S3-H11 ----------------------------------------------------------- */
static void test_reacquire(void)
{
    line_follow_t lf;
    junction_observation_t obs;
    uint32_t t;

    line_follow_reset(&lf);
    t = hold(&lf, false, true, 0U, 10U);                /* line last on right */
    t = hold(&lf, false, false, t, 10U);
    line_follow_reacquire_start(&lf, LINE_SIDE_NONE, t);
    assert(line_follow_reacquiring(&lf));
    t = hold(&lf, false, false, t, 100U);               /* still searching */
    assert(line_follow_reacquiring(&lf));
    assert(line_follow_steer(&lf, t) == LINE_REACQUIRE_STEER);

    t = hold(&lf, true, false, t, 2U);                  /* found */
    assert(!line_follow_reacquiring(&lf));
    assert(lf.status.state == LINE_STATE_STEER_LEFT);

    /* A hint overrides the last side. */
    line_follow_reacquire_start(&lf, LINE_SIDE_RIGHT, t);
    assert(line_follow_steer(&lf, t) == LINE_REACQUIRE_STEER);

    /* Hitting the line square-on (both black) is NOT a junction. */
    t = hold(&lf, true, true, t, 100U);
    assert(!line_follow_reacquiring(&lf));
    assert(!line_follow_take_junction(&lf, &obs, t));
}

/* ---- S3-H12 ----------------------------------------------------------- */
static void test_edge_buffer(void)
{
    barcode_edges_t e;
    bool black;
    uint32_t time_us;
    uint32_t i;

    barcode_edges_reset(&e);
    assert(!barcode_edges_pop(&e, &black, &time_us));    /* empty */

    assert(barcode_edges_push(&e, true, 1001U));
    assert(barcode_edges_push(&e, false, 25000U));
    assert(barcode_edges_pop(&e, &black, &time_us) && black && (time_us == 1000U));
    assert(barcode_edges_pop(&e, &black, &time_us) && !black && (time_us == 25000U));

    for (i = 0U; i < (BARCODE_EDGES_SIZE - 1U); i++)
    {
        assert(barcode_edges_push(&e, (i & 1U) != 0U, i * 100U));
    }
    assert(!barcode_edges_push(&e, true, 9999U));        /* full */
    assert(e.overflow == 1U);
    for (i = 0U; i < (BARCODE_EDGES_SIZE - 1U); i++)
    {
        assert(barcode_edges_pop(&e, &black, &time_us));
        assert(time_us == ((i * 100U) & ~1U));           /* FIFO order kept */
    }
}

/* ---- S3-H13 ----------------------------------------------------------- */
/* Every A-Z and 0-9 decodes, forward and backward, and takes its command
 * from the run-time map. Independent table (wide = 1, first element = bit 8). */
static void test_decode_full_alphabet_with_map(void)
{
    static const char CHARS[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    static const uint16_t BITS[36] = {
        0x109U, 0x049U, 0x148U, 0x019U, 0x118U, 0x058U, 0x00DU, 0x10CU,
        0x04CU, 0x01CU, 0x103U, 0x043U, 0x142U, 0x013U, 0x112U, 0x052U,
        0x007U, 0x106U, 0x046U, 0x016U, 0x181U, 0x0C1U, 0x1C0U, 0x091U,
        0x190U, 0x0D0U, 0x034U, 0x121U, 0x061U, 0x160U, 0x031U, 0x130U,
        0x070U, 0x025U, 0x124U, 0x064U};
    static const route_command_t CYCLE[4] = {
        ROUTE_COMMAND_LEFT, ROUTE_COMMAND_RIGHT,
        ROUTE_COMMAND_STRAIGHT, ROUTE_COMMAND_UTURN};
    uint32_t w[29], r[29];
    route_command_t cmd;
    char ch;
    bool reversed;
    uint8_t i, k;

    barcode_map_clear();
    for (i = 0U; i < 36U; i++)
    {
        assert(barcode_map_set(CHARS[i], CYCLE[i % 4U]));
    }
    for (i = 0U; i < 36U; i++)
    {
        build_symbol(BITS[i], 20000U, 60000U, w);
        assert(scan(w, &cmd, &ch, &reversed));
        assert((ch == CHARS[i]) && (cmd == CYCLE[i % 4U]) && !reversed);
        for (k = 0U; k < 29U; k++)
        {
            r[k] = w[28U - k];
        }
        /* Backwards must give the SAME letter (K backwards is U's pattern,
         * so this proves the start/stop check picks the right direction). */
        assert(scan(r, &cmd, &ch, &reversed));
        assert((ch == CHARS[i]) && (cmd == CYCLE[i % 4U]) && reversed);
    }

    /* A good read of an unmapped letter: read succeeds, command NONE. */
    barcode_map_clear();
    build_symbol(BITS[10], 20000U, 60000U, w);           /* K */
    assert(scan(w, &cmd, &ch, NULL));
    assert((ch == 'K') && (cmd == ROUTE_COMMAND_NONE));
    barcode_map_defaults();
}

/* ---- S3-H14 ----------------------------------------------------------- */
static bool map_cmd(const char *text, char *reply)
{
    return barcode_map_command(text, (unsigned int)strlen(text), reply,
                               BARCODE_MAP_REPLY_MAX);
}

static void test_map_command_over_mqtt(void)
{
    char reply[BARCODE_MAP_REPLY_MAX];

    assert(barcode_map_is_command("MAP SHOW", 8U));
    assert(barcode_map_is_command("  map k l", 9U));
    assert(!barcode_map_is_command("START", 5U));
    assert(!barcode_map_is_command("MAPX", 4U));

    assert(map_cmd("MAP SHOW", reply));
    assert(strcmp(reply, "OK A=L B=R C=S D=U") == 0);

    /* Demo-day style: forget defaults, set the announced letters. */
    assert(map_cmd("MAP CLEAR K LEFT Q RIGHT Z STRAIGHT M UTURN", reply));
    assert(strcmp(reply, "OK K=L M=U Q=R Z=S") == 0);
    assert(barcode_map_get('A') == ROUTE_COMMAND_NONE);
    assert(barcode_map_get('k') == ROUTE_COMMAND_LEFT);

    /* Short names, '=' and ',' separators, lower case. */
    assert(map_cmd("map a=r, 7=u", reply));
    assert(barcode_map_get('A') == ROUTE_COMMAND_RIGHT);
    assert(barcode_map_get('7') == ROUTE_COMMAND_UTURN);

    /* Errors change nothing (all-or-nothing). */
    assert(!map_cmd("MAP B LEFT C SIDEWAYS", reply));
    assert(strcmp(reply, "ERR bad command: SIDEWAYS") == 0);
    assert(barcode_map_get('B') == ROUTE_COMMAND_NONE);
    assert(!map_cmd("MAP KK LEFT", reply));
    assert(!map_cmd("MAP * LEFT", reply));
    assert(!map_cmd("MAP K", reply));
    assert(!map_cmd("MAP", reply));

    assert(map_cmd("MAP K NONE", reply));
    assert(barcode_map_get('K') == ROUTE_COMMAND_NONE);
    assert(map_cmd("MAP CLEAR", reply));
    assert(strcmp(reply, "OK (empty)") == 0);
    assert(map_cmd("MAP DEFAULT", reply));
    assert(strcmp(reply, "OK A=L B=R C=S D=U") == 0);
}

int main(void)
{
    test_status_is_one_byte();
    test_majority_filter();
    test_steering_ramp_and_damping();
    test_junction_debounce_and_single_report();
    test_barcode_calibration_and_hysteresis();
    test_barcode_adaptation();
    test_decode_all_letters_both_directions_all_speeds();
    test_malformed_reads_rejected();
    test_barcode_then_junction_executes_route();
    test_no_barcode_defaults_and_override();
    test_reacquire();
    test_edge_buffer();
    test_decode_full_alphabet_with_map();
    test_map_command_over_mqtt();
    (void)puts("subsystem3 sensor host tests passed (S3-H01..S3-H14)");
    return 0;
}
