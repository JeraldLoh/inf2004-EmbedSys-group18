#include "barcode_decoder.h"

#include <stddef.h>

#include "barcode_map.h"

#define BARCODE_QUIET_START_US 8000U
#define BARCODE_QUIET_END_US 28000U   /* lower bound for the end quiet zone */
#define BARCODE_MIN_RUN_US 400U
#define BARCODE_MAX_SCAN_US 10000000U /* largest printed copy ~270 mm: 5.4 s at 50 mm/s */
#define BARCODE_ABANDON_MIN_US 600000U /* shortest white that ends a partial read */
#define BARCODE_MAX_BAR_US 1500000U    /* longer black is tape, not a bar */
#define BARCODE_WIDE_NUMERATOR 17U
#define BARCODE_WIDE_DENOMINATOR 10U
#define BARCODE_SYMBOL_RUNS 29U   /* 3 characters x 9 + 2 gaps */

/* Full Code 39 table. Each character is 9 elements read in order
 * bar space bar space bar space bar space bar; bit 8 is the first element
 * and a 1 means that element is wide. Every entry has exactly 3 wide.
 * Checked against the python-barcode library's Code 39 table. */
typedef struct
{
    uint16_t bits;
    char character;
} code39_entry_t;

static const code39_entry_t CODE39_TABLE[] = {
    {0x109U, 'A'}, {0x049U, 'B'}, {0x148U, 'C'}, {0x019U, 'D'},
    {0x118U, 'E'}, {0x058U, 'F'}, {0x00DU, 'G'}, {0x10CU, 'H'},
    {0x04CU, 'I'}, {0x01CU, 'J'}, {0x103U, 'K'}, {0x043U, 'L'},
    {0x142U, 'M'}, {0x013U, 'N'}, {0x112U, 'O'}, {0x052U, 'P'},
    {0x007U, 'Q'}, {0x106U, 'R'}, {0x046U, 'S'}, {0x016U, 'T'},
    {0x181U, 'U'}, {0x0C1U, 'V'}, {0x1C0U, 'W'}, {0x091U, 'X'},
    {0x190U, 'Y'}, {0x0D0U, 'Z'},
    {0x034U, '0'}, {0x121U, '1'}, {0x061U, '2'}, {0x160U, '3'},
    {0x031U, '4'}, {0x130U, '5'}, {0x070U, '6'}, {0x025U, '7'},
    {0x124U, '8'}, {0x064U, '9'},
    {0x085U, '-'}, {0x184U, '.'}, {0x0C4U, ' '},
    {0x094U, '*'}   /* start/stop */
};

static char code39_character(uint16_t bits)
{
    uint8_t index;

    for (index = 0U;
         index < (uint8_t)(sizeof(CODE39_TABLE) / sizeof(CODE39_TABLE[0]));
         index++)
    {
        if (CODE39_TABLE[index].bits == bits)
        {
            return CODE39_TABLE[index].character;
        }
    }
    return '?';
}

/* Wide/narrow limits, worked out separately for bars (even runs) and spaces
 * (odd runs). Sensor blur makes every black bar read a little thinner and
 * every white space a little wider, so one shared "narrow" width misjudges
 * one of the two colours. Each colour gets the midpoint of its own shortest
 * and longest run; a valid symbol always has both widths of both colours
 * because the '*' start/stop character does. Returns false if impossible. */
static bool wide_limits(const barcode_decoder_t *decoder,
                        uint32_t *bar_limit, uint32_t *space_limit)
{
    uint32_t low[2] = {0U, 0U};
    uint32_t high[2] = {0U, 0U};
    uint8_t index;

    for (index = 0U; index < BARCODE_SYMBOL_RUNS; index++)
    {
        uint8_t colour = (uint8_t)(index & 1U);   /* 0 = bar, 1 = space */
        uint32_t width = decoder->run_us[index];

        if ((low[colour] == 0U) || (width < low[colour]))
        {
            low[colour] = width;
        }
        if (width > high[colour])
        {
            high[colour] = width;
        }
    }
    /* A real wide element is at least ~2x narrow; anything flatter is noise. */
    if ((low[0] == 0U) || (low[1] == 0U) ||
        ((high[0] * BARCODE_WIDE_DENOMINATOR) < (low[0] * BARCODE_WIDE_NUMERATOR)) ||
        ((high[1] * BARCODE_WIDE_DENOMINATOR) < (low[1] * BARCODE_WIDE_NUMERATOR)))
    {
        return false;
    }
    *bar_limit = (low[0] + high[0]) / 2U;
    *space_limit = (low[1] + high[1]) / 2U;
    return true;
}

/* Builds the 9-bit wide/narrow pattern of the character that starts at
 * element "start". When "reverse" is true the runs are read from the end
 * backwards, which is what a barcode looks like when the car crosses it in
 * the opposite direction (for example after a U-turn). */
static uint16_t pattern_at(
    const barcode_decoder_t *decoder,
    uint8_t start,
    uint32_t bar_limit,
    uint32_t space_limit,
    bool reverse)
{
    uint16_t bits = 0U;
    uint8_t index;
    uint8_t run;

    for (index = 0U; index < 9U; index++)
    {
        run = (uint8_t)(start + index);
        if (reverse)
        {
            /* 28 - run keeps the parity, so bars stay bars. */
            run = (uint8_t)(BARCODE_SYMBOL_RUNS - 1U - run);
        }
        bits = (uint16_t)(bits << 1);
        if (decoder->run_us[run] > (((run & 1U) == 0U) ? bar_limit : space_limit))
        {
            bits = (uint16_t)(bits | 1U);
        }
    }
    return bits;
}

/* Every Code 39 character has exactly 3 wide elements out of 9. */
static bool has_three_wide(uint16_t bits)
{
    uint8_t count = 0U;

    while (bits != 0U)
    {
        count = (uint8_t)(count + (bits & 1U));
        bits = (uint16_t)(bits >> 1);
    }
    return count == 3U;
}

/* The gap between characters is always a narrow space. Gaps sit at elements
 * 9 and 19, which are the same positions whichever way the code is read. */
static bool gaps_are_narrow(const barcode_decoder_t *decoder, uint32_t space_limit)
{
    return (decoder->run_us[9] <= space_limit) &&
           (decoder->run_us[19] <= space_limit);
}

static bool decode_direction(
    barcode_decoder_t *decoder,
    uint32_t bar_limit,
    uint32_t space_limit,
    bool reverse)
{
    uint16_t start_bits = pattern_at(decoder, 0U, bar_limit, space_limit, reverse);
    uint16_t data_bits = pattern_at(decoder, 10U, bar_limit, space_limit, reverse);
    uint16_t stop_bits = pattern_at(decoder, 20U, bar_limit, space_limit, reverse);
    char data;

    /* Validation: reject anything that is not a well-formed symbol, so a
     * smudge or speed change gives "no read" instead of the wrong command. */
    if (!has_three_wide(start_bits) || !has_three_wide(data_bits) ||
        !has_three_wide(stop_bits))
    {
        return false;
    }
    if ((code39_character(start_bits) != '*') ||
        (code39_character(stop_bits) != '*'))
    {
        return false;
    }

    /* The '*' check above also fixes the direction: '*' read backwards is
     * 'P', so a reversed crossing always fails the forward try. That matters
     * because some letters are mirrors of others (K backwards is U). */
    data = code39_character(data_bits);
    if ((data == '?') || (data == '*'))
    {
        return false;
    }
    /* Any valid character is a good read. The command comes from the
     * run-time map (set over MQTT); NONE means "read, but not mapped". */
    decoder->command = barcode_map_get(data);
    decoder->character = data;
    decoder->reversed = reverse;
    return true;
}

static bool decode_complete_symbol(barcode_decoder_t *decoder)
{
    uint32_t bar_limit;
    uint32_t space_limit;

    /* Exactly one Code 39 character is authoritative: * X * plus two gaps. */
    if (decoder->run_count < BARCODE_SYMBOL_RUNS)
    {
        return false;
    }

    if (!wide_limits(decoder, &bar_limit, &space_limit) ||
        !gaps_are_narrow(decoder, space_limit))
    {
        return false;
    }

    /* Try the normal direction first, then the mirrored one. */
    return decode_direction(decoder, bar_limit, space_limit, false) ||
           decode_direction(decoder, bar_limit, space_limit, true);
}

static void push_run(barcode_decoder_t *decoder, uint32_t width_us)
{
    if ((width_us >= BARCODE_MIN_RUN_US) &&
        (decoder->run_count < BARCODE_DECODER_MAX_RUNS))
    {
        decoder->run_us[decoder->run_count++] = width_us;
    }
}

static void finish_or_reset(barcode_decoder_t *decoder)
{
    {
        uint8_t k;

        for (k = 0U; k < decoder->run_count; k++)
        {
            decoder->trace_run_us[k] = decoder->run_us[k];
        }
        decoder->trace_count = decoder->run_count;
        decoder->trace_id++;
        decoder->trace_ok = false;
        decoder->trace_pending = true;
    }
    if (decode_complete_symbol(decoder))
    {
        decoder->state = BARCODE_DECODER_COMPLETE;
        decoder->trace_ok = true;
    }
    else
    {
        uint8_t index;

        for (index = 0U; index < decoder->run_count; index++)
        {
            decoder->fail_run_us[index] = decoder->run_us[index];
        }
        decoder->fail_count = decoder->run_count;
        decoder->fail_pending = true;
        barcode_decoder_reset(decoder);   /* leaves the fail_* fields alone */
    }
}

void barcode_decoder_reset(barcode_decoder_t *decoder)
{
    uint8_t index;

    if (decoder == NULL)
    {
        return;
    }

    decoder->state = BARCODE_DECODER_IDLE;
    decoder->run_started_us = 0U;
    decoder->capture_started_us = 0U;
    decoder->white_started_us = 0U;
    decoder->run_count = 0U;
    decoder->last_black = false;
    decoder->saw_quiet_zone = false;
    decoder->command = ROUTE_COMMAND_NONE;
    decoder->character = '\0';
    decoder->reversed = false;
    for (index = 0U; index < BARCODE_DECODER_MAX_RUNS; index++)
    {
        decoder->run_us[index] = 0U;
    }
}

void barcode_decoder_feed(
    barcode_decoder_t *decoder,
    bool centre_sensor_black,
    uint32_t now_us)
{
    uint32_t width_us;

    if ((decoder == NULL) ||
        (decoder->state == BARCODE_DECODER_COMPLETE))
    {
        return;
    }

    if (decoder->state == BARCODE_DECODER_IDLE)
    {
        if (!centre_sensor_black)
        {
            if (decoder->white_started_us == 0U)
            {
                decoder->white_started_us = now_us;
            }
            if ((now_us - decoder->white_started_us) >=
                BARCODE_QUIET_START_US)
            {
                decoder->saw_quiet_zone = true;
            }
        }
        else
        {
            if (decoder->saw_quiet_zone)
            {
                decoder->state = BARCODE_DECODER_CAPTURE;
                decoder->run_started_us = now_us;
                decoder->capture_started_us = now_us;
                decoder->last_black = true;
                decoder->saw_quiet_zone = false;
            }
            decoder->white_started_us = 0U;
        }
        return;
    }

    if (centre_sensor_black != decoder->last_black)
    {
        width_us = now_us - decoder->run_started_us;
        push_run(decoder, width_us);
        decoder->run_started_us = now_us;
        decoder->last_black = centre_sensor_black;
        if (decoder->run_count >= BARCODE_SYMBOL_RUNS)
        {
            finish_or_reset(decoder);
            return;
        }
    }
    else
    {
        /* How long the current colour has lasted, against the longest
         * element captured so far. Measured on the car (2026-11-10): sensor
         * blur makes narrow spaces read ~45 ms but wide spaces ~245 ms, over
         * 5x, so "5x the narrowest" cut real barcodes off at a wide space.
         * Every wide space is shorter than 2x the longest element (the wide
         * bars read longest), so 2x longest only ever matches the white
         * after the barcode or a sensor that has left it. */
        uint32_t current_us = now_us - decoder->run_started_us;
        uint32_t longest = 0U;
        uint32_t limit;
        uint8_t index;

        for (index = 0U; index < decoder->run_count; index++)
        {
            if (decoder->run_us[index] > longest)
            {
                longest = decoder->run_us[index];
            }
        }

        if (centre_sensor_black)
        {
            /* Far longer than any bar: line tape or a crossbar. Drop it at
             * once so the decoder is idle again before the next barcode. */
            if (current_us > BARCODE_MAX_BAR_US)
            {
                barcode_decoder_reset(decoder);
                return;
            }
        }
        else if (decoder->run_count > 0U)
        {
            /* At 19+ elements a long white is the end quiet zone; below
             * that it is a partial read. Either way, decide now instead of
             * waiting out BARCODE_MAX_SCAN_US. A floor protects the first
             * elements, when only a thin bar may have been seen. */
            limit = longest * 2U;
            if (decoder->run_count >= 19U)
            {
                if (limit < BARCODE_QUIET_END_US)
                {
                    limit = BARCODE_QUIET_END_US;
                }
            }
            else if (limit < BARCODE_ABANDON_MIN_US)
            {
                limit = BARCODE_ABANDON_MIN_US;
            }
            if (current_us > limit)
            {
                if (decoder->run_count < 19U)
                {
                    push_run(decoder, current_us);   /* shows in diagnostics */
                }
                finish_or_reset(decoder);
                return;
            }
        }
    }

    if ((now_us - decoder->capture_started_us) > BARCODE_MAX_SCAN_US)
    {
        finish_or_reset(decoder);
    }
}

bool barcode_decoder_take(
    barcode_decoder_t *decoder,
    route_command_t *command,
    char *character)
{
    if ((decoder == NULL) || (command == NULL) || (character == NULL) ||
        (decoder->state != BARCODE_DECODER_COMPLETE))
    {
        return false;
    }

    *command = decoder->command;
    *character = decoder->character;
    barcode_decoder_reset(decoder);
    return true;
}

bool barcode_decoder_busy(const barcode_decoder_t *decoder)
{
    return (decoder != NULL) && (decoder->state == BARCODE_DECODER_CAPTURE);
}

bool barcode_decoder_was_reversed(const barcode_decoder_t *decoder)
{
    return (decoder != NULL) && decoder->reversed;
}
