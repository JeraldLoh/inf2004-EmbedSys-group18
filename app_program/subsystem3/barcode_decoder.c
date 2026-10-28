#include "barcode_decoder.h"

#include <stddef.h>

#include "barcode_map.h"

#define BARCODE_QUIET_START_US 8000U
#define BARCODE_QUIET_END_US 28000U   /* lower bound for the end quiet zone */
#define BARCODE_QUIET_END_FACTOR 5U   /* end quiet zone = 5 x narrow bar */
#define BARCODE_MIN_RUN_US 400U
#define BARCODE_MAX_SCAN_US 6000000U  /* 141 mm barcode at ~25 mm/s */
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

static uint32_t minimum_run_us(const barcode_decoder_t *decoder)
{
    uint32_t minimum = 0U;
    uint8_t index;

    for (index = 0U; index < decoder->run_count; index++)
    {
        uint32_t width = decoder->run_us[index];
        if ((width >= BARCODE_MIN_RUN_US) &&
            ((minimum == 0U) || (width < minimum)))
        {
            minimum = width;
        }
    }
    return minimum;
}

/* Builds the 9-bit wide/narrow pattern of the character that starts at
 * element "start". When "reverse" is true the runs are read from the end
 * backwards, which is what a barcode looks like when the car crosses it in
 * the opposite direction (for example after a U-turn). */
static uint16_t pattern_at(
    const barcode_decoder_t *decoder,
    uint8_t start,
    uint32_t narrow_us,
    bool reverse)
{
    uint16_t bits = 0U;
    uint32_t wide_limit =
        (narrow_us * BARCODE_WIDE_NUMERATOR) / BARCODE_WIDE_DENOMINATOR;
    uint8_t index;
    uint8_t run;

    for (index = 0U; index < 9U; index++)
    {
        run = (uint8_t)(start + index);
        if (reverse)
        {
            run = (uint8_t)(BARCODE_SYMBOL_RUNS - 1U - run);
        }
        bits = (uint16_t)(bits << 1);
        if (decoder->run_us[run] > wide_limit)
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
static bool gaps_are_narrow(const barcode_decoder_t *decoder, uint32_t narrow_us)
{
    uint32_t wide_limit =
        (narrow_us * BARCODE_WIDE_NUMERATOR) / BARCODE_WIDE_DENOMINATOR;

    return (decoder->run_us[9] <= wide_limit) &&
           (decoder->run_us[19] <= wide_limit);
}

static bool decode_direction(
    barcode_decoder_t *decoder,
    uint32_t narrow_us,
    bool reverse)
{
    uint16_t start_bits = pattern_at(decoder, 0U, narrow_us, reverse);
    uint16_t data_bits = pattern_at(decoder, 10U, narrow_us, reverse);
    uint16_t stop_bits = pattern_at(decoder, 20U, narrow_us, reverse);
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
    uint32_t narrow_us;

    /* Exactly one Code 39 character is authoritative: * X * plus two gaps. */
    if (decoder->run_count < BARCODE_SYMBOL_RUNS)
    {
        return false;
    }

    narrow_us = minimum_run_us(decoder);
    if ((narrow_us == 0U) || !gaps_are_narrow(decoder, narrow_us))
    {
        return false;
    }

    /* Try the normal direction first, then the mirrored one. */
    return decode_direction(decoder, narrow_us, false) ||
           decode_direction(decoder, narrow_us, true);
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
    if (decode_complete_symbol(decoder))
    {
        decoder->state = BARCODE_DECODER_COMPLETE;
    }
    else
    {
        barcode_decoder_reset(decoder);
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
    else if (!centre_sensor_black && (decoder->run_count >= 19U))
    {
        /* The end quiet zone is judged relative to the narrow bar width, so
         * the decoder works at any car speed. A wide space is at most ~3x
         * narrow, so 5x narrow can only be the white area after the barcode. */
        uint32_t quiet_us = minimum_run_us(decoder) * BARCODE_QUIET_END_FACTOR;
        if (quiet_us < BARCODE_QUIET_END_US)
        {
            quiet_us = BARCODE_QUIET_END_US;
        }
        if ((now_us - decoder->run_started_us) >= quiet_us)
        {
            finish_or_reset(decoder);
            return;
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
