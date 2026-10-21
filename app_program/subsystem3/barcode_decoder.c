#include "barcode_decoder.h"

#include <stddef.h>

#define BARCODE_QUIET_START_US 8000U
#define BARCODE_QUIET_END_US 28000U
#define BARCODE_MIN_RUN_US 400U
#define BARCODE_MAX_SCAN_US 1500000U
#define BARCODE_WIDE_NUMERATOR 17U
#define BARCODE_WIDE_DENOMINATOR 10U

static route_command_t command_from_character(char character)
{
    switch (character)
    {
        case 'A': return ROUTE_COMMAND_LEFT;
        case 'B': return ROUTE_COMMAND_RIGHT;
        case 'C': return ROUTE_COMMAND_STRAIGHT;
        case 'D': return ROUTE_COMMAND_UTURN;
        default: return ROUTE_COMMAND_NONE;
    }
}

static char code39_character(uint16_t bits)
{
    switch (bits)
    {
        case 0x109U: return 'A';
        case 0x049U: return 'B';
        case 0x148U: return 'C';
        case 0x019U: return 'D';
        case 0x094U: return '*';
        default: return '?';
    }
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

static uint16_t pattern_at(
    const barcode_decoder_t *decoder,
    uint8_t start,
    uint32_t narrow_us)
{
    uint16_t bits = 0U;
    uint32_t wide_limit =
        (narrow_us * BARCODE_WIDE_NUMERATOR) / BARCODE_WIDE_DENOMINATOR;
    uint8_t index;

    for (index = 0U; index < 9U; index++)
    {
        bits = (uint16_t)(bits << 1);
        if (decoder->run_us[start + index] > wide_limit)
        {
            bits = (uint16_t)(bits | 1U);
        }
    }
    return bits;
}

static bool decode_complete_symbol(barcode_decoder_t *decoder)
{
    uint32_t narrow_us;
    char start;
    char data;
    char stop;

    /* Exactly one Code 39 character is authoritative: * X * plus two gaps. */
    if (decoder->run_count < 29U)
    {
        return false;
    }

    narrow_us = minimum_run_us(decoder);
    if (narrow_us == 0U)
    {
        return false;
    }

    start = code39_character(pattern_at(decoder, 0U, narrow_us));
    data = code39_character(pattern_at(decoder, 10U, narrow_us));
    stop = code39_character(pattern_at(decoder, 20U, narrow_us));

    if ((start != '*') || (stop != '*'))
    {
        return false;
    }

    decoder->command = command_from_character(data);
    if (decoder->command == ROUTE_COMMAND_NONE)
    {
        return false;
    }

    decoder->character = data;
    return true;
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
        if (decoder->run_count >= 29U)
        {
            finish_or_reset(decoder);
            return;
        }
    }
    else if (!centre_sensor_black && (decoder->run_count >= 19U) &&
        ((now_us - decoder->run_started_us) >= BARCODE_QUIET_END_US))
    {
        finish_or_reset(decoder);
        return;
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
