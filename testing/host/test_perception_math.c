#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem3/barcode_decoder.h"
#include "../../app_program/subsystem4/imu_math.h"

/* Feeds *A* as 29 timed runs. blur_us shortens every bar and lengthens every
 * space by that much (what a sensor spot wider than a thin bar does), and
 * reverse feeds the runs back to front (car crossing the other way). */
static uint32_t feed_code39_a_at(barcode_decoder_t *decoder, uint32_t start_us,
                                 uint32_t narrow_us, uint32_t wide_us,
                                 uint32_t blur_us, bool reverse)
{
    static const uint16_t patterns[3] = {0x094U, 0x109U, 0x094U};
    uint32_t runs[29];
    uint32_t now_us = start_us;
    bool black = false;
    uint8_t count = 0U;
    uint8_t symbol;
    uint8_t bit;
    uint8_t index;

    for (symbol = 0U; symbol < 3U; symbol++)
    {
        for (bit = 0U; bit < 9U; bit++)
        {
            uint16_t mask = (uint16_t)(1U << (8U - bit));
            runs[count++] = ((patterns[symbol] & mask) != 0U) ? wide_us : narrow_us;
        }
        if (symbol < 2U)
        {
            runs[count++] = narrow_us;   /* inter-character gap */
        }
    }

    barcode_decoder_feed(decoder, black, now_us);
    now_us += 9000U;
    barcode_decoder_feed(decoder, black, now_us);
    black = true;
    barcode_decoder_feed(decoder, black, now_us);

    for (index = 0U; index < 29U; index++)
    {
        uint32_t run = runs[reverse ? (28U - index) : index];
        run = ((index & 1U) == 0U) ? (run - blur_us) : (run + blur_us);
        now_us += run;
        black = !black;
        barcode_decoder_feed(decoder, black, now_us);
    }
    /* End quiet zone. */
    now_us += 10U * wide_us;
    barcode_decoder_feed(decoder, false, now_us);
    return now_us;
}

static void feed_code39_a_timed(barcode_decoder_t *decoder, uint32_t narrow_us,
                                uint32_t wide_us, uint32_t blur_us, bool reverse)
{
    (void)feed_code39_a_at(decoder, 1U, narrow_us, wide_us, blur_us, reverse);
}

static void feed_code39_a(barcode_decoder_t *decoder)
{
    feed_code39_a_timed(decoder, 1000U, 2000U, 0U, false);
}

/* A few clipped bars, then a long white stretch, then the real barcode a
 * second later: the partial read must be dropped in time for the real one. */
static void test_barcode_partial_read_then_real(void)
{
    barcode_decoder_t decoder;
    route_command_t command = ROUTE_COMMAND_NONE;
    char character = '\0';
    uint32_t now_us = 1U;
    uint8_t i;

    barcode_decoder_reset(&decoder);
    barcode_decoder_feed(&decoder, false, now_us);
    now_us += 20000U;
    barcode_decoder_feed(&decoder, false, now_us);
    for (i = 0U; i < 5U; i++)   /* black, white, black, white, black */
    {
        barcode_decoder_feed(&decoder, (i & 1U) == 0U, now_us);
        now_us += 60000U;
    }
    barcode_decoder_feed(&decoder, false, now_us);
    for (i = 0U; i < 100U; i++)   /* 1 s of white, sampled every 10 ms */
    {
        now_us += 10000U;
        barcode_decoder_feed(&decoder, false, now_us);
    }
    assert(!barcode_decoder_busy(&decoder));
    (void)feed_code39_a_at(&decoder, now_us, 70000U, 175000U, 30000U, false);
    assert(barcode_decoder_take(&decoder, &command, &character));
    assert(character == 'A');
}

/* Element widths measured on the car 2026-11-10 crossing the 29 cm "A" of
 * the course sample (2 ms waveform). Blur stretches bars and shrinks spaces:
 * narrow spaces ~45-84 ms but wide spaces ~245 ms. The old "5x narrowest =
 * end of barcode" rule stopped at element 22, a wide space. */
static void test_barcode_measured_car_widths(void)
{
    static const uint32_t runs_ms[29] = {
        133, 250, 99, 45, 266, 50, 275, 55, 149,   /* *   */
        66,                                        /* gap */
        260, 66, 99, 69, 93, 243, 99, 72, 260,     /* A   */
        84,                                        /* gap */
        73, 245, 90, 74, 264, 76, 267, 74, 107     /* *   */
    };
    barcode_decoder_t decoder;
    route_command_t command = ROUTE_COMMAND_NONE;
    char character = '\0';
    uint32_t now_us = 1U;
    uint32_t end_us;
    uint8_t index;
    bool black = false;

    barcode_decoder_reset(&decoder);
    barcode_decoder_feed(&decoder, false, now_us);
    now_us += 20000U;
    barcode_decoder_feed(&decoder, false, now_us);
    black = true;
    barcode_decoder_feed(&decoder, black, now_us);
    for (index = 0U; index < 29U; index++)
    {
        /* Sample every 1 ms like the car, so the end rules see each run. */
        end_us = now_us + (runs_ms[index] * 1000U);
        while (now_us < end_us)
        {
            now_us += 1000U;
            barcode_decoder_feed(&decoder, black, now_us);
        }
        black = !black;
        barcode_decoder_feed(&decoder, black, now_us);
    }
    for (index = 0U; index < 100U; index++)
    {
        now_us += 1000U;
        barcode_decoder_feed(&decoder, false, now_us);
    }
    assert(barcode_decoder_take(&decoder, &command, &character));
    assert(character == 'A');
    assert(command == ROUTE_COMMAND_LEFT);
}

/* Line tape under the barcode sensor (a long black) must not leave the
 * decoder busy: a barcode straight after it must still read. */
static void test_barcode_after_long_black(void)
{
    barcode_decoder_t decoder;
    route_command_t command = ROUTE_COMMAND_NONE;
    char character = '\0';
    uint32_t now_us = 1U;
    uint16_t i;

    barcode_decoder_reset(&decoder);
    barcode_decoder_feed(&decoder, false, now_us);
    now_us += 20000U;
    for (i = 0U; i < 2000U; i++)   /* 2 s of black, 1 ms samples */
    {
        now_us += 1000U;
        barcode_decoder_feed(&decoder, true, now_us);
    }
    for (i = 0U; i < 50U; i++)     /* 50 ms white */
    {
        now_us += 1000U;
        barcode_decoder_feed(&decoder, false, now_us);
    }
    assert(!barcode_decoder_busy(&decoder));
    (void)feed_code39_a_at(&decoder, now_us, 70000U, 175000U, 30000U, false);
    assert(barcode_decoder_take(&decoder, &command, &character));
    assert(character == 'A');
}

/* Measured on the car 2026-11-10 at ~70 mm/s: thin bars ~40 ms but thin
 * spaces ~100 ms, crossed backwards. One shared "narrow" width rejected it. */
static void test_barcode_blurred_reverse_read(void)
{
    barcode_decoder_t decoder;
    route_command_t command = ROUTE_COMMAND_NONE;
    char character = '\0';

    barcode_decoder_reset(&decoder);
    feed_code39_a_timed(&decoder, 70000U, 175000U, 30000U, true);
    assert(barcode_decoder_take(&decoder, &command, &character));
    assert(character == 'A');
    assert(command == ROUTE_COMMAND_LEFT);
}

static void test_barcode_a_maps_to_left(void)
{
    barcode_decoder_t decoder;
    route_command_t command = ROUTE_COMMAND_NONE;
    char character = '\0';

    barcode_decoder_reset(&decoder);
    feed_code39_a(&decoder);
    assert(barcode_decoder_take(&decoder, &command, &character));
    assert(command == ROUTE_COMMAND_LEFT);
    assert(character == 'A');
    assert(!barcode_decoder_take(&decoder, &command, &character));
}

static void test_lsm303_signed_decode(void)
{
    /* +1000 mg, -1000 mg, 0 mg after the sensor's 4-bit right shift. */
    const uint8_t raw[6] = {0x80U, 0x3EU, 0x80U, 0xC1U, 0x00U, 0x00U};
    int16_t sample[3];
    bool saturated = true;

    assert(imu_decode_accel_mg(raw, IMU_ACCEL_FS_2G, sample, &saturated));
    assert(sample[0] == 1000);
    assert(sample[1] == -1000);
    assert(sample[2] == 0);
    assert(!saturated);
    assert(!imu_decode_accel_mg(NULL, IMU_ACCEL_FS_2G, sample, &saturated));
}

int main(void)
{
    test_barcode_a_maps_to_left();
    test_barcode_blurred_reverse_read();
    test_barcode_partial_read_then_real();
    test_barcode_measured_car_widths();
    test_barcode_after_long_black();
    test_lsm303_signed_decode();
    (void)puts("perception math host tests passed");
    return 0;
}
