#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../app_program/subsystem3/barcode_decoder.h"
#include "../../app_program/subsystem4/imu_math.h"

static void feed_code39_a(barcode_decoder_t *decoder)
{
    /* Narrow=1 ms, wide=2 ms. Patterns are * (0x094), A (0x109), *. */
    static const uint16_t patterns[3] = {0x094U, 0x109U, 0x094U};
    uint32_t now_us = 1U;
    bool black = false;
    uint8_t symbol;
    uint8_t bit;

    barcode_decoder_feed(decoder, black, now_us);
    now_us += 9000U;
    barcode_decoder_feed(decoder, black, now_us);
    black = true;
    barcode_decoder_feed(decoder, black, now_us);

    for (symbol = 0U; symbol < 3U; symbol++)
    {
        for (bit = 0U; bit < 9U; bit++)
        {
            uint16_t mask = (uint16_t)(1U << (8U - bit));
            now_us += ((patterns[symbol] & mask) != 0U) ? 2000U : 1000U;
            black = !black;
            barcode_decoder_feed(decoder, black, now_us);
        }
        if (symbol < 2U)
        {
            now_us += 1000U;
            black = !black;
            barcode_decoder_feed(decoder, black, now_us);
        }
    }
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
    test_lsm303_signed_decode();
    (void)puts("perception math host tests passed");
    return 0;
}
