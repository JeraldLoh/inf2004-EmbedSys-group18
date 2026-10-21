#include "lsm303_decode.h"

#include <stddef.h>

static int16_t decode_axis(uint8_t low, uint8_t high)
{
    int16_t packed = (int16_t)(((uint16_t)high << 8) | low);
    return (int16_t)(packed / 16);
}

bool lsm303_decode_acceleration(
    const uint8_t raw[6],
    lsm303_acceleration_t *sample)
{
    if ((raw == NULL) || (sample == NULL))
    {
        return false;
    }

    sample->x_mg = decode_axis(raw[0], raw[1]);
    sample->y_mg = decode_axis(raw[2], raw[3]);
    sample->z_mg = decode_axis(raw[4], raw[5]);
    return true;
}
