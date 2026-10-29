/* Portable decoder checks required by the team host CMake target.
 * These do not exercise I2C, calibration or physical sensor accuracy. */
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include "../../app_program/subsystem4/lsm303_decode.h"

int main(void)
{
    const uint8_t signed_axes[6] = {0x80, 0x3e, 0x80, 0xc1, 0x00, 0x00};
    const uint8_t extremes[6] = {0xf0, 0x7f, 0x00, 0x80, 0xf0, 0xff};
    lsm303_acceleration_t sample = {0};
    assert(lsm303_decode_acceleration(signed_axes, &sample));
    assert(sample.x_mg == 1000 && sample.y_mg == -1000 && sample.z_mg == 0);
    assert(lsm303_decode_acceleration(extremes, &sample));
    assert(sample.x_mg == 2047 && sample.y_mg == -2048 && sample.z_mg == -1);
    assert(!lsm303_decode_acceleration(NULL, &sample));
    assert(!lsm303_decode_acceleration(signed_axes, NULL));
    return 0;
}
