#ifndef GROUP18_LSM303_DECODE_H
#define GROUP18_LSM303_DECODE_H

#include <stdbool.h>
#include <stdint.h>

typedef struct
{
    int16_t x_mg;
    int16_t y_mg;
    int16_t z_mg;
} lsm303_acceleration_t;

/* Decode the LSM303DLHC's six little-endian, left-justified 12-bit samples. */
bool lsm303_decode_acceleration(
    const uint8_t raw[6],
    lsm303_acceleration_t *sample);

#endif
