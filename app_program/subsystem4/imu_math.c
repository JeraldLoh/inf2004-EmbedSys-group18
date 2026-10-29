#include "imu_math.h"

#include <stddef.h>

/* round(32767 * sin(d deg)) for d = 0..90. */
static const uint16_t sin_table_q15[91] = {
    0U, 572U, 1144U, 1715U, 2286U, 2856U, 3425U, 3993U, 4560U, 5126U,
    5690U, 6252U, 6813U, 7371U, 7927U, 8481U, 9032U, 9580U, 10126U, 10668U,
    11207U, 11743U, 12275U, 12803U, 13328U, 13848U, 14364U, 14876U, 15383U, 15886U,
    16383U, 16876U, 17364U, 17846U, 18323U, 18794U, 19260U, 19720U, 20173U, 20621U,
    21062U, 21497U, 21925U, 22347U, 22762U, 23170U, 23571U, 23964U, 24351U, 24730U,
    25101U, 25465U, 25821U, 26169U, 26509U, 26841U, 27165U, 27481U, 27788U, 28087U,
    28377U, 28659U, 28932U, 29196U, 29451U, 29697U, 29934U, 30162U, 30381U, 30591U,
    30791U, 30982U, 31163U, 31335U, 31498U, 31650U, 31794U, 31927U, 32051U, 32165U,
    32269U, 32364U, 32448U, 32523U, 32587U, 32642U, 32687U, 32722U, 32747U, 32762U,
    32767U,
};

static int32_t decode_le12(uint8_t low, uint8_t high)
{
    int16_t packed = (int16_t)(((uint16_t)high << 8) | low);
    return (int32_t)packed / 16;
}

static int32_t decode_be16(uint8_t high, uint8_t low)
{
    return (int32_t)(int16_t)(((uint16_t)high << 8) | low);
}

static uint32_t abs_u32(int32_t value)
{
    return (value < 0) ? (uint32_t)(-(int64_t)value) : (uint32_t)value;
}

static uint64_t abs_u64(int64_t value)
{
    return (value < 0) ? (uint64_t)0 - (uint64_t)value : (uint64_t)value;
}

uint8_t imu_accel_mg_per_lsb(uint8_t fs_code)
{
    static const uint8_t mg_per_lsb[4] = {1U, 2U, 4U, 12U};
    return mg_per_lsb[fs_code & 3U];
}

bool imu_decode_accel_mg(
    const uint8_t raw[6],
    uint8_t fs_code,
    int16_t out_mg[3],
    bool *saturated)
{
    int32_t scale;
    bool clipped = false;
    unsigned axis;

    if ((raw == NULL) || (out_mg == NULL) || (saturated == NULL))
    {
        return false;
    }

    scale = (int32_t)imu_accel_mg_per_lsb(fs_code);
    for (axis = 0U; axis < 3U; axis++)
    {
        int32_t counts = decode_le12(raw[2U * axis], raw[(2U * axis) + 1U]);
        if ((counts >= 2047) || (counts <= -2048))
        {
            clipped = true;
        }
        out_mg[axis] = (int16_t)(counts * scale);
    }
    *saturated = clipped;
    return true;
}

bool imu_decode_mag(const uint8_t raw[6], int16_t out[3], bool *overflow)
{
    int32_t x;
    int32_t y;
    int32_t z;

    if ((raw == NULL) || (out == NULL) || (overflow == NULL))
    {
        return false;
    }

    x = decode_be16(raw[0], raw[1]);
    z = decode_be16(raw[2], raw[3]);
    y = decode_be16(raw[4], raw[5]);
    *overflow = (x == -4096) || (y == -4096) || (z == -4096);

    out[0] = (int16_t)x;
    out[1] = (int16_t)y;
    out[2] = (int16_t)((z * 1100) / 980);
    return true;
}

bool imu_axis_codes_valid(uint8_t right, uint8_t fwd, uint8_t up)
{
    uint8_t a = (uint8_t)(right & 3U);
    uint8_t b = (uint8_t)(fwd & 3U);
    uint8_t c = (uint8_t)(up & 3U);

    return (right < 8U) && (fwd < 8U) && (up < 8U) &&
        (a < 3U) && (b < 3U) && (c < 3U) &&
        (a != b) && (b != c) && (a != c);
}

static int32_t select_axis(const int32_t sensor[3], uint8_t code)
{
    int32_t value = sensor[code & 3U];
    return ((code & 4U) != 0U) ? -value : value;
}

void imu_map_axes(
    const int32_t sensor[3],
    uint8_t right,
    uint8_t fwd,
    uint8_t up,
    int32_t vehicle[3])
{
    vehicle[IMU_RIGHT] = select_axis(sensor, right);
    vehicle[IMU_FWD] = select_axis(sensor, fwd);
    vehicle[IMU_UP] = select_axis(sensor, up);
}

uint32_t imu_isqrt_u64(uint64_t value)
{
    uint64_t result = 0U;
    uint64_t bit = 1ULL << 62;

    while (bit > value)
    {
        bit >>= 2;
    }
    while (bit != 0U)
    {
        if (value >= (result + bit))
        {
            value -= result + bit;
            result = (result >> 1) + bit;
        }
        else
        {
            result >>= 1;
        }
        bit >>= 2;
    }
    return (uint32_t)result;
}

/*
 * atan(z) for z in [0, 1] (Q15) in centi-degrees, using
 * atan(z) ~= (pi/4) z + z (1 - z) (0.2447 + 0.0663 z)  [radians],
 * max error about 0.09 deg.
 */
static int32_t atan_unit_cdeg(uint32_t z_q15)
{
    uint32_t z_one_minus_z = (z_q15 * (32768U - z_q15)) >> 15;
    uint32_t poly = 1402U + ((380U * z_q15) >> 15);

    return (int32_t)(((4500U * z_q15) >> 15) + ((z_one_minus_z * poly) >> 15));
}

int16_t imu_atan2_ddeg(int32_t y, int32_t x)
{
    uint32_t ax = abs_u32(x);
    uint32_t ay = abs_u32(y);
    int32_t cdeg;

    if ((ax == 0U) && (ay == 0U))
    {
        return 0;
    }

    if (ay <= ax)
    {
        cdeg = atan_unit_cdeg((uint32_t)(((uint64_t)ay << 15) / ax));
    }
    else
    {
        cdeg = 9000 - atan_unit_cdeg((uint32_t)(((uint64_t)ax << 15) / ay));
    }

    if (x < 0)
    {
        cdeg = 18000 - cdeg;
    }
    if (y < 0)
    {
        cdeg = -cdeg;
    }
    return (int16_t)((cdeg >= 0) ? ((cdeg + 5) / 10) : ((cdeg - 5) / 10));
}

int16_t imu_wrap_ddeg(int32_t angle_ddeg)
{
    int32_t wrapped = angle_ddeg % 3600;

    if (wrapped >= 1800)
    {
        wrapped -= 3600;
    }
    else if (wrapped < -1800)
    {
        wrapped += 3600;
    }
    return (int16_t)wrapped;
}

int32_t imu_sin_q15(int32_t angle_ddeg)
{
    int32_t magnitude = (angle_ddeg < 0) ? -angle_ddeg : angle_ddeg;
    int32_t degree;
    int32_t tenth;
    int32_t value;

    if (magnitude > 900)
    {
        magnitude = 900;
    }
    degree = magnitude / 10;
    tenth = magnitude % 10;
    value = (int32_t)sin_table_q15[degree];
    if (tenth != 0)
    {
        /* Table is increasing on 0..90 deg, so the step is non-negative. */
        value += ((((int32_t)sin_table_q15[degree + 1] - value) * tenth) + 5) / 10;
    }
    return (angle_ddeg < 0) ? -value : value;
}

int16_t imu_pitch_ddeg(const int32_t accel[3])
{
    int64_t horizontal_sq =
        ((int64_t)accel[IMU_RIGHT] * accel[IMU_RIGHT]) +
        ((int64_t)accel[IMU_UP] * accel[IMU_UP]);

    return imu_atan2_ddeg(
        accel[IMU_FWD],
        (int32_t)imu_isqrt_u64((uint64_t)horizontal_sq));
}

int16_t imu_roll_ddeg(const int32_t accel[3])
{
    return imu_atan2_ddeg(-accel[IMU_RIGHT], accel[IMU_UP]);
}

bool imu_heading_ddeg(
    const int32_t accel[3],
    const int32_t mag[3],
    uint16_t *heading_ddeg)
{
    int64_t east[3];
    int64_t east_fwd;
    int64_t north_fwd;
    uint64_t largest;
    uint32_t accel_norm;
    int32_t angle;

    if ((accel == NULL) || (mag == NULL) || (heading_ddeg == NULL))
    {
        return false;
    }

    accel_norm = imu_isqrt_u64(
        (uint64_t)(((int64_t)accel[0] * accel[0]) +
                   ((int64_t)accel[1] * accel[1]) +
                   ((int64_t)accel[2] * accel[2])));
    if (accel_norm == 0U)
    {
        return false;
    }

    /* East = mag x up; north = up x east. Both lie in the horizontal plane. */
    east[0] = ((int64_t)mag[1] * accel[2]) - ((int64_t)mag[2] * accel[1]);
    east[1] = ((int64_t)mag[2] * accel[0]) - ((int64_t)mag[0] * accel[2]);
    east[2] = ((int64_t)mag[0] * accel[1]) - ((int64_t)mag[1] * accel[0]);

    /* Heading = atan2(fwd . east, fwd . north); north carries an extra |a|. */
    east_fwd = east[IMU_FWD] * (int64_t)accel_norm;
    north_fwd = ((int64_t)accel[IMU_UP] * east[IMU_RIGHT]) -
        ((int64_t)accel[IMU_RIGHT] * east[IMU_UP]);

    if ((east_fwd == 0) && (north_fwd == 0))
    {
        return false;
    }

    largest = abs_u64(east_fwd);
    if (abs_u64(north_fwd) > largest)
    {
        largest = abs_u64(north_fwd);
    }
    while (largest >= (1ULL << 30))
    {
        east_fwd /= 2;
        north_fwd /= 2;
        largest >>= 1;
    }

    angle = imu_atan2_ddeg((int32_t)east_fwd, (int32_t)north_fwd);
    if (angle < 0)
    {
        angle += 3600;
    }
    if (angle >= 3600)
    {
        angle -= 3600;
    }
    *heading_ddeg = (uint16_t)angle;
    return true;
}
