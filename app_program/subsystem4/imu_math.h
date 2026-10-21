#ifndef GROUP18_IMU_MATH_H
#define GROUP18_IMU_MATH_H

/*
 * Buddy 4 fixed-point IMU math. The RP2040 has no FPU and the course build
 * uses soft-float, so everything here is integer: angles are deci-degrees
 * (0.1 deg), sines are Q15, accelerations are milli-g.
 *
 * Vehicle frame (right-handed, matching the briefing's IMU figure):
 *   index 0 = RIGHT, 1 = FORWARD, 2 = UP.
 * At rest the accelerometer reads +1 g along UP.
 *
 * Portable: no kernel or hardware dependency.
 */
#include <stdbool.h>
#include <stdint.h>

enum
{
    IMU_RIGHT = 0,
    IMU_FWD = 1,
    IMU_UP = 2
};

/* 3-bit axis code: bits 1..0 = sensor axis (0 X, 1 Y, 2 Z), bit 2 = negate. */
#define IMU_AXIS_CODE(sensor_axis, negate) \
    ((uint8_t)(((sensor_axis) & 3U) | ((negate) ? 4U : 0U)))
#define IMU_AXIS_CODE_WIDTH 3U

/* LSM303DLHC accelerometer full-scale codes (CTRL_REG4_A FS bits). */
enum
{
    IMU_ACCEL_FS_2G = 0,
    IMU_ACCEL_FS_4G = 1,
    IMU_ACCEL_FS_8G = 2,
    IMU_ACCEL_FS_16G = 3
};

/* Sensitivity in high-resolution mode, mg per 12-bit LSB. */
uint8_t imu_accel_mg_per_lsb(uint8_t fs_code);

/*
 * Decode OUT_X_L_A..OUT_Z_H_A: little-endian, left-justified 12-bit.
 * saturated is set when any axis is at its full-scale limit.
 */
bool imu_decode_accel_mg(
    const uint8_t raw[6],
    uint8_t fs_code,
    int16_t out_mg[3],
    bool *saturated);

/*
 * Decode OUT_X_H_M..OUT_Y_L_M: big-endian, register order X, Z, Y.
 * Output is X, Y, Z with Z rescaled to the X/Y gain (1100/980 LSB/gauss at
 * +/-1.3 gauss). overflow is set if the sensor reported -4096 on any axis.
 */
bool imu_decode_mag(const uint8_t raw[6], int16_t out[3], bool *overflow);

/* True when the three codes are valid and use three different sensor axes. */
bool imu_axis_codes_valid(uint8_t right, uint8_t fwd, uint8_t up);

/* Sensor X/Y/Z to vehicle RIGHT/FORWARD/UP using the mounting codes. */
void imu_map_axes(
    const int32_t sensor[3],
    uint8_t right,
    uint8_t fwd,
    uint8_t up,
    int32_t vehicle[3]);

uint32_t imu_isqrt_u64(uint64_t value);

/* atan2 in deci-degrees, -1800..1800. Max error about 0.1 deg. */
int16_t imu_atan2_ddeg(int32_t y, int32_t x);

/* Wrap an angle to -1800..1799 deci-degrees. */
int16_t imu_wrap_ddeg(int32_t angle_ddeg);

/* sin() of -900..900 deci-degrees (clamped) as Q15. */
int32_t imu_sin_q15(int32_t angle_ddeg);

/* Nose-up pitch and right-side-down roll from a vehicle-frame gravity vector. */
int16_t imu_pitch_ddeg(const int32_t accel[3]);
int16_t imu_roll_ddeg(const int32_t accel[3]);

/*
 * Tilt-compensated magnetic heading, 0..3599 deci-degrees clockwise from
 * magnetic north to the vehicle's forward axis. accel and mag are in the
 * vehicle frame. Returns false if either vector is degenerate.
 */
bool imu_heading_ddeg(
    const int32_t accel[3],
    const int32_t mag[3],
    uint16_t *heading_ddeg);

#endif
