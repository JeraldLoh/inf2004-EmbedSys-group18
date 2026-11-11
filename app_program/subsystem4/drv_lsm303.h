#ifndef GROUP18_DRV_LSM303_H
#define GROUP18_DRV_LSM303_H

/*
 * GY-511 / LSM303DLHC driver over a software I2C master on GP2 (SDA) and
 * GP3 (SCL). The I2C1 hardware block stopped transmitting in Wi-Fi builds;
 * see drv_lsm303.c for the evidence.
 *
 * Owned by the Buddy 4 IMU task only: no other task may use GP2/GP3
 * (TEAM.md section 3). Every call busy-waits for one bus transaction
 * (an accelerometer read is about 0.5 ms) and returns a kernel error code:
 * E_IO if the sensor does not acknowledge or SCL stays low.
 */
#include <tk/tkernel.h>

typedef struct
{
    unsigned int open : 1;
    unsigned int mag_present : 1;
    unsigned int fs_code : 2;    /* IMU_ACCEL_FS_* in use */
} drv_lsm303_t;

/* Open the bus device, probe both chips and configure them. */
ER drv_lsm303_open(drv_lsm303_t *dev, UINT fs_code);

void drv_lsm303_close(drv_lsm303_t *dev);

/*
 * One burst read of STATUS_REG_A + OUT_X_L_A..OUT_Z_H_A.
 * *fresh is TRUE if the sensor flagged a new X/Y/Z sample (ZYXDA).
 */
ER drv_lsm303_read_accel(drv_lsm303_t *dev, UB raw[6], BOOL *fresh);

/* Burst read of OUT_X_H_M..OUT_Y_L_M (register order X, Z, Y). */
ER drv_lsm303_read_mag(drv_lsm303_t *dev, UB raw[6]);

/*
 * Diagnostics: print one "[B4 I2C]" line with the step drv_lsm303_open() is
 * in (or last finished), the SDA/SCL levels found before bus recovery, the
 * levels now, and the I2C1 controller status. Read-only; safe from any task.
 */
void drv_lsm303_print_bus(void);

#endif
