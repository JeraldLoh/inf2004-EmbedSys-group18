#ifndef GROUP18_DRV_LSM303_H
#define GROUP18_DRV_LSM303_H

/*
 * GY-511 / LSM303DLHC driver over the course micro T-Kernel I2C device.
 *
 * Owned by the Buddy 4 IMU task only: no other task may use the I2C1 device
 * (TEAM.md section 3). Every call blocks the caller for one bus transaction
 * (about 0.3 ms at 400 kHz) and returns a kernel error code.
 *
 * Requires DEVCNF_USE_IIC = 1, I2C1 on GP2/GP3 and unit 1 started in
 * kernel/sysdepend/pico_rp2040/devinit.c; otherwise open fails with E_NOEXS.
 */
#include <tk/tkernel.h>

typedef struct
{
    ID dd;                       /* device descriptor while open */
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

#endif
