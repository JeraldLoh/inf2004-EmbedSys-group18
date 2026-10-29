/*
 * LSM303DLHC register access through the course I2C device ("iicb", I2C1).
 * Register values from the ST LSM303DLHC datasheet (DocID018771).
 */
#include "drv_lsm303.h"

#include "../../device/include/dev_i2c.h"
#include "s4_config.h"

/* Accelerometer registers. Bit 7 of the sub-address enables auto-increment. */
#define LSM303_CTRL_REG1_A 0x20U
#define LSM303_CTRL_REG4_A 0x23U
#define LSM303_STATUS_REG_A 0x27U
#define LSM303_AUTO_INCREMENT 0x80U
#define LSM303_ZYXDA 0x08U

/* 100 Hz output data rate, normal power, X/Y/Z enabled. */
#define LSM303_CTRL1_100HZ_XYZ 0x57U
/* Block data update + high resolution; FS bits are 5..4. */
#define LSM303_CTRL4_BDU_HR 0x88U

/* Magnetometer registers (auto-increment is implicit). */
#define LSM303_CRA_REG_M 0x00U
#define LSM303_CRB_REG_M 0x01U
#define LSM303_MR_REG_M 0x02U
#define LSM303_OUT_X_H_M 0x03U
#define LSM303_IRA_REG_M 0x0AU

#define LSM303_CRA_75HZ 0x18U
#define LSM303_CRB_1_3_GAUSS 0x20U  /* 1100 LSB/gauss X/Y, 980 Z */
#define LSM303_MR_CONTINUOUS 0x00U

static ER write_reg(ID dd, UW address, UB reg, UB value)
{
    UB data[2];
    SZ asize;

    data[0] = reg;
    data[1] = value;
    return tk_swri_dev(dd, (W)address, data, (SZ)sizeof(data), &asize);
}

/* Write the register address, repeated start, then read len bytes. */
static ER read_regs(ID dd, UW address, UB reg, UB *buf, SZ len)
{
    T_I2C_EXEC exec;
    SZ asize;

    exec.sadr = address;
    exec.snd_size = 1;
    exec.snd_data = &reg;
    exec.rcv_size = len;
    exec.rcv_data = buf;
    return tk_swri_dev(dd, TDN_I2C_EXEC, &exec, (SZ)sizeof(exec), &asize);
}

ER drv_lsm303_open(drv_lsm303_t *dev, UINT fs_code)
{
    UB id[3];
    UB check;
    ID dd;
    ER er;

    if (dev == NULL)
    {
        return E_PAR;
    }
    dev->open = 0U;
    dev->mag_present = 0U;
    dev->fs_code = fs_code & 3U;

    dd = tk_opn_dev((UB *)S4_I2C_DEVICE, TD_UPDATE);
    if (dd < E_OK)
    {
        return (ER)dd;
    }

    /* The DLHC accelerometer has no WHO_AM_I: write a register and read it
       back to prove the chip answers at this address. */
    er = write_reg(dd, S4_ACCEL_ADDR, LSM303_CTRL_REG1_A, LSM303_CTRL1_100HZ_XYZ);
    if (er >= E_OK)
    {
        er = read_regs(dd, S4_ACCEL_ADDR, LSM303_CTRL_REG1_A, &check, 1);
    }
    if ((er >= E_OK) && (check != LSM303_CTRL1_100HZ_XYZ))
    {
        er = E_IO;
    }
    if (er >= E_OK)
    {
        er = write_reg(dd, S4_ACCEL_ADDR, LSM303_CTRL_REG4_A,
                       (UB)(LSM303_CTRL4_BDU_HR | ((fs_code & 3U) << 4)));
    }
    if (er < E_OK)
    {
        (void)tk_cls_dev(dd, 0);
        return er;
    }

    /* Magnetometer identification registers read "H43". A missing
       magnetometer only disables heading; it is not fatal. */
    if ((read_regs(dd, S4_MAG_ADDR, LSM303_IRA_REG_M, id, 3) >= E_OK) &&
        (id[0] == 'H') && (id[1] == '4') && (id[2] == '3') &&
        (write_reg(dd, S4_MAG_ADDR, LSM303_CRA_REG_M, LSM303_CRA_75HZ) >= E_OK) &&
        (write_reg(dd, S4_MAG_ADDR, LSM303_CRB_REG_M, LSM303_CRB_1_3_GAUSS) >= E_OK) &&
        (write_reg(dd, S4_MAG_ADDR, LSM303_MR_REG_M, LSM303_MR_CONTINUOUS) >= E_OK))
    {
        dev->mag_present = 1U;
    }

    dev->dd = dd;
    dev->open = 1U;
    return E_OK;
}

void drv_lsm303_close(drv_lsm303_t *dev)
{
    if ((dev != NULL) && dev->open)
    {
        (void)tk_cls_dev(dev->dd, 0);
        dev->open = 0U;
        dev->mag_present = 0U;
    }
}

ER drv_lsm303_read_accel(drv_lsm303_t *dev, UB raw[6], BOOL *fresh)
{
    UB buf[7];
    ER er;
    INT i;

    if ((dev == NULL) || !dev->open || (raw == NULL) || (fresh == NULL))
    {
        return E_OBJ;
    }
    er = read_regs(dev->dd, S4_ACCEL_ADDR,
                   (UB)(LSM303_STATUS_REG_A | LSM303_AUTO_INCREMENT), buf, 7);
    if (er < E_OK)
    {
        return er;
    }
    *fresh = ((buf[0] & LSM303_ZYXDA) != 0U) ? TRUE : FALSE;
    for (i = 0; i < 6; i++)
    {
        raw[i] = buf[i + 1];
    }
    return E_OK;
}

ER drv_lsm303_read_mag(drv_lsm303_t *dev, UB raw[6])
{
    if ((dev == NULL) || !dev->open || !dev->mag_present || (raw == NULL))
    {
        return E_OBJ;
    }
    return read_regs(dev->dd, S4_MAG_ADDR, LSM303_OUT_X_H_M, raw, 6);
}
