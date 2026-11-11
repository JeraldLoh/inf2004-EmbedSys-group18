/*
 * LSM303DLHC register access over a software (bit-banged) I2C master on
 * GP2 (SDA) / GP3 (SCL). Register values from the ST LSM303DLHC datasheet
 * (DocID018771); bus timing from NXP UM10204.
 *
 * Why not the course I2C device ("iicb", I2C1): in every Wi-Fi (CYW43)
 * build the I2C1 block stopped transmitting. With master mode, target
 * address, clock, reset, pads and pin function all verified correct, it kept
 * one byte in its TX FIFO with no bus activity, so every transfer timed out
 * and the IMU never calibrated. The same code worked in non-Wi-Fi builds.
 * Driving the two pins from SIO removes the dependency on the I2C1 block.
 *
 * Cost: about 1 ms of busy CPU per 20 ms sample (accelerometer + magnetometer
 * at roughly 150 kHz) in the IMU task. The master owns the clock, so being
 * preempted mid-transfer only stretches the transfer; the LSM303 has no bus
 * timeout. Only the IMU task may call these functions (TEAM.md section 3).
 */
#include "drv_lsm303.h"

#include <tm/tmonitor.h>

#include "../project_config/pinmap.h"
#include "s4_config.h"

/* Accelerometer registers. Bit 7 of the sub-address enables auto-increment. */
#define LSM303_CTRL_REG1_A 0x20U
#define LSM303_CTRL_REG4_A 0x23U
#define LSM303_CTRL_REG5_A 0x24U
#define LSM303_CTRL5_BOOT 0x80U   /* reboot memory content (self-clearing) */
#define LSM303_BOOT_MS 5U         /* time for the reboot to finish */
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

/*
 * Bus timing. The 1 us timer makes a 3 us request last 2..3 us, so a bit
 * takes 4..6 us plus instruction time: at most 250 kHz, inside the LSM303's
 * 400 kHz limit, and slow enough for the GY-511 pull-ups on a Grove cable.
 */
#define SW_I2C_HALF_US 3U
#define SW_I2C_RECOVER_PULSES 9U        /* 8 data bits + ACK */
#define SW_I2C_SCL_WAIT_US 1000U        /* clock stretching, at most */

#define SDA_MASK (1UL << PIN_IMU_SDA)
#define SCL_MASK (1UL << PIN_IMU_SCL)

/* Diagnostics for drv_lsm303_print_bus(). Single writer: the IMU task. */
enum
{
    OPEN_STEP_NONE = 0, OPEN_STEP_RECOVER, OPEN_STEP_BOOT, OPEN_STEP_BOOT_WAIT,
    OPEN_STEP_CTRL1_WRITE, OPEN_STEP_CTRL1_READ, OPEN_STEP_CTRL4_WRITE,
    OPEN_STEP_MAG, OPEN_STEP_DONE, OPEN_STEP_FAILED, OPEN_STEP_COUNT
};
static volatile UB open_step = OPEN_STEP_NONE;
static volatile UB bus_before = 0xFFU;   /* bit0 SDA, bit1 SCL; FF = never */
static volatile UB recover_pulses = 0U;
static volatile UW open_count = 0U;
static volatile ER last_open_er = E_OK;
static volatile UW nack_count = 0U;      /* sensor did not acknowledge */
static volatile UW stuck_count = 0U;     /* SCL held low past the wait */

/* ---------------------------------------------------------------------- */
/* Pin level helpers: open drain, low = drive 0, high = release (pull-up). */

static void delay_us(UW us)
{
    UW start = in_w(TIMER_TIMERAWL);

    while ((UW)(in_w(TIMER_TIMERAWL) - start) < us)
    {
    }
}

static BOOL sda_high(void)
{
    return ((in_w(GPIO_IN) & SDA_MASK) != 0UL) ? TRUE : FALSE;
}

static BOOL scl_high(void)
{
    return ((in_w(GPIO_IN) & SCL_MASK) != 0UL) ? TRUE : FALSE;
}

static void sda_low(void)
{
    out_w(GPIO_OE_SET, SDA_MASK);
}

static void sda_release(void)
{
    out_w(GPIO_OE_CLR, SDA_MASK);
}

static void scl_low(void)
{
    out_w(GPIO_OE_SET, SCL_MASK);
}

/* Release SCL and wait for it to rise (a slave may stretch the clock). */
static BOOL scl_release(void)
{
    UW waited = 0U;

    out_w(GPIO_OE_CLR, SCL_MASK);
    while (!scl_high())
    {
        if (waited >= SW_I2C_SCL_WAIT_US)
        {
            stuck_count++;
            return FALSE;
        }
        delay_us(1U);
        waited++;
    }
    return TRUE;
}

static void half_bit(void)
{
    delay_us(SW_I2C_HALF_US);
}

/* ---------------------------------------------------------------------- */
/* Bus conditions and bytes (NXP UM10204 section 3.1). */

/* START, or repeated START: SDA falls while SCL is high. Leaves SCL low. */
static BOOL bus_start(void)
{
    sda_release();
    half_bit();
    if (!scl_release())
    {
        return FALSE;
    }
    half_bit();
    sda_low();
    half_bit();
    scl_low();
    half_bit();
    return TRUE;
}

/* STOP: SDA rises while SCL is high. Leaves the bus idle (both high). */
static void bus_stop(void)
{
    sda_low();
    half_bit();
    (void)scl_release();
    half_bit();
    sda_release();
    half_bit();
}

/* Send 8 bits MSB first; TRUE if the slave pulled SDA low for ACK. */
static BOOL bus_write_byte(UB value)
{
    UINT bit;
    BOOL ack;

    for (bit = 0U; bit < 8U; bit++)
    {
        if ((value & 0x80U) != 0U)
        {
            sda_release();
        }
        else
        {
            sda_low();
        }
        value = (UB)(value << 1);
        half_bit();
        if (!scl_release())
        {
            return FALSE;
        }
        half_bit();
        scl_low();
    }
    sda_release();
    half_bit();
    if (!scl_release())
    {
        return FALSE;
    }
    half_bit();
    ack = sda_high() ? FALSE : TRUE;
    scl_low();
    if (!ack)
    {
        nack_count++;
    }
    return ack;
}

/* Read 8 bits MSB first, then ACK (more to come) or NACK (last byte). */
static UB bus_read_byte(BOOL ack)
{
    UB value = 0U;
    UINT bit;

    sda_release();
    for (bit = 0U; bit < 8U; bit++)
    {
        half_bit();
        (void)scl_release();
        half_bit();
        value = (UB)((value << 1) | (sda_high() ? 1U : 0U));
        scl_low();
    }
    if (ack)
    {
        sda_low();
    }
    half_bit();
    (void)scl_release();
    half_bit();
    scl_low();
    sda_release();
    return value;
}

/*
 * Take GP2/GP3 from the I2C1 block (the course driver claims them at boot)
 * and make them open-drain SIO pins with pull-ups, then free the bus.
 *
 * Recovery: a Pico reset can land in the middle of a read, leaving the
 * LSM303 waiting for clock pulses with SDA held low. Clock SCL until it lets
 * go (at most 9 pulses), then send STOP. Harmless on an idle bus.
 */
static void bus_init_and_recover(void)
{
    const UW mask = SDA_MASK | SCL_MASK;
    UINT pulses;

    out_w(GPIO_OE_CLR, mask);   /* released ... */
    out_w(GPIO_OUT_CLR, mask);  /* ... and 0 whenever driven */
    out_w(GPIO(PIN_IMU_SDA), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO(PIN_IMU_SCL), GPIO_IE | GPIO_DRIVE_4MA | GPIO_PUE | GPIO_SHEMITT);
    out_w(GPIO_CTRL(PIN_IMU_SDA), GPIO_CTRL_FUNCSEL_SIO);
    out_w(GPIO_CTRL(PIN_IMU_SCL), GPIO_CTRL_FUNCSEL_SIO);
    half_bit();
    (void)scl_release();
    bus_before = (UB)((sda_high() ? 1U : 0U) | (scl_high() ? 2U : 0U));

    for (pulses = 0U; (pulses < SW_I2C_RECOVER_PULSES) && !sda_high(); pulses++)
    {
        scl_low();
        half_bit();
        (void)scl_release();
        half_bit();
    }
    recover_pulses = (UB)pulses;

    scl_low();
    half_bit();
    bus_stop();
}

/* ---------------------------------------------------------------------- */
/* Register access. */

static ER write_reg(UB address, UB reg, UB value)
{
    ER er = E_IO;

    if (bus_start() &&
        bus_write_byte((UB)(address << 1)) &&
        bus_write_byte(reg) &&
        bus_write_byte(value))
    {
        er = E_OK;
    }
    bus_stop();
    return er;
}

/* Write the register address, repeated START, then read len bytes. */
static ER read_regs(UB address, UB reg, UB *buf, SZ len)
{
    SZ i;

    if (!bus_start() ||
        !bus_write_byte((UB)(address << 1)) ||
        !bus_write_byte(reg) ||
        !bus_start() ||
        !bus_write_byte((UB)((address << 1) | 1U)))
    {
        bus_stop();
        return E_IO;
    }
    for (i = 0; i < len; i++)
    {
        buf[i] = bus_read_byte((i + 1 < len) ? TRUE : FALSE);
    }
    bus_stop();
    return E_OK;
}

/* ---------------------------------------------------------------------- */

ER drv_lsm303_open(drv_lsm303_t *dev, UINT fs_code)
{
    UB id[3];
    UB check = 0U;
    ER er;

    if (dev == NULL)
    {
        return E_PAR;
    }
    dev->open = 0U;
    dev->mag_present = 0U;
    dev->fs_code = fs_code & 3U;
    open_count++;

    open_step = OPEN_STEP_RECOVER;
    bus_init_and_recover();

    /* Reboot the accelerometer so it starts from its power-on state whatever
       the Pico was doing before a reset. A failure here shows up in the
       register check below, so its result is not needed. */
    open_step = OPEN_STEP_BOOT;
    (void)write_reg(S4_ACCEL_ADDR, LSM303_CTRL_REG5_A, LSM303_CTRL5_BOOT);
    open_step = OPEN_STEP_BOOT_WAIT;
    (void)tk_dly_tsk(LSM303_BOOT_MS);

    /* The DLHC accelerometer has no WHO_AM_I: write a register and read it
       back to prove the chip answers at this address. */
    open_step = OPEN_STEP_CTRL1_WRITE;
    er = write_reg(S4_ACCEL_ADDR, LSM303_CTRL_REG1_A, LSM303_CTRL1_100HZ_XYZ);
    if (er >= E_OK)
    {
        open_step = OPEN_STEP_CTRL1_READ;
        er = read_regs(S4_ACCEL_ADDR, LSM303_CTRL_REG1_A, &check, 1);
    }
    if ((er >= E_OK) && (check != LSM303_CTRL1_100HZ_XYZ))
    {
        er = E_IO;
    }
    if (er >= E_OK)
    {
        open_step = OPEN_STEP_CTRL4_WRITE;
        er = write_reg(S4_ACCEL_ADDR, LSM303_CTRL_REG4_A,
                       (UB)(LSM303_CTRL4_BDU_HR | ((fs_code & 3U) << 4)));
    }
    if (er < E_OK)
    {
        last_open_er = er;
        open_step = OPEN_STEP_FAILED;
        return er;
    }

    /* Magnetometer identification registers read "H43". A missing
       magnetometer only disables heading; it is not fatal. */
    open_step = OPEN_STEP_MAG;
    if ((read_regs(S4_MAG_ADDR, LSM303_IRA_REG_M, id, 3) >= E_OK) &&
        (id[0] == 'H') && (id[1] == '4') && (id[2] == '3') &&
        (write_reg(S4_MAG_ADDR, LSM303_CRA_REG_M, LSM303_CRA_75HZ) >= E_OK) &&
        (write_reg(S4_MAG_ADDR, LSM303_CRB_REG_M, LSM303_CRB_1_3_GAUSS) >= E_OK) &&
        (write_reg(S4_MAG_ADDR, LSM303_MR_REG_M, LSM303_MR_CONTINUOUS) >= E_OK))
    {
        dev->mag_present = 1U;
    }

    dev->open = 1U;
    last_open_er = E_OK;
    open_step = OPEN_STEP_DONE;
    return E_OK;
}

void drv_lsm303_close(drv_lsm303_t *dev)
{
    if (dev != NULL)
    {
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
    er = read_regs(S4_ACCEL_ADDR,
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
    return read_regs(S4_MAG_ADDR, LSM303_OUT_X_H_M, raw, 6);
}

void drv_lsm303_print_bus(void)
{
    static const char *const steps[OPEN_STEP_COUNT] = {
        "none", "bus-recover", "boot-write", "boot-wait", "ctrl1-write",
        "ctrl1-readback", "ctrl4-write", "mag-setup", "done", "failed"
    };
    UB step = open_step;
    UB before = bus_before;
    UW in = in_w(GPIO_IN);

    tm_printf((UB *)"[B4 I2C] software bus | opens=%lu step=%s last_er=%d nacks=%lu "
              "scl_stuck=%lu | before recover SDA=%d SCL=%d pulses=%d | now SDA=%d SCL=%d "
              "func=%lu/%lu (5=SIO)\n",
              (unsigned long)open_count,
              (step < OPEN_STEP_COUNT) ? steps[step] : "?",
              (int)last_open_er, (unsigned long)nack_count, (unsigned long)stuck_count,
              (before == 0xFFU) ? -1 : (int)(before & 1U),
              (before == 0xFFU) ? -1 : (int)((before >> 1) & 1U),
              (int)recover_pulses,
              (int)((in >> PIN_IMU_SDA) & 1U), (int)((in >> PIN_IMU_SCL) & 1U),
              (unsigned long)(in_w(GPIO_CTRL(PIN_IMU_SDA)) & 0x1FUL),
              (unsigned long)(in_w(GPIO_CTRL(PIN_IMU_SCL)) & 0x1FUL));
}
