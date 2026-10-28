#include "drv_ir.h"

#include <bsp/libbsp.h>

/* RP2040 hardware timer, raw low word: counts microseconds since boot and
 * wraps every ~71 minutes. Unsigned subtraction (now - then) still gives the
 * right difference across a wrap, which is why all timing uses uint32_t. */
#ifndef TIMER_TIMERAWL
#define TIMER_TIMERAWL (0x40054000UL + 0x28UL)
#endif

LOCAL ID adc_dd = E_NOEXS;

EXPORT ER drv_ir_init(void)
{
    ER er;

    er = gpio_set_pin(S3_PIN_LINE_LEFT, GPIO_MODE_IN);
    if (er < E_OK)
    {
        return er;
    }
    er = gpio_set_pin(S3_PIN_LINE_RIGHT, GPIO_MODE_IN);
    if (er < E_OK)
    {
        return er;
    }

    if (S3_PIN_BARCODE_D0 != 0xFFU)
    {
        er = gpio_set_pin(S3_PIN_BARCODE_D0, GPIO_MODE_IN);
        if (er < E_OK)
        {
            return er;
        }
        /* The edge interrupt itself is enabled by the team's shared GPIO
         * dispatcher (one IO_BANK0 IRQ serves all pins: encoders, echo,
         * buttons), so it is not set up here. */
    }

    /* "adc" is the device name registered by device/adc (DEVCNF_ADC_DEVNAME).
     * NOTE: set DEVCONF_ADC_PIN_INIT_0 to TRUE in
     * device/adc/sysdepend/rp2040/adc_cnf_sysdep.h so the driver switches
     * GP26 to analogue mode (digital input and pulls off). That is a shared
     * file, so agree it with the team first. */
    adc_dd = tk_opn_dev((UB *)"adc", TD_READ);
    return (adc_dd < E_OK) ? adc_dd : E_OK;
}

EXPORT void drv_ir_read_line(bool *left_black, bool *right_black)
{
    if (left_black != NULL)
    {
        *left_black = (gpio_get_val(S3_PIN_LINE_LEFT) == S3_LINE_BLACK_LEVEL);
    }
    if (right_black != NULL)
    {
        *right_black = (gpio_get_val(S3_PIN_LINE_RIGHT) == S3_LINE_BLACK_LEVEL);
    }
}

EXPORT ER drv_ir_read_barcode(uint16_t *raw)
{
    UW value = 0U;
    SZ asize = 0;
    ER er;

    if ((raw == NULL) || (adc_dd < E_OK))
    {
        return E_PAR;
    }
    /* For the ADC device, "start" is the channel number and size is 1. */
    er = tk_srea_dev(adc_dd, S3_BARCODE_ADC_CHANNEL, &value, 1, &asize);
    if (er < E_OK)
    {
        return er;
    }
    *raw = (uint16_t)(value & 0x0FFFU);
    return E_OK;
}

EXPORT bool drv_ir_read_barcode_digital(void)
{
    if (S3_PIN_BARCODE_D0 == 0xFFU)
    {
        return false;
    }
    return gpio_get_val(S3_PIN_BARCODE_D0) == S3_BARCODE_D0_BLACK_LEVEL;
}

EXPORT uint32_t drv_ir_now_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}
