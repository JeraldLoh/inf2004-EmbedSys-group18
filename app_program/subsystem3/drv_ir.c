#include "drv_ir.h"

#include <bsp/libbsp.h>
#if APP_CAR_RUN
#include "../../device/adc/sysdepend/rp2040/adc_sysdep.h"
#endif

/* RP2040 hardware timer, raw low word: counts microseconds since boot and
 * wraps every ~71 minutes. Unsigned subtraction (now - then) still gives the
 * right difference across a wrap, which is why all timing uses uint32_t. */
#ifndef TIMER_TIMERAWL
#define TIMER_TIMERAWL (0x40054000UL + 0x28UL)
#endif

#if APP_CAR_RUN
/* Buddy 3 owns the ADC in the normal car profile. Poll the one-shot
 * hardware conversion without entering the kernel device/IRQ wait path. */
#define S3_ADC_WAIT_US 250U

LOCAL bool wait_for_adc(UW address, UW mask)
{
    uint32_t started = drv_ir_now_us();
    do
    {
        if ((in_w(address) & mask) != 0U) return true;
    } while ((uint32_t)(drv_ir_now_us() - started) < S3_ADC_WAIT_US);
    return false;
}
#else
LOCAL ID adc_dd = E_NOEXS;
LOCAL uint32_t next_open_us;
LOCAL unsigned adc_failures;
#endif

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

#if APP_CAR_RUN
    /* Board startup releases the ADC reset and configures GP26 (and GP28
     * only in the analogue-line profile). Enable FIFO with IRQs disabled. */
    out_w(ADC_INTE, 0U);
    out_w(ADC_FCS, (1U << ADC_FCS_THRESH_POS) | ADC_FCS_EN);
    out_w(ADC_CS, ADC_CS_EN);
#endif
    return E_OK;
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

LOCAL ER read_adc(W channel, uint16_t *raw)
{
#if APP_CAR_RUN
    if (raw == NULL || channel < 0 || channel >= ADC_CH_NUM) return E_PAR;
    if (!wait_for_adc(ADC_CS, ADC_CS_READY)) return E_TMOUT;
    /* Discard a late completion from a previously timed-out conversion. */
    for (unsigned i = 0; i < 4U && (in_w(ADC_FCS) & ADC_FCS_LEVEL) != 0U; ++i)
        (void)in_w(ADC_FIFO);
    out_w(ADC_CS, ADC_CS_EN | ((UW)channel << ADC_CS_AINSEL_POS) | ADC_CS_STRAT_ONCE);
    if (!wait_for_adc(ADC_FCS, ADC_FCS_LEVEL)) return E_TMOUT;
    UW value = in_w(ADC_FIFO);
    if ((in_w(ADC_CS) & ADC_CS_ERR) != 0U) return E_IO;
    *raw = (uint16_t)(value & 0x0FFFU);
    return E_OK;
#else
    UW value = 0U;
    SZ asize = 0;
    ER er;

    if (raw == NULL)
    {
        return E_PAR;
    }
    if (adc_dd < E_OK)
    {
        uint32_t now = drv_ir_now_us();
        if (next_open_us != 0U && (int32_t)(now - next_open_us) < 0)
        {
            return adc_dd;
        }
        adc_dd = tk_opn_dev((UB *)"adca", TD_READ);
        if (adc_dd < E_OK)
        {
            next_open_us = now + 1000000U;
            return adc_dd;
        }
    }
    /* For the ADC device, "start" is the channel number and size is 1. */
    er = tk_srea_dev(adc_dd, channel, &value, 1, &asize);
    if (er < E_OK || asize != 1)
    {
        if (er == E_OK) er = E_IO;
        if (++adc_failures >= 8U)
        {
            (void)tk_cls_dev(adc_dd, 0);
            adc_dd = er;
            adc_failures = 0U;
            next_open_us = drv_ir_now_us() + 1000000U;
        }
        return er;
    }
    adc_failures = 0U;
    *raw = (uint16_t)(value & 0x0FFFU);
    return E_OK;
#endif
}

EXPORT ER drv_ir_read_barcode(uint16_t *raw)
{
    return read_adc(S3_BARCODE_ADC_CHANNEL, raw);
}

/* GP28 is switched to analogue mode by DEVCONF_ADC_PIN_INIT_2, which is TRUE
 * only in LINE_PID=1 images. */
EXPORT ER drv_ir_read_line_analogue(uint16_t *raw)
{
    return read_adc(S3_LINE_ADC_CHANNEL, raw);
}

EXPORT uint32_t drv_ir_now_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}
