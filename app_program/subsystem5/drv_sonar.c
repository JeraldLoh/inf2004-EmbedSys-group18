/* Use the C library's size_t (range.h), not the kernel's (tk/syslib.h). */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>

#include "drv_sonar.h"

#include <bsp/libbsp.h>
#include <sys/sysdef.h>

#define SONAR_IDLE_TIMEOUT_US 60000U /* previous echo must have finished */
#define SONAR_RISE_TIMEOUT_US 10000U /* HC-SR04 raises ECHO ~0.5 ms after TRIG */
#define SONAR_HIGH_TIMEOUT_US 40000U /* ~38 ms pulse means nothing in range */
#define SONAR_TRIG_PULSE_US 10U
#define SONAR_MAX_ATTEMPTS_PER_SAMPLE 2U

static uint32_t now_us(void)
{
    return (uint32_t)in_w(TIMER_TIMERAWL);
}

/* Runs in the GPIO interrupt (core 0). */
static void on_echo(void *user, uint32_t rising, uint32_t falling,
    uint32_t observed_us)
{
    drv_sonar_t *sonar = user;

    if ((rising != 0U) && (falling != 0U))
    {
        sonar->both_in_one = true;
        return;
    }
    if (rising != 0U)
    {
        sonar->rise_us = observed_us;
        sonar->rose = true;
    }
    else if ((falling != 0U) && sonar->rose)
    {
        sonar->fall_us = observed_us;
        sonar->fell = true;
    }
}

ER drv_sonar_init(drv_sonar_t *sonar, gpio_dispatcher_t *dispatcher,
    UINT trig_pin, UINT echo_pin)
{
    if ((sonar == NULL) || (dispatcher == NULL) || (trig_pin >= 30U) ||
        (echo_pin >= 30U) || (trig_pin == echo_pin))
    {
        return E_PAR;
    }
    sonar->trig_pin = trig_pin;
    sonar->echo_pin = echo_pin;
    sonar->cm_per_us = range_cm_per_us(20.0f);
    sonar->min_cm = 2.0f;
    sonar->max_cm = 400.0f;
    sonar->dispatcher = dispatcher;
    sonar->attached = false;
    sonar->rose = false;
    sonar->fell = false;
    sonar->both_in_one = false;

    if ((gpio_set_val(trig_pin, 0U) != E_OK) ||
        (gpio_set_pin(trig_pin, GPIO_MODE_OUT) != E_OK) ||
        (gpio_set_pin(echo_pin, GPIO_MODE_IN) != E_OK))
    {
        return E_IO;
    }
    /* Defined low level if the sensor is unplugged (ECHO goes through a
     * divider to the pin). */
    clr_w(GPIO(echo_pin), GPIO_PUE);
    set_w(GPIO(echo_pin), GPIO_PDE);

    if (!gpio_dispatcher_attach(dispatcher, (uint32_t)1U << echo_pin, on_echo,
        sonar, false))
    {
        return E_OBJ;
    }
    sonar->attached = true;
    return E_OK;
}

ER drv_sonar_deinit(drv_sonar_t *sonar)
{
    if ((sonar == NULL) || !sonar->attached)
    {
        return E_OBJ;
    }
    if (!gpio_dispatcher_detach(sonar->dispatcher,
        (uint32_t)1U << sonar->echo_pin))
    {
        return E_SYS;
    }
    sonar->attached = false;
    (void)gpio_set_val(sonar->trig_pin, 0U);
    return E_OK;
}

static bool waited_too_long(uint32_t since_us, uint32_t limit_us)
{
    return (now_us() - since_us) > limit_us;
}

/* One ping. Returns false if both edges arrived in one interrupt, so the
 * width is unknown and the ping should simply be repeated. */
static bool ping(drv_sonar_t *sonar, range_reading_t *out)
{
    uint32_t start;
    range_reading_t fault = {RANGE_FAULT, 0.0f};
    range_reading_t no_echo = {RANGE_NO_ECHO, 0.0f};

    start = now_us();
    while (gpio_get_val(sonar->echo_pin) != 0U)
    {
        if (waited_too_long(start, SONAR_IDLE_TIMEOUT_US))
        {
            *out = fault; /* ECHO stuck high */
            return true;
        }
        (void)tk_dly_tsk(1);
    }

    sonar->rose = false;
    sonar->fell = false;
    sonar->both_in_one = false;
    (void)gpio_set_val(sonar->trig_pin, 1U);
    start = now_us();
    while (!waited_too_long(start, SONAR_TRIG_PULSE_US))
    {
        /* 10 us pulse: too short to sleep */
    }
    (void)gpio_set_val(sonar->trig_pin, 0U);

    start = now_us();
    while (!sonar->rose && !sonar->both_in_one)
    {
        if (waited_too_long(start, SONAR_RISE_TIMEOUT_US))
        {
            *out = fault; /* no ECHO pulse at all: power, wiring or pin */
            return true;
        }
        (void)tk_dly_tsk(1);
    }
    while (!sonar->fell && !sonar->both_in_one)
    {
        if (waited_too_long(sonar->rise_us, SONAR_HIGH_TIMEOUT_US))
        {
            *out = no_echo;
            return true;
        }
        (void)tk_dly_tsk(1);
    }
    if (sonar->both_in_one)
    {
        return false;
    }
    *out = range_from_echo_us(sonar->fall_us - sonar->rise_us,
        sonar->cm_per_us, sonar->min_cm, sonar->max_cm);
    return true;
}

range_reading_t drv_sonar_read(drv_sonar_t *sonar, size_t samples,
    UINT spacing_ms)
{
    range_reading_t buf[RANGE_MAX_SAMPLES];
    range_reading_t fault = {RANGE_FAULT, 0.0f};
    size_t taken = 0U;
    size_t attempts = 0U;

    if ((sonar == NULL) || !sonar->attached)
    {
        return fault;
    }
    if ((samples == 0U) || (samples > RANGE_MAX_SAMPLES))
    {
        samples = 1U;
    }
    while ((taken < samples) &&
        (attempts < samples * SONAR_MAX_ATTEMPTS_PER_SAMPLE))
    {
        if (attempts > 0U)
        {
            (void)tk_dly_tsk(spacing_ms); /* let stray echoes die out */
        }
        attempts++;
        if (ping(sonar, &buf[taken]))
        {
            taken++;
        }
    }
    return (taken == 0U) ? fault : range_combine(buf, taken);
}
