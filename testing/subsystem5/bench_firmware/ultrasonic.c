#include "ultrasonic.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "pico/stdlib.h"

#define ECHO_IDLE_TIMEOUT_US 60000u // wait for a previous echo to finish
#define ECHO_RISE_TIMEOUT_US 10000u // HC-SR04 raises ECHO ~0.5 ms after trigger
#define ECHO_HIGH_TIMEOUT_US 40000u // ~38 ms pulse means "no echo"

static volatile uint32_t s_rise_us;
static volatile uint32_t s_fall_us;
static volatile bool s_rose;
static volatile bool s_fell;
static unsigned s_echo_pin;

static void echo_irq(void) {
    uint32_t now = time_us_32();
    uint32_t events = gpio_get_irq_event_mask(s_echo_pin);
    if (events & GPIO_IRQ_EDGE_RISE) {
        gpio_acknowledge_irq(s_echo_pin, GPIO_IRQ_EDGE_RISE);
        s_rise_us = now;
        s_rose = true;
    }
    if (events & GPIO_IRQ_EDGE_FALL) {
        gpio_acknowledge_irq(s_echo_pin, GPIO_IRQ_EDGE_FALL);
        if (s_rose) {
            s_fall_us = now;
            s_fell = true;
        }
    }
}

void ultrasonic_init(ultrasonic_t *u, unsigned trig_pin, unsigned echo_pin) {
    u->trig_pin = trig_pin;
    u->echo_pin = echo_pin;
    u->min_cm = 2.0f;
    u->max_cm = 400.0f;
    ultrasonic_set_temperature_c(u, 20.0f);

    gpio_init(trig_pin);
    gpio_set_dir(trig_pin, GPIO_OUT);
    gpio_put(trig_pin, 0);

    gpio_init(echo_pin);
    gpio_set_dir(echo_pin, GPIO_IN);
    gpio_pull_down(echo_pin); // defined level if the sensor is unplugged

    s_echo_pin = echo_pin;
    // Raw handler: coexists with other buddies' GPIO IRQs (e.g. wheel encoders).
    gpio_add_raw_irq_handler(echo_pin, echo_irq);
    gpio_set_irq_enabled(echo_pin, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void ultrasonic_set_temperature_c(ultrasonic_t *u, float celsius) {
    float speed_m_s = 331.3f + 0.606f * celsius;
    u->cm_per_us = speed_m_s * 100.0f / 1e6f / 2.0f;
}

range_reading_t ultrasonic_read_once(const ultrasonic_t *u) {
    range_reading_t r = {RANGE_FAULT, 0.0f};

    uint32_t t0 = time_us_32();
    while (gpio_get(u->echo_pin)) {
        if (time_us_32() - t0 > ECHO_IDLE_TIMEOUT_US) return r;
    }

    s_rose = false;
    s_fell = false;
    gpio_put(u->trig_pin, 1);
    sleep_us(10);
    gpio_put(u->trig_pin, 0);

    t0 = time_us_32();
    while (!s_rose) {
        if (time_us_32() - t0 > ECHO_RISE_TIMEOUT_US) return r;
    }
    while (!s_fell) {
        if (time_us_32() - s_rise_us > ECHO_HIGH_TIMEOUT_US) {
            r.status = RANGE_NO_ECHO;
            return r;
        }
    }

    float cm = (float)(s_fall_us - s_rise_us) * u->cm_per_us;
    if (cm < u->min_cm) {
        r.status = RANGE_TOO_CLOSE;
    } else if (cm > u->max_cm) {
        r.status = RANGE_NO_ECHO;
    } else {
        r.status = RANGE_OK;
        r.cm = cm;
    }
    return r;
}

range_reading_t ultrasonic_read(const ultrasonic_t *u, size_t samples, uint32_t spacing_ms) {
    range_reading_t buf[RANGE_MAX_SAMPLES];
    if (samples == 0 || samples > RANGE_MAX_SAMPLES) samples = 1;
    for (size_t i = 0; i < samples; i++) {
        if (i > 0) sleep_ms(spacing_ms); // let stray echoes from the last ping die out
        buf[i] = ultrasonic_read_once(u);
    }
    return range_combine(buf, samples);
}
