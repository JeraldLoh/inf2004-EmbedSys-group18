#include "barcode_sensor.h"

#include <stddef.h>

uint16_t barcode_sensor_darkness(uint16_t raw)
{
    if (raw > BARCODE_ADC_MAX)
    {
        raw = BARCODE_ADC_MAX;
    }
#if BARCODE_BLACK_IS_HIGH
    return raw;
#else
    return (uint16_t)(BARCODE_ADC_MAX - raw);
#endif
}

/* Recompute both thresholds from the current black and white levels.
 * Returns false (and leaves the old thresholds) if contrast is too low. */
static bool update_thresholds(barcode_sensor_t *sensor)
{
    uint32_t span;
    uint32_t mid;
    uint32_t band;

    if (sensor->black_level <= sensor->white_level)
    {
        return false;
    }
    span = (uint32_t)sensor->black_level - sensor->white_level;
    if (span < BARCODE_MIN_CONTRAST)
    {
        return false;
    }
    mid = sensor->white_level + (span / 2U);
    band = (span * BARCODE_HYSTERESIS_PERCENT) / 100U;
    sensor->threshold_low = (uint16_t)(mid - band);
    sensor->threshold_high = (uint16_t)(mid + band);
    return true;
}

#if BARCODE_ADAPT
/* level += (target - level) / 2^shift, in signed arithmetic. */
static uint16_t follow(uint16_t level, uint16_t target)
{
    int32_t diff = (int32_t)target - (int32_t)level;
    return (uint16_t)((int32_t)level + (diff / (int32_t)(1L << BARCODE_ADAPT_SHIFT)));
}
#endif

void barcode_sensor_reset(barcode_sensor_t *sensor)
{
    if (sensor == NULL)
    {
        return;
    }
    sensor->white_level = 0xFFFFU;   /* "nothing seen yet" */
    sensor->black_level = 0U;
    sensor->threshold_low = 0U;
    sensor->threshold_high = 0U;
    sensor->calibrated = false;
    sensor->black = false;
}

void barcode_sensor_calibrate_sample(barcode_sensor_t *sensor, uint16_t raw)
{
    uint16_t dark = barcode_sensor_darkness(raw);

    if (sensor == NULL)
    {
        return;
    }
    if (dark < sensor->white_level)
    {
        sensor->white_level = dark;
    }
    if (dark > sensor->black_level)
    {
        sensor->black_level = dark;
    }
}

bool barcode_sensor_calibrate_finish(barcode_sensor_t *sensor)
{
    if (sensor == NULL)
    {
        return false;
    }
    sensor->calibrated = update_thresholds(sensor);
    sensor->black = false;
    return sensor->calibrated;
}

bool barcode_sensor_is_black(barcode_sensor_t *sensor, uint16_t raw)
{
    uint16_t dark = barcode_sensor_darkness(raw);

    if ((sensor == NULL) || !sensor->calibrated)
    {
        return false;
    }

    if (!sensor->black && (dark > sensor->threshold_high))
    {
        sensor->black = true;
    }
    else if (sensor->black && (dark < sensor->threshold_low))
    {
        sensor->black = false;
    }

#if BARCODE_ADAPT
    /* Only learn from samples that are clearly black or clearly white, so
     * the half-covered samples at bar edges do not drag the levels inward. */
    {
        uint16_t old_black = sensor->black_level;
        uint16_t old_white = sensor->white_level;

        if (sensor->black && (dark > sensor->threshold_high))
        {
            sensor->black_level = follow(sensor->black_level, dark);
        }
        else if (!sensor->black && (dark < sensor->threshold_low))
        {
            sensor->white_level = follow(sensor->white_level, dark);
        }
        if (!update_thresholds(sensor))
        {
            /* Never adapt into a state with too little contrast. */
            sensor->black_level = old_black;
            sensor->white_level = old_white;
        }
    }
#endif
    return sensor->black;
}
