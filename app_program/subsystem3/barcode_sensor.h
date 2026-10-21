#ifndef GROUP18_BARCODE_SENSOR_H
#define GROUP18_BARCODE_SENSOR_H

/* Buddy 3: analogue barcode sensor -> black/white (hardware independent).
 *
 * The ADC gives 0..4095. Black tape reflects less IR than white foam, so on
 * the usual TCRT5000 module the analogue output is HIGHER on black. Set
 * BARCODE_BLACK_IS_HIGH to 0 if your module measures the other way.
 * Internally every reading is turned into "darkness" (always higher = more
 * black), so the rest of the code does not care which way the module reads.
 *
 * Three ideas here:
 *  1. Calibration: measure the real black and white levels by sweeping the
 *     sensor over a barcode, instead of hard-coding a threshold.
 *  2. Hysteresis: two thresholds. The output only flips to black above the
 *     HIGH one and back to white below the LOW one, so noise around the
 *     middle cannot make it chatter (which would create fake narrow bars).
 *  3. Adaptation: during the run, the black and white levels slowly follow
 *     the readings (1/64 of the difference per sample), so the thresholds
 *     track gradual lighting changes across the room.
 */
#include <stdbool.h>
#include <stdint.h>

#ifndef BARCODE_BLACK_IS_HIGH
#define BARCODE_BLACK_IS_HIGH 1
#endif

/* Minimum black/white difference (ADC counts) to trust the levels. */
#ifndef BARCODE_MIN_CONTRAST
#define BARCODE_MIN_CONTRAST 300U
#endif

/* Hysteresis half-band as a percentage of the black-white span. */
#ifndef BARCODE_HYSTERESIS_PERCENT
#define BARCODE_HYSTERESIS_PERCENT 10U
#endif

/* Adaptation speed: level moves 1/2^SHIFT of the error per sample.
 * Set BARCODE_ADAPT to 0 to turn adaptation off. */
#ifndef BARCODE_ADAPT
#define BARCODE_ADAPT 1
#endif
#ifndef BARCODE_ADAPT_SHIFT
#define BARCODE_ADAPT_SHIFT 6U
#endif

#define BARCODE_ADC_MAX 4095U

typedef struct
{
    uint16_t white_level;     /* darkness of white foam */
    uint16_t black_level;     /* darkness of black tape */
    uint16_t threshold_low;   /* darkness below this -> white */
    uint16_t threshold_high;  /* darkness above this -> black */
    bool calibrated;
    bool black;
} barcode_sensor_t;

void barcode_sensor_reset(barcode_sensor_t *sensor);

/* Feed raw samples during a calibration sweep (over both black and white). */
void barcode_sensor_calibrate_sample(barcode_sensor_t *sensor, uint16_t raw);

/* Compute thresholds from the sweep. Returns false if contrast is too low
 * (sensor too high, wrong surface, or not swept over a bar). */
bool barcode_sensor_calibrate_finish(barcode_sensor_t *sensor);

/* Classify one raw sample. Returns true when the sensor is over black.
 * Always returns false (white) until calibrated. */
bool barcode_sensor_is_black(barcode_sensor_t *sensor, uint16_t raw);

/* Convert a raw ADC value to darkness (for debug printing). */
uint16_t barcode_sensor_darkness(uint16_t raw);

#endif
