#ifndef GROUP18_DRV_IR_H
#define GROUP18_DRV_IR_H

/* Buddy 3: IR sensor hardware access (target only, uses micro T-Kernel).
 *
 * Pins follow the agreed team map. They are defined here, not in pinmap.h,
 * until the team updates the shared pin map.
 *   GP0         right line sensor, digital D0
 *   GP7         left line sensor, digital D0
 *   GP26/ADC0   barcode sensor, analogue A0
 */
#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#define S3_PIN_LINE_RIGHT 0U
#define S3_PIN_LINE_LEFT 7U
#define S3_PIN_BARCODE_ADC 26U
#define S3_BARCODE_ADC_CHANNEL 0   /* GP26 = ADC input 0 */

/* GPIO level the line module's D0 outputs when it is over BLACK.
 * Most TCRT5000/LM393 boards give HIGH on black (no reflection) and light
 * their LED on white. Check yours with drv_ir_debug_print() and flip if needed. */
#ifndef S3_LINE_BLACK_LEVEL
#define S3_LINE_BLACK_LEVEL 1U
#endif

/* OPTIONAL digital barcode input (module D0), used only when
 * S3_BARCODE_USE_EDGES = 1. No pin is assigned in the team map yet:
 * pick a free GPIO and set it here. */
#ifndef S3_PIN_BARCODE_D0
#define S3_PIN_BARCODE_D0 0xFFU   /* 0xFF = not wired */
#endif
#ifndef S3_BARCODE_D0_BLACK_LEVEL
#define S3_BARCODE_D0_BLACK_LEVEL 1U
#endif

ER drv_ir_init(void);
void drv_ir_read_line(bool *left_black, bool *right_black);
ER drv_ir_read_barcode(uint16_t *raw);   /* 12-bit, 0..4095 */
bool drv_ir_read_barcode_digital(void);  /* D0 pin, true = black */
uint32_t drv_ir_now_us(void);            /* free-running 1 MHz timer */

#endif
