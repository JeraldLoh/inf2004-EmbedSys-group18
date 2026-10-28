#ifndef GROUP18_BARCODE_DECODER_H
#define GROUP18_BARCODE_DECODER_H

#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

#define BARCODE_DECODER_MAX_RUNS 40U

typedef enum
{
    BARCODE_DECODER_IDLE = 0,
    BARCODE_DECODER_CAPTURE,
    BARCODE_DECODER_COMPLETE
} barcode_decoder_state_t;

typedef struct
{
    barcode_decoder_state_t state;
    uint32_t run_us[BARCODE_DECODER_MAX_RUNS];
    uint32_t run_started_us;
    uint32_t capture_started_us;
    uint32_t white_started_us;
    uint8_t run_count;
    bool last_black;
    bool saw_quiet_zone;
    route_command_t command;
    char character;
    bool reversed;   /* last read was crossed in the opposite direction */
} barcode_decoder_t;

void barcode_decoder_reset(barcode_decoder_t *decoder);
void barcode_decoder_feed(
    barcode_decoder_t *decoder,
    bool centre_sensor_black,
    uint32_t now_us);
bool barcode_decoder_take(
    barcode_decoder_t *decoder,
    route_command_t *command,
    char *character);

/* True while a barcode is being captured. The controller should hold a
 * steady speed during this, because bar widths are measured in time. */
bool barcode_decoder_busy(const barcode_decoder_t *decoder);

/* True if the last decoded barcode was read backwards. Valid until the
 * next barcode_decoder_take() reset; read it before taking. */
bool barcode_decoder_was_reversed(const barcode_decoder_t *decoder);

#endif
