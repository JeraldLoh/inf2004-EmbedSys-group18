#ifndef GROUP18_SUBSYSTEM3_H
#define GROUP18_SUBSYSTEM3_H

/* Buddy 3: Line, barcode and junction perception.
 * The shared lifecycle declarations are in app.h.
 * DRAFT: priority, stack size and period still need team agreement.
 */
#include "../app.h"
#include "../contracts/control_messages.h"
#include "line_follow.h"

/* Diagnostics: loop counter and last step of the Buddy 3 task (see subsystem3.c). */
IMPORT void subsystem3_debug(uint32_t *loops, uint32_t *stage);

/* Latest line status (1 packed byte, safe to read from any task/core). */
IMPORT line_status_t subsystem3_line_status(void);

/* Steering proposal for Buddy 2: -100 (full left) .. 0 .. +100 (full right).
 * It grows the longer the car stays off centre and briefly counter-steers
 * after a correction to stop zig-zagging. If analogue is supplied, return
 * whether this same steering sample came from the analogue PID. */
IMPORT int8_t subsystem3_steer(bool *analogue);

/* Take the newest decoded barcode / junction. Each returns true once per event. */
IMPORT BOOL subsystem3_take_route(route_instruction_t *route);
IMPORT BOOL subsystem3_take_junction(junction_observation_t *junction);

/* No-barcode junction probe (see subsystem3.c). The controller:
 *   1. sees subsystem3_probe_requested() == TRUE
 *   2. stops just past the crossbar, calls subsystem3_probe_begin()
 *   3. pivots ~10 deg left and back (Buddy 2 goal), calls subsystem3_probe_end()
 *   4. then takes the default route and the confirmed junction as normal. */
IMPORT BOOL subsystem3_probe_requested(void);
IMPORT ER subsystem3_probe_begin(void);
IMPORT ER subsystem3_probe_end(void);

/* Search for the line, e.g. at the end of Buddy 5's obstacle bypass. Pass the
 * side the line should be on, or LINE_SIDE_NONE to use where it was last
 * seen. Done when subsystem3_line_status().state is no longer REACQUIRE. */
IMPORT void subsystem3_reacquire(line_side_t side_hint);

/* Barcodes use fixed thresholds from power-on. LINE_PID=1 builds call
 * subsystem3_calibrate_barcode() at startup: sweep the sensors over black and
 * white for 3 s (barcode and line PID). Line following carries on meanwhile.
 * In those builds barcodes are ignored until the sweep succeeds. */
IMPORT void subsystem3_calibrate_barcode(void);
IMPORT BOOL subsystem3_barcode_calibrated(void);

#endif
