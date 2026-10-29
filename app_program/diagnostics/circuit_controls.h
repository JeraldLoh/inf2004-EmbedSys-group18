#ifndef GROUP18_CIRCUIT_CONTROLS_H
#define GROUP18_CIRCUIT_CONTROLS_H
#include <stdbool.h>
#include <stdint.h>
typedef struct { uint16_t effort; unsigned phase; uint32_t since; } circuit_controls_t;
/* Power selection only while STOP remains held. START must first be released,
 * then pressed >=30ms and released stably >=30ms. Never produces a run event. */
bool circuit_controls_update(circuit_controls_t *controls, uint32_t now_us,
    bool start_pressed, bool stop_pressed);
#endif
