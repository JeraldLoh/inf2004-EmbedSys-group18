#ifndef GROUP18_SUBSYSTEM5_H
#define GROUP18_SUBSYSTEM5_H

/* Buddy 5: Obstacle scanning and avoidance (kernel task).
 *
 * The task runs the avoidance state machine (avoidance.c) every 10 ms with
 * the HC-SR04 on GP16/GP17 and the scan servo on GP12, and reports every
 * state change to the vehicle controller as an obstacle mission event
 * (b5_mission_bridge.c).
 *
 * Lifecycle: subsystem5_bind() once at startup, then subsystem5_init() with
 * every other init, then subsystem5_start() after all inits succeeded.
 *
 * Not available yet, so refused rather than faked: car turns, forward /
 * reverse moves and line searches (no Buddy 2 goal executor) and line
 * detection (Buddy 3 not wired). A refused move fails the encounter at once;
 * the controller then holds a latched safe stop.
 */
#include "../app.h"
#include "../project_config/gpio_dispatcher.h"
#include "../vehicle_controller/controller_service.h"

typedef struct
{
    gpio_dispatcher_t *dispatcher;  /* shared GPIO interrupt owner (ECHO) */
    bool (*urgent_stop)(void);      /* stop the motors now; false if not applied */
    bool (*post_event)(const controller_event_t *event); /* never blocks */
} subsystem5_bindings_t;

/* Startup only, before subsystem5_init(). All three fields are required. */
ER subsystem5_bind(const subsystem5_bindings_t *bindings);

#endif
