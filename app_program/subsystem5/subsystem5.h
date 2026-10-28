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
 * Motion and line sensing come in through the optional bindings below.
 * Without them (B5_RUN) car turns, moves and line searches are refused
 * rather than faked: a refused move fails the encounter at once and the
 * controller holds a latched safe stop. With them (CAR_RUN) every move is a
 * Buddy 2 goal stamped with the controller's obstacle authority.
 */
#include "../app.h"
#include "../project_config/gpio_dispatcher.h"
#include "../vehicle_controller/controller_service.h"
#include "b5_motion.h"

typedef struct
{
    gpio_dispatcher_t *dispatcher;  /* shared GPIO interrupt owner (ECHO) */
    bool (*urgent_stop)(void);      /* stop the motors now; false if not applied */
    bool (*post_event)(const controller_event_t *event); /* never blocks */
    /* Optional (may be NULL): controller status for the 1 s status line. */
    bool (*controller_status)(controller_status_t *status);
    /* Optional, all or nothing: controller gate, Buddy 2 executor and
     * Buddy 3 line sensing (see b5_motion.h). */
    b5_motion_ops_t motion;
} subsystem5_bindings_t;

/* Startup only, before subsystem5_init(). dispatcher, urgent_stop and
 * post_event are required; motion is all-or-nothing. */
ER subsystem5_bind(const subsystem5_bindings_t *bindings);

#endif
