#ifndef B5_MISSION_BRIDGE_H
#define B5_MISSION_BRIDGE_H

/* Turns Buddy 5's avoidance state into vehicle controller events, so the
 * controller's obstacle mission follows what Buddy 5 is doing:
 *
 *   obstacle confirmed            -> OBSTACLE_DETECTED (mission OBSTACLE_STOP)
 *   COARSE/FINE/TURN_SCAN, WAIT_RESCAN, REVERSE -> progress SCAN
 *   DECIDE                        -> progress PLAN_BYPASS
 *   BYPASS                        -> progress BYPASS
 *   REACQUIRE                     -> progress RECOVERY
 *   back to MONITOR               -> OBSTACLE_CLEARED (rejoined if from RECOVERY)
 *   FAILED                        -> OBSTACLE_FAILED (SENSOR_CRITICAL or
 *                                    RECOVERY_FAILED)
 *
 * Portable: no kernel calls. Call b5_bridge_update() after every av_tick()
 * and post the returned events in order.
 */
#include <stdbool.h>
#include <stddef.h>

#include "avoidance.h"
#include "../vehicle_controller/controller_service.h"

#define B5_BRIDGE_MAX_EVENTS 4U

typedef struct
{
    bool active;            /* an obstacle mission is in progress */
    bool failure_reported;  /* FAILED already sent; quiet until back to MONITOR */
    mission_state_t mission;
} b5_bridge_t;

void b5_bridge_init(b5_bridge_t *bridge);

/* sensor_fault: the failure was the ultrasonic sensor (only read when state
 * is FAILED). Returns how many events were written to out. */
size_t b5_bridge_update(
    b5_bridge_t *bridge,
    av_state_t state,
    bool sensor_fault,
    controller_event_t out[B5_BRIDGE_MAX_EVENTS]);

#endif
