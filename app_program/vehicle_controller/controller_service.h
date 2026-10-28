#ifndef CONTROLLER_SERVICE_H
#define CONTROLLER_SERVICE_H

/* Portable core of the controller task: owns the vehicle controller, applies
 * one event at a time and carries out the resulting action through output
 * hooks. No kernel calls, so it is unit tested on a PC; controller_task.c is
 * the thin kernel wrapper (queue + task) around it.
 *
 * Only STOP_MOTION has an executor today (Buddy 2's urgent stop). Other
 * actions are recorded in last_action for whoever executes them later.
 */
#include "vehicle_controller.h"

typedef enum
{
    CONTROLLER_EVENT_SET_MODE = 0,
    CONTROLLER_EVENT_LATCH_STOP,
    CONTROLLER_EVENT_CLEAR_STOP,
    CONTROLLER_EVENT_OBSTACLE_DETECTED,
    CONTROLLER_EVENT_OBSTACLE_PROGRESS,
    CONTROLLER_EVENT_OBSTACLE_CLEARED,
    CONTROLLER_EVENT_OBSTACLE_FAILED,
    CONTROLLER_EVENT_COUNT
} controller_event_type_t;

typedef struct
{
    controller_event_type_t type;
    system_mode_t mode;          /* SET_MODE */
    stop_reason_mask_t reasons;  /* LATCH_STOP, CLEAR_STOP, OBSTACLE_FAILED */
    mission_state_t mission;     /* OBSTACLE_PROGRESS */
    bool rejoined_line;          /* OBSTACLE_CLEARED */
} controller_event_t;

typedef struct
{
    void *context;
    /* Required. Must stop the motors now; false if the stop was not applied. */
    bool (*stop_motion)(void *context);
} controller_outputs_t;

typedef struct
{
    system_mode_t mode;
    mission_state_t mission;
    stop_reason_mask_t stop_reasons;
    hardware_fault_mask_t hardware_faults;
    controller_action_type_t last_action;
    uint32_t events_applied;
    uint32_t events_rejected;   /* not valid in the current mission/mode */
    uint32_t stop_commands;
    uint32_t stop_failures;
} controller_status_t;

typedef struct
{
    vehicle_controller_t controller;
    controller_outputs_t outputs;
    controller_status_t status;
    bool initialized;
} controller_service_t;

bool controller_service_init(
    controller_service_t *service,
    const controller_outputs_t *outputs);
/* Returns true if the event changed the controller, false if it was invalid
 * or not allowed now (counted in events_rejected). */
bool controller_service_handle(
    controller_service_t *service,
    const controller_event_t *event);
void controller_service_status(
    const controller_service_t *service,
    controller_status_t *status);

#endif
