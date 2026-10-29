#ifndef CONTROLLER_SERVICE_H
#define CONTROLLER_SERVICE_H

/* Portable core of the controller task: owns the vehicle controller, applies
 * one event at a time and carries out the resulting action through output
 * hooks. No kernel calls, so it is unit tested on a PC; controller_task.c is
 * the thin kernel wrapper (queue + task) around it.
 *
 * STOP_MOTION runs the stop_motion output. With the optional drive and
 * read_line outputs, controller_service_tick() also carries out line
 * tracking: while AUTO is following the line it turns Buddy 3's steering into
 * short-lived continuous requests for Buddy 2 (IF-02 in, IF-05 out). The
 * other actions are recorded in last_action only.
 *
 * With the junction outputs as well (controller_service_bind_junction()), the
 * tick also drives junctions:
 *   - Buddy 3's routes (barcodes) and junctions go to the vehicle controller.
 *     The junction is taken before the route: Buddy 3 publishes the route
 *     first, so a junction taken here always finds its route ready.
 *   - No barcode: Buddy 3 reports an unconfirmed junction (JUNCTION_APPROACH)
 *     and asks for its probe. The car settles, probe_begin(), pivots
 *     probe_mdeg left and back as Buddy 2 goals, probe_end(); Buddy 3 then
 *     publishes the default route and the confirmed junction.
 *   - EXECUTE_ROUTE: drive approach_mm forward (wheel axle over the junction
 *     centre), then pivot 90 deg (LEFT/RIGHT) or 180 deg (UTURN); STRAIGHT
 *     only drives forward. The last goal's result goes to the controller,
 *     which resumes line following (VERIFY_LINE).
 *   - Junctions are ignored for holdoff_ms after a turn (the sensors sweep
 *     over lines while pivoting), and dropped outside AUTO or during an
 *     obstacle mission.
 *   - A goal that times out or faults latches MOTION_FAULT; one that cannot
 *     run (rejected, cancelled, aborted) latches ROUTE_REQUIRED, which a new
 *     START clears.
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

/* Buddy 3's line state, read once per tick. */
typedef struct
{
    int8_t steer;          /* -100 (full left) .. +100 (full right) */
    bool barcode_active;   /* a barcode is being read: hold a steady speed */
} controller_line_input_t;

typedef struct
{
    void *context;
    /* Required. Must stop the motors now; false if the stop was not applied. */
    bool (*stop_motion)(void *context);
    /* Optional pair (both or neither): line tracking. drive hands Buddy 2 a
     * continuous request and returns false if it was rejected; read_line
     * fills in Buddy 3's latest steering. */
    bool (*drive)(void *context, const motion_continuous_request_t *request);
    bool (*read_line)(void *context, controller_line_input_t *line);
} controller_outputs_t;

/* Optional junction outputs; all required when bound. Buddy 3 events in,
 * Buddy 2 goals out. */
typedef struct
{
    void *context;
    /* Each returns true once per new event (Buddy 3's take functions). */
    bool (*take_route)(void *context, route_instruction_t *route);
    bool (*take_junction)(void *context, junction_observation_t *junction);
    /* Buddy 3's no-barcode probe handshake. */
    bool (*probe_requested)(void *context);
    bool (*probe_begin)(void *context);
    bool (*probe_end)(void *context);
    /* Buddy 2: false if the goal was rejected. goal_result gives PROGRESS
     * while goal_id runs, then its final result. */
    bool (*goal)(void *context, const motion_goal_request_t *goal);
    bool (*goal_result)(void *context, goal_id_t goal_id,
        motion_goal_result_t *result);
} controller_junction_outputs_t;

/* Junction numbers. Placeholders until measured on the car. */
typedef struct
{
    int32_t approach_mm;       /* line sensors to wheel axle; must be > 0 */
    int32_t turn_mdeg;         /* LEFT / RIGHT pivot */
    int32_t uturn_mdeg;
    int32_t probe_mdeg;        /* Buddy 3 probe: pivot left this far and back */
    uint32_t goal_speed_mm_s;
    uint32_t goal_timeout_ms;
    uint32_t settle_ms;        /* stop before probing; wait for its result */
    uint32_t holdoff_ms;       /* ignore junctions this long after a turn */
} controller_junction_config_t;

typedef enum
{
    CONTROLLER_JUNCTION_IDLE = 0,
    CONTROLLER_JUNCTION_PROBE_SETTLE,  /* line requests expire, car stops */
    CONTROLLER_JUNCTION_PROBE_OUT,     /* pivot left */
    CONTROLLER_JUNCTION_PROBE_BACK,    /* pivot right */
    CONTROLLER_JUNCTION_PROBE_RESULT,  /* waiting for Buddy 3's verdict */
    CONTROLLER_JUNCTION_APPROACH,      /* forward to the junction centre */
    CONTROLLER_JUNCTION_TURN
} controller_junction_phase_t;

/* Line-tracking numbers. Placeholders until measured on the track. */
typedef struct
{
    uint32_t cruise_mm_s;               /* normal line following */
    uint32_t slow_mm_s;                 /* barcode read / junction approach */
    int32_t curvature_per_steer_u1_m;   /* steer 100 -> this * 100 */
    uint32_t request_life_ms;           /* Buddy 2 stops if not refreshed */
} controller_line_config_t;

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
    authority_generation_t authority_generation;
    uint32_t drive_requests;    /* line-tracking requests accepted by Buddy 2 */
    uint32_t drive_rejected;
    int8_t last_steer;
    controller_junction_phase_t junction_phase;
    uint32_t junctions_seen;      /* passed to the vehicle controller */
    uint32_t junctions_ignored;   /* during the holdoff after a turn */
    uint32_t routes_accepted;
    uint32_t routes_rejected;
    uint32_t junctions_done;      /* routes driven to the end */
    route_command_t last_route;
} controller_status_t;

typedef struct
{
    vehicle_controller_t controller;
    controller_outputs_t outputs;
    controller_line_config_t line;
    controller_status_t status;
    command_id_t next_command_id;
    bool initialized;

    controller_junction_outputs_t junction_out;
    controller_junction_config_t junction;
    bool junction_bound;
    controller_junction_phase_t junction_phase;
    system_time_ms_t phase_since_ms;
    route_command_t junction_route;
    motion_goal_request_t junction_goal;
    goal_id_t next_goal_id;
    bool holdoff;
    system_time_ms_t holdoff_until_ms;
} controller_service_t;

void controller_line_default_config(controller_line_config_t *config);
void controller_junction_default_config(controller_junction_config_t *config);

bool controller_service_init(
    controller_service_t *service,
    const controller_outputs_t *outputs);
/* After init, before the first tick. Needs the drive/read_line outputs and
 * every junction output; false otherwise or if already bound. */
bool controller_service_bind_junction(
    controller_service_t *service,
    const controller_junction_outputs_t *outputs);
/* Returns true if the event changed the controller, false if it was invalid
 * or not allowed now (counted in events_rejected). */
bool controller_service_handle(
    controller_service_t *service,
    const controller_event_t *event);
void controller_service_status(
    const controller_service_t *service,
    controller_status_t *status);

/* Call every control period (10 ms). Runs the junction steps when bound (see
 * the top of this file), then sends one line-tracking request while AUTO is
 * following the line (TRACK_LINE, VERIFY_LINE, JUNCTION_APPROACH), no
 * junction step is running and line motion is allowed; otherwise sends
 * nothing, so Buddy 2 stops when the last request expires. No-op without the
 * drive/read_line outputs. */
void controller_service_tick(
    controller_service_t *service,
    system_time_ms_t now_ms);

/* IF-04 gate: may source drive now? True only in AUTO with motion allowed for
 * that source; then *generation is the authority to put on the request. */
bool controller_service_authority(
    const controller_service_t *service,
    motion_source_t source,
    authority_generation_t *generation);

void controller_service_motion_result(controller_service_t *service,
    const motion_goal_result_t *result);

#endif
