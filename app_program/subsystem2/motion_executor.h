#ifndef GROUP18_MOTION_EXECUTOR_H
#define GROUP18_MOTION_EXECUTOR_H

/* DRAFT for Buddy 2's review (written by Buddy 5 as integration lead).
 *
 * Portable core of the Buddy 2 motion executor (IF-05 in, IF-06 out): turns
 * one continuous request or one finite goal into left/right effort every
 * control period, using the encoder counts to end goals. No kernel calls and
 * no hardware: subsystem2_motion.c is the kernel task that feeds it encoder
 * snapshots and writes its output through drv_motor_apply().
 *
 * Rules:
 * - One command at a time. The newest accepted command replaces the old one;
 *   a replaced goal ends CANCELLED.
 * - A command whose authority generation is older than the current one is
 *   rejected (stale). After motion_executor_abort() (driver refused, STOP
 *   pressed) nothing with the same or an older generation is accepted, so
 *   motion only resumes with fresh authority (IF-07).
 * - motion_executor_halt() stops now. It also rejects later LINE requests of
 *   the same generation, so a line-tracking request that was already in
 *   flight cannot restart the car after an obstacle stop.
 * - Continuous requests drive open loop (feedforward) and stop by themselves
 *   when expires_ms passes. They never reverse a wheel.
 * - Goals end when every wheel has turned its target counts (COMPLETED) or
 *   when timeout_ms passes (TIMEOUT).
 * - A wheel that changes direction gets one zero-effort period first, as
 *   drv_motor_apply() requires.
 *
 * The speed/effort numbers in the default config are placeholders until
 * measured; see subsystem2/README.md. Track width is measured (112 mm) but
 * not yet checked with a pivot on the floor.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/control_messages.h"
#include "motion_math.h"

/* Distance between the left and right wheel contact centres. Measured on
 * the car with a ruler, 2026-10-28; still to check with a 360 deg pivot. */
#ifndef MOTION_TRACK_WIDTH_MM
#define MOTION_TRACK_WIDTH_MM 112u
#endif

typedef struct
{
    motion_calibration_t calibration;  /* wheelbase_mm = track width */
    int16_t effort_per_100_mm_s;       /* open-loop feedforward, per-mille */
    int16_t goal_min_effort;           /* goals never ask for less (stiction) */
    int16_t max_effort;                /* cap for either wheel */
    int16_t straight_gain_per_mm;      /* goal correction per mm L/R mismatch */
    int16_t straight_max;              /* cap for that correction */
    uint32_t default_goal_speed_mm_s;  /* when a goal gives 0 */
    uint32_t max_speed_mm_s;           /* requests above this are rejected */
    uint32_t default_goal_timeout_ms;  /* when a goal gives 0 */
    uint32_t slow_zone_permille;       /* half speed after this much progress */
} motion_executor_config_t;

typedef enum
{
    MOTION_EXECUTOR_IDLE = 0,
    MOTION_EXECUTOR_CONTINUOUS,
    MOTION_EXECUTOR_GOAL
} motion_executor_mode_t;

typedef struct
{
    int16_t left;                        /* effort to apply now */
    int16_t right;
    authority_generation_t generation;   /* authority of the current command */
    bool active;                         /* a command owns the motors */
} motion_executor_output_t;

typedef struct
{
    motion_executor_config_t config;
    motion_executor_mode_t mode;
    authority_generation_t generation;   /* of the current/last command */
    bool generation_seen;
    authority_generation_t floor;        /* abort: reject this and older */
    bool floor_set;
    authority_generation_t line_floor;   /* halt: reject LINE at or below */
    bool line_floor_set;

    motion_continuous_request_t request;

    motion_goal_request_t goal;
    uint32_t goal_started_ms;
    uint32_t start_left;
    uint32_t start_right;
    uint32_t target_left;                /* counts */
    uint32_t target_right;

    motion_goal_result_t result;         /* latest finished goal */
    bool result_valid;

    int16_t last_left;                   /* last output, for the deadtime */
    int16_t last_right;

    /* Measured wheel speed, for calibration and telemetry. */
    uint32_t speed_left_count;
    uint32_t speed_right_count;
    uint32_t speed_ms;
    bool speed_started;
    int32_t left_mm_s;
    int32_t right_mm_s;

    uint32_t accepted;
    uint32_t rejected;
} motion_executor_t;

void motion_executor_default_config(motion_executor_config_t *config);
bool motion_executor_init(motion_executor_t *executor,
    const motion_executor_config_t *config);

/* Both return false (and count a rejection) for an invalid or stale request. */
bool motion_executor_continuous(motion_executor_t *executor,
    const motion_continuous_request_t *request, uint32_t now_ms);
bool motion_executor_goal(motion_executor_t *executor,
    const motion_goal_request_t *goal, uint32_t now_ms,
    uint32_t left_count, uint32_t right_count);

/* Stop now; an active goal ends CANCELLED. See the LINE rule above. */
void motion_executor_halt(motion_executor_t *executor);
/* The driver refused or a STOP fired: end the command with code (ABORTED or
 * FAULT) and reject its generation and older from now on. */
void motion_executor_abort(motion_executor_t *executor,
    motion_result_code_t code);

/* Once per control period with the latest encoder counts. */
motion_executor_output_t motion_executor_step(motion_executor_t *executor,
    uint32_t now_ms, uint32_t left_count, uint32_t right_count);

/* Latest state of goal_id: PROGRESS while it runs, then its final result.
 * False if goal_id is neither running nor the latest finished goal. */
bool motion_executor_goal_result(const motion_executor_t *executor,
    goal_id_t goal_id, motion_goal_result_t *result);

#endif
