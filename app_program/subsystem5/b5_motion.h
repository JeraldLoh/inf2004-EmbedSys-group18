#ifndef GROUP18_B5_MOTION_H
#define GROUP18_B5_MOTION_H

/* Buddy 5 moves as Buddy 2 goals (CAR_RUN).
 *
 * Turns the avoidance logic's move commands (turn N deg, forward/reverse N
 * cm, search for the line) into Buddy 2 goals and continuous requests, each
 * stamped with the controller's obstacle authority (IF-04 gate), and reports
 * done/failed back.
 *
 * A move first waits for obstacle authority, up to wait_ms: the avoidance
 * logic asks for its first turn in the same tick that it reports the
 * obstacle, before the controller has entered the obstacle mission. A
 * rejected submission is retried within that window for the same reason.
 *
 * Portable: no kernel calls, host-tested. subsystem5.c calls it from the
 * Buddy 5 task.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/control_messages.h"

/* Optional as a group: all set, or all NULL (no moves in this build). */
typedef struct
{
    bool (*authority)(motion_source_t source, authority_generation_t *generation);
    bool (*goal)(const motion_goal_request_t *goal);
    bool (*continuous)(const motion_continuous_request_t *request);
    bool (*goal_result)(goal_id_t goal_id, motion_goal_result_t *result);
    void (*halt)(void);   /* stop now without latching the driver inhibit */
    bool (*line_seen)(void);                  /* Buddy 3: on the line, not searching */
    void (*line_reacquire)(bool toward_left); /* Buddy 3: start a line search */
} b5_motion_ops_t;

typedef enum
{
    B5_MOVE_NONE = 0,   /* nothing asked */
    B5_MOVE_PENDING,    /* waiting for obstacle authority */
    B5_MOVE_GOAL,       /* Buddy 2 goal running */
    B5_MOVE_SEARCH,     /* arcing toward the line */
    B5_MOVE_DONE,
    B5_MOVE_FAILED
} b5_move_state_t;

typedef struct
{
    uint32_t goal_speed_mm_s;
    uint32_t goal_timeout_ms;
    uint32_t wait_ms;              /* for obstacle authority */
    int32_t search_speed_mm_s;
    int32_t search_curvature_u1_m; /* toward the line side */
    uint32_t search_life_ms;       /* refreshed every service call */
} b5_motion_config_t;

typedef struct
{
    b5_motion_ops_t ops;
    b5_motion_config_t config;
    b5_move_state_t state;
    motion_goal_request_t goal;
    uint32_t deadline_ms;
    goal_id_t next_id;
    bool search_left;
    motion_result_code_t last_result;   /* of the last finished goal */
} b5_motion_t;

/* All seven set, or all NULL. */
bool b5_motion_ops_valid(const b5_motion_ops_t *ops);
bool b5_motion_ops_present(const b5_motion_ops_t *ops);

void b5_motion_default_config(b5_motion_config_t *config);
void b5_motion_init(b5_motion_t *motion, const b5_motion_ops_t *ops,
    const b5_motion_config_t *config);

/* deg > 0 = left (CCW). 0 completes at once. */
void b5_motion_turn(b5_motion_t *motion, int32_t mdeg, uint32_t now_ms);
/* mm > 0; reverse = true drives backwards. 0 completes at once. */
void b5_motion_straight(b5_motion_t *motion, int32_t mm, bool reverse,
    uint32_t now_ms);
void b5_motion_search(b5_motion_t *motion, bool toward_left);
void b5_motion_stop(b5_motion_t *motion);   /* halt; nothing outstanding */
void b5_motion_resume(b5_motion_t *motion); /* controller takes over */

/* Every Buddy 5 tick, before av_tick(). */
void b5_motion_service(b5_motion_t *motion, uint32_t now_ms);

bool b5_motion_done(const b5_motion_t *motion);
bool b5_motion_failed(const b5_motion_t *motion);
bool b5_motion_line_seen(const b5_motion_t *motion);

#endif
