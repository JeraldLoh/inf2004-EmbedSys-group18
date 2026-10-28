#ifndef GROUP18_LINE_DRIVE_H
#define GROUP18_LINE_DRIVE_H

/* Buddy 3 BENCH ONLY: turns subsystem 3's steering and junction events into
 * left/right wheel effort for the line-follow bench (APP_S3_BENCH == 2).
 * This is NOT the vehicle controller and NOT Buddy 2's executor; it exists to
 * test line following on the car before those are written.
 *
 * Junction rule (bench):
 *   1. junction event        -> both wheels 0 for junction_wait_ms
 *   2. a barcode route was read before it:
 *        LEFT  -> left wheel 0, right wheel base for turn_ms (swing turn)
 *        RIGHT -> right wheel 0, left wheel base for turn_ms
 *        STRAIGHT -> as below
 *        UTURN -> not supported on the bench: the run ends (DONE)
 *      no barcode -> STRAIGHT: both wheels base for cross_ms, sensors
 *        ignored, so the car drives off the crossbar
 *   3. back to line following
 *
 * Efforts are signed per-mille like Buddy 2's drv_motor_apply(), but this
 * module only ever outputs 0..max_effort: never negative, so Buddy 2's
 * driver never sees a direct forward/reverse flip.
 *
 * Portable: no tk_* calls, no hardware.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

typedef struct
{
    int16_t base_effort;         /* straight-line effort, per-mille */
    int16_t max_effort;          /* cap for either wheel */
    int16_t gain;                /* effort per steering unit (-100..100) */
    uint32_t junction_wait_ms;   /* stop time at a junction */
    uint32_t cross_ms;           /* blind straight drive off the crossbar */
    uint32_t turn_ms;            /* blind swing turn for a barcode turn */
} line_drive_config_t;

typedef enum
{
    LINE_DRIVE_FOLLOW = 0,
    LINE_DRIVE_JUNCTION_WAIT,
    LINE_DRIVE_CROSS,
    LINE_DRIVE_TURN_LEFT,
    LINE_DRIVE_TURN_RIGHT,
    LINE_DRIVE_DONE
} line_drive_state_t;

typedef enum
{
    LINE_DRIVE_EVENT_NONE = 0,
    LINE_DRIVE_EVENT_JUNCTION_STOP,   /* stopped at a junction */
    LINE_DRIVE_EVENT_GO,              /* wait over; "go_command" says where */
    LINE_DRIVE_EVENT_RESUME,          /* back to line following */
    LINE_DRIVE_EVENT_DONE             /* run ended (U-turn requested) */
} line_drive_event_t;

typedef struct
{
    line_drive_config_t config;
    line_drive_state_t state;
    uint32_t since_ms;
    bool announced;                   /* JUNCTION_STOP already reported */
    route_command_t pending_route;    /* from the last barcode, or NONE */
    char pending_character;
    route_command_t go_command;       /* what the last GO event chose */
    char go_character;                /* barcode letter, or '-' = default */
} line_drive_t;

/* Sensible bench defaults: 20% base, 35% cap. Override in the build with
 * -DS3_BENCH_BASE_EFFORT=... etc. (see line_drive.c). */
void line_drive_default_config(line_drive_config_t *config);
void line_drive_init(line_drive_t *drive, const line_drive_config_t *config);

/* left = base + gain*steer, right = base - gain*steer, each clamped to
 * 0..max. steer > 0 means "turn right", so the left wheel speeds up. */
void line_drive_mix(int8_t steer, int16_t base, int16_t gain, int16_t max,
    int16_t *left, int16_t *right);

/* A barcode was decoded: remember it for the next junction (newest wins). */
void line_drive_route(line_drive_t *drive, route_command_t command, char character);

/* Subsystem 3 reported a junction. Ignored unless following the line. */
void line_drive_junction(line_drive_t *drive, uint32_t now_ms);

/* Call every control cycle. Writes the efforts to apply now. */
line_drive_event_t line_drive_step(line_drive_t *drive, int8_t steer,
    uint32_t now_ms, int16_t *left, int16_t *right);

#endif
