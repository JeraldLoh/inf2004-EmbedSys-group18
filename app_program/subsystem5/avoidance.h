#ifndef BUDDY5_AVOIDANCE_H
#define BUDDY5_AVOIDANCE_H

// Obstacle detection, profiling, avoidance and line-recovery state machine.
//
// Hardware-independent: every sensor/actuator access goes through
// av_platform_t, so the same code runs on the Pico and in host unit tests.
// Non-blocking by design: call av_tick() from the main loop; each call does a
// bounded amount of work (at most one range measurement).

#include <stdbool.h>
#include <stdint.h>

#include "obstacle.h"
#include "range.h"

#define AV_MAX_COARSE_POINTS 9

typedef enum {
    AV_STATE_MONITOR,          // watching straight ahead; Buddy 5 is passive
    AV_STATE_COARSE_SCAN,      // wide sweep at a few fixed angles
    AV_STATE_FINE_SCAN_RIGHT,  // small steps outward from center to find the right edge
    AV_STATE_FINE_SCAN_LEFT,   // same, left
    AV_STATE_DECIDE,           // build profile and choose a side
    AV_STATE_WAIT_RESCAN,      // no side clear; pause then scan again
    AV_STATE_REVERSE,          // still boxed in; back up then scan again
    AV_STATE_BYPASS,           // driving around the obstacle
    AV_STATE_REACQUIRE,        // searching for the line behind the obstacle
    AV_STATE_FAILED,           // stopped; needs av_reset()
    AV_STATE_COUNT
} av_state_t;

typedef enum {
    AV_EVT_OBSTACLE_DETECTED,
    AV_EVT_SCAN_POINT,
    AV_EVT_OBSTACLE_CLEARED, // obstacle gone when rescanned; no bypass needed
    AV_EVT_PROFILE_READY,
    AV_EVT_DECISION,
    AV_EVT_RESCAN,
    AV_EVT_REVERSING,
    AV_EVT_BYPASS_START,
    AV_EVT_EMERGENCY_STOP, // something appeared ahead mid-manoeuvre
    AV_EVT_REACQUIRE_START,
    AV_EVT_LINE_REACQUIRED,
    AV_EVT_FAILED,
} av_event_type_t;

typedef struct {
    av_event_type_t type;
    uint32_t time_ms;
    scan_point_t point;                // DETECTED, SCAN_POINT, EMERGENCY_STOP
    side_t side;                       // DECISION, BYPASS_START, REACQUIRE_START
    int attempt;                       // RESCAN, REVERSING, REACQUIRE_START
    const obstacle_profile_t *profile; // PROFILE_READY, DECISION
    const side_choice_t *choice;       // DECISION
    uint32_t duration_ms;              // LINE_REACQUIRED: detection -> back on line
    const char *reason;                // FAILED, OBSTACLE_CLEARED
} av_event_t;

// Integration surface. Everything except on_event is required.
typedef struct {
    void *ctx;
    uint32_t (*now_ms)(void *ctx);
    void (*servo_set)(void *ctx, int angle_deg);
    // One filtered range reading at the current servo angle.
    range_reading_t (*measure)(void *ctx);

    // Buddy 2 (motion). Commands return immediately; motion_done() reports
    // when the last reverse/bypass/search command has finished.
    void (*motion_stop)(void *ctx);
    void (*motion_reverse)(void *ctx, float cm);
    void (*motion_bypass)(void *ctx, side_t side, const obstacle_profile_t *profile);
    void (*motion_search_line)(void *ctx, side_t toward);
    void (*motion_resume_line_follow)(void *ctx);
    bool (*motion_done)(void *ctx);

    // Buddy 3 (line sensors).
    bool (*line_detected)(void *ctx);

    // Optional: telemetry / logging hook (Buddy 1).
    void (*on_event)(void *ctx, const av_event_t *event);
} av_platform_t;

typedef struct {
    // Servo angles. center must be exactly halfway between left and right.
    int left_deg;
    int center_deg;
    int right_deg;
    int coarse_points; // odd, 3..AV_MAX_COARSE_POINTS, evenly dividing left..right
    int fine_step_deg;
    int fine_half_range_deg;

    float detect_cm;           // obstacle closer than this triggers avoidance
    float clear_hysteresis_cm; // on rescan, center must exceed detect+this to count as gone
    int detect_confirm_count;  // consecutive detections required (debounce)
    float edge_margin_cm;      // reading must exceed obstacle distance by this to be an edge
    float min_clearance_cm;    // lateral room needed on the chosen side
    float max_range_cm;        // value used for NO_ECHO readings
    float emergency_stop_cm;   // collision guard during bypass/reacquire; 0 disables

    uint32_t servo_settle_base_ms;
    float servo_ms_per_deg;

    int max_rescans;  // per reverse cycle
    uint32_t rescan_delay_ms;
    int max_reverses; // per encounter
    float reverse_cm;

    uint32_t motion_timeout_ms;
    uint32_t reacquire_timeout_ms; // per search attempt
    int max_reacquire_attempts;
    int max_consecutive_faults;
} av_config_t;

typedef struct {
    uint32_t detections;
    uint32_t obstacle_cleared;
    uint32_t profiles_complete; // both edges found
    uint32_t profiles_partial;  // at least one edge outside scan window
    uint32_t bypass_left;
    uint32_t bypass_right;
    uint32_t rescans;
    uint32_t reverses;
    uint32_t emergency_stops;
    uint32_t reacquire_ok;
    uint32_t reacquire_retries;
    uint32_t reacquire_fail;
    uint32_t motion_timeouts;
    uint32_t sensor_faults;
    uint32_t failures;
    uint32_t last_scan_ms;      // detection -> decision
    uint32_t last_encounter_ms; // detection -> back on line
    uint32_t max_encounter_ms;
    uint64_t total_encounter_ms;
} av_metrics_t;

// Treat as opaque; defined here only so callers can allocate it statically.
typedef struct {
    av_config_t cfg;
    av_platform_t hw;
    scan_geometry_t geom;
    av_state_t state;

    int servo_deg;
    bool servo_known;
    bool step_pending;
    int pending_angle;
    uint32_t settle_until;

    int confirm_count;
    int fault_streak;
    range_reading_t last_reading;
    bool have_reading;

    scan_point_t coarse[AV_MAX_COARSE_POINTS];
    int coarse_idx;
    float ahead_cm;
    int fine_offset;
    scan_point_t last_hit;
    obstacle_edge_t right_edge;

    obstacle_profile_t profile;
    bool have_profile;
    side_choice_t choice;

    bool off_line;
    int rescans;
    int reverses;
    int reacquire_attempts;
    uint32_t deadline;
    uint32_t t_detect;
    const char *fail_reason;

    av_metrics_t metrics;
} av_t;

void av_config_default(av_config_t *cfg);
// Returns NULL if valid, otherwise a description of the first problem.
const char *av_config_check(const av_config_t *cfg);

// Returns false (and leaves av unusable) if cfg or platform is invalid.
bool av_init(av_t *av, const av_config_t *cfg, const av_platform_t *hw);
void av_tick(av_t *av);
// Back to MONITOR from any state. Motion stays stopped; the caller decides
// when to resume. Metrics are kept.
void av_reset(av_t *av);

av_state_t av_state(const av_t *av);
const char *av_state_name(av_state_t s);
// True while Buddy 5 owns the motors (anything other than MONITOR).
bool av_is_active(const av_t *av);
bool av_last_reading(const av_t *av, range_reading_t *out);
const obstacle_profile_t *av_profile(const av_t *av); // NULL until first profile
const av_metrics_t *av_metrics(const av_t *av);
const char *av_fail_reason(const av_t *av);

#endif
