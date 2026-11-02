#ifndef GROUP18_MEASURED_MOTION_EXECUTOR_H
#define GROUP18_MEASURED_MOTION_EXECUTOR_H
#include "wheel_measurement.h"
#include "wheel_pid.h"
#include "../contracts/control_messages.h"

typedef struct
{
    wheel_measurement_config_t measurement;
    wheel_pid_config_t pid[2];
    uint32_t period_us, watchdog_us;
    uint32_t track_width_mm, max_speed_mm_s, acceleration_um_s2;
    uint32_t reversal_us, settle_us, settle_speed_um_s;
    uint32_t tolerance_um, overshoot_um;
    uint32_t progress_timeout_us, startup_grace_us, goal_margin_ms;
    uint32_t max_goal_mm;
    /* Optional calibrated rolling floor for forward/reverse final approach.
     * Zero preserves the original profile; never exceeds the request cap. */
    uint32_t linear_goal_min_speed_mm_s;
    uint32_t pivot_goal_min_speed_mm_s; /* Independent turn candidate, zero by default. */
    /* Effort cap during pivot goals only, if above the PID cap; 0 = PID cap.
     * Pivots scrub both wheels sideways and need more than straight driving. */
    uint16_t pivot_max_effort;
    /* Straight goals: once one wheel reaches its distance, the other counts
     * as done within this much, so it never drives on alone (that swings the
     * car round the stopped wheel and stalls it). 0 = each wheel exact. */
    uint32_t linear_sync_slack_um;
    /* The same for pivot goals, capped at half the wheel distance. 0 = off. */
    uint32_t pivot_sync_slack_um;
    /* Set only after real direction/load/STOP/speed/pivot qualification.
     * Test fixtures setting true are NOT physical qualification evidence. */
    bool qualified;
} measured_motion_executor_config_t;

typedef struct
{
    void *context;
    bool (*stop_pending)(void *context);
    bool (*arm_watchdog)(void *context, uint32_t duration_us);
    void (*cancel_watchdog)(void *context);
} measured_motion_executor_io_t;

typedef enum { MOTION_EXEC_DISABLED = 0, MOTION_EXEC_IDLE,
    MOTION_EXEC_CONTINUOUS, MOTION_EXEC_GOAL } measured_motion_executor_state_t;

typedef enum {
    MOTION_DETAIL_NONE = 0, MOTION_DETAIL_ENCODER_CAPTURE,
    MOTION_DETAIL_MEASUREMENT_QUALITY, MOTION_DETAIL_AUTHORITY_DEADLINE,
    MOTION_DETAIL_DRIVER_INHIBIT, MOTION_DETAIL_COMMAND_EXPIRED,
    MOTION_DETAIL_GOAL_TIMEOUT, MOTION_DETAIL_GOAL_ADMISSION,
    MOTION_DETAIL_ZERO_OUTPUT_IO, MOTION_DETAIL_OVERSHOOT,
    MOTION_DETAIL_PID_INPUT, MOTION_DETAIL_NO_PROGRESS,
    MOTION_DETAIL_WATCHDOG_ARM, MOTION_DETAIL_OUTPUT_IO
} motion_detail_reason_t;
const char *measured_motion_detail_name(motion_detail_reason_t reason);
typedef struct
{
    measured_motion_executor_state_t state;
    uint32_t generation;
    uint32_t command_id;
    int32_t target_um_s[2];
    wheel_measurement_sample_t wheels;
    motion_result_code_t last_stop;
    bool authorized;
    bool terminal_pending;
    uint8_t low_speed_floor_mask; /* Trial requested floor: bit0=L, bit1=R. */
    uint32_t detail_sequence, detail_generation, detail_command;
    motion_detail_reason_t detail_reason;
    motion_result_code_t detail_result;
    uint8_t detail_wheel_mask;
    int32_t detail_target_um_s[2];
    uint32_t detail_speed_um_s[2];
    bool detail_speed_valid;
} measured_motion_executor_status_t;

/* Single task/state owner, or externally serialized. All normal PWM commits
 * occur in step(). Driver STOP remains independently ISR-safe. */
typedef struct
{
    drv_motor_t *motor;
    drv_encoder_t *encoder;
    measured_motion_executor_config_t config;
    measured_motion_executor_io_t io;
    wheel_measurement_t measurement;
    wheel_pid_t pid[2];
    measured_motion_executor_status_t status;
    motion_continuous_request_t continuous;
    motion_goal_request_t goal;
    motion_goal_result_t terminal;
    uint64_t goal_base_um[2];
    uint64_t goal_target_um;
    uint64_t progress_um[2];
    uint32_t progress_us[2];
    uint32_t ramp_um_s[2];
    uint32_t ramp_remainder[2];
    int8_t last_sign[2];
    uint32_t permit_ticket, last_command_id, last_goal_id;
    uint32_t last_step_us, transition_until_us, started_us;
    uint32_t admission_until_us, continuous_until_us, goal_until_us, settled_since_us;
    bool primed, transition, goal_started, settling;
    bool calibration_authorized;
    uint32_t calibration_until_us;
} measured_motion_executor_t;

/* No tuned default. Caller supplies measured calibration/control configuration. */
bool measured_motion_executor_init(measured_motion_executor_t *executor, drv_motor_t *motor,
    drv_encoder_t *encoder, const measured_motion_executor_config_t *config,
    const measured_motion_executor_io_t *io);
motion_status_t measured_motion_executor_authorize(measured_motion_executor_t *executor, uint32_t generation);
#if (defined(APP_B2_MOVE_TEST) && APP_B2_MOVE_TEST) || (defined(APP_CAR_B2_TRIAL) && APP_CAR_B2_TRIAL)
/* Movement-test or explicit integrated-trial build ONLY. Explicit bounded calibration authority,
 * not production qualification. Hard limits: <=120mm/s, <=30% effort,
 * pivot goals <=45%, <=1m goals, watchdog <=100ms and total run <=20s, or no run limit with
 * MEASURED_MOTION_NO_RUN_LIMIT. No config flags changed. */
#define MEASURED_MOTION_NO_RUN_LIMIT UINT32_MAX
motion_status_t measured_motion_executor_authorize_calibration(measured_motion_executor_t *executor,
    uint32_t generation, uint32_t now_us, uint32_t max_run_us);
#endif
/* Continuous speed is nonnegative; reverse is a finite explicit goal. Curvature
 * is signed whole reciprocal metres: positive turns right. Zero cap=no extra
 * cap. Controller supplies TTL in the raw-us domain after validating its ms
 * envelope. These are not new SET_SPEED/profile wire opcodes. */
motion_status_t measured_motion_executor_continuous(measured_motion_executor_t *executor,
    const motion_continuous_request_t *request, uint32_t now_us, uint32_t ttl_us);
motion_status_t measured_motion_executor_goal(measured_motion_executor_t *executor,
    const motion_goal_request_t *request, uint32_t now_us, uint32_t admission_ttl_us);
motion_status_t measured_motion_executor_step(measured_motion_executor_t *executor);
void measured_motion_executor_cancel(measured_motion_executor_t *executor, motion_result_code_t code);
/* Serialized controller manoeuvre pause: zero outputs, cancel any goal, keep
 * current authority only if zero output succeeds. Not an urgent/global STOP;
 * the caller must reject in-flight continuous work before using this API. */
void measured_motion_executor_pause(measured_motion_executor_t *executor);
bool measured_motion_executor_result(const measured_motion_executor_t *executor, motion_goal_result_t *result);
bool measured_motion_executor_ack(measured_motion_executor_t *executor, goal_id_t id, authority_generation_t generation);
#endif
