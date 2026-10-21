/*
 * Change Requirement 1 evidence: storage size of each Buddy 4 data item in
 * the conventional "before" representation (one natural C type per field,
 * floats for angles, bools/enums per flag, text MQTT) versus the packed
 * "after" representation actually used in app_program/subsystem4.
 *
 * Not part of the firmware (outside app_program, never linked). Compile it
 * for the real target ABI and read the symbol sizes:
 *
 *   arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O2 -c \
 *       testing/subsystem4/s4_size_report.c -o s4_size_report.o
 *   arm-none-eabi-nm -S --size-sort s4_size_report.o
 *
 * Each symbol's size (second column, hex) is the RAM/payload cost of that
 * item. Results are recorded in app_program/subsystem4/DATA_REPRESENTATION.md.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../../app_program/subsystem4/s4_packed.h"
#include "../../app_program/subsystem4/terrain_estimator.h"
#include "../../app_program/contracts/control_messages.h"

/* ---- 1. Shared snapshot (IMU task -> controller, Buddy 1, ISRs) -------- */
typedef struct
{
    terrain_state_t terrain;
    uint32_t speed_cap_mm_s;
    int32_t motion_class;
    int32_t longitudinal;
    bool fault_i2c, fault_mag, fault_stale, fault_uncal;
    bool fault_saturated, fault_tilt, fault_collision;
    bool calibrated, mag_calibrated, distance_from_odometry, heading_valid;
    uint32_t collision_count;
    int32_t collision_dir;
    int32_t turn_state;
    float pitch_deg, roll_deg, heading_deg;
    uint32_t sample_seq;
    float turn_rate_dps;
    int32_t live_height_mm;
    uint32_t shock_mg;
    uint32_t vibration_mg;
    int32_t hump_height_mm;
    uint32_t hump_uncertainty_mm;
    int32_t hump_peak_pitch_deg;
    uint32_t hump_length_mm;
    bool hump_valid;
    uint32_t hump_seq;
} before_snapshot_t;

before_snapshot_t before_snapshot;
uint32_t after_snapshot[S4_WORDS];

/* ---- 2. Inter-task event message (IMU task -> controller queue) -------- */
typedef struct
{
    int32_t type;
    terrain_state_t state;
    uint32_t speed_cap_mm_s;
    int32_t height_mm;
    uint32_t uncertainty_mm;
    int32_t peak_pitch_deg;
    uint32_t length_mm;
    bool valid;
    int32_t collision_dir;
    uint32_t magnitude_mg;
    int32_t turn_angle_deg;
    uint32_t fault_mask;
} before_event_t;

before_event_t before_event;
s4_event_msg_t after_event;

/* ---- 3. Odometry input (Buddy 2 -> Buddy 4) ---------------------------- */
typedef struct
{
    int32_t cumulative_mm;
    uint32_t post_seq;
    bool valid;
} before_odometry_t;

before_odometry_t before_odometry;
uint32_t after_odometry;

/* ---- 4. Hump result held for reporting --------------------------------- */
terrain_report_t before_hump_report;   /* existing shared contract */
uint32_t after_hump_word;

/* ---- 5. Estimator state flags (task-private) --------------------------- */
typedef struct
{
    terrain_state_t terrain;
    terrain_state_t pending;
    uint8_t confirm;
    int32_t motion;
    int32_t longitudinal;
    bool fault_i2c, fault_mag, fault_stale, fault_uncal;
    bool fault_saturated, fault_tilt, fault_collision;
    bool calibrated, cal_active;
    uint8_t cal_retries;
    bool mag_cal_active, mag_calibrated, heading_valid;
    bool filters_primed, mag_primed;
    int32_t turn_state;
    uint8_t turn_quiet;
    bool in_episode, saw_descent, used_fallback, dist_odometry;
    uint32_t collision_count;
    int32_t collision_dir;
    uint32_t hump_seq;
    uint32_t sample_seq;
    uint32_t cal_count;
} before_flags_t;

typedef struct
{
    unsigned int terrain : 3;
    unsigned int pending : 3;
    unsigned int confirm : 4;
    unsigned int motion : 2;
    unsigned int longitudinal : 2;
    unsigned int faults : 7;
    unsigned int calibrated : 1;
    unsigned int cal_active : 1;
    unsigned int cal_retries : 3;
    unsigned int mag_cal_active : 1;
    unsigned int mag_calibrated : 1;
    unsigned int heading_valid : 1;
    unsigned int filters_primed : 1;
    unsigned int mag_primed : 1;
    unsigned int turn_state : 2;
    unsigned int turn_quiet : 4;
    unsigned int in_episode : 1;
    unsigned int saw_descent : 1;
    unsigned int used_fallback : 1;
    unsigned int dist_odometry : 1;
    unsigned int collision_count : 3;
    unsigned int collision_dir : 2;
    unsigned int hump_seq : 3;
    unsigned int sample_seq : 5;
    unsigned int cal_count : 8;
} after_flags_t;

before_flags_t before_flags;
after_flags_t after_flags;

/* ---- 6. Configuration (all 32-bit vs sized/bit-field) ------------------ */
typedef struct
{
    uint32_t values[33];   /* the 33 tunables of s4_cfg_t, one word each */
} before_cfg_t;

before_cfg_t before_cfg;
s4_cfg_t after_cfg;

/* ---- 7. Whole estimator: flags + config expanded vs as built ------------ */
s4_est_t after_estimator;

/* ---- 8. MQTT payloads: text vs binary (sizes include one NUL for text) -- */
const char before_mqtt_telemetry[] =
    "{\"terrain\":\"ASCENDING\",\"cap_mm_s\":150,\"motion\":\"MOVING\","
    "\"long\":\"STEADY\",\"faults\":0,\"cal\":1,\"magcal\":1,\"odom\":1,"
    "\"coll_cnt\":0,\"coll_dir\":\"FRONT\",\"turn\":\"NONE\",\"hdg_valid\":1,"
    "\"pitch\":12.5,\"roll\":-0.5,\"heading\":273,\"seq\":17,\"turn_dps\":0,"
    "\"live_h_mm\":21,\"shock_mg\":0,\"vib_mg\":24,\"hump_mm\":39,\"unc_mm\":7,"
    "\"peak_pitch\":17,\"len_cm\":44,\"valid\":1,\"hump_seq\":1}";
uint8_t after_mqtt_telemetry[S4_TLM_SIZE];

const char before_mqtt_command[] = "{\"cmd\":\"recalibrate\"}";
uint8_t after_mqtt_command[1];
