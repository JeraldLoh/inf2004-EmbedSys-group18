/* Proposed binary telemetry v2. Wire encoding is explicit; never memcpy a C struct. */
#ifndef BUDDY1_BINARY_H
#define BUDDY1_BINARY_H
#include <stdint.h>
#define B1_BINARY_MAX 512
#define B1_SECTION_2 8u
#define B1_SECTION_3 1u
#define B1_SECTION_4 2u
#define B1_SECTION_5 4u
typedef struct {
 int32_t target_speed_mm_s;
 int32_t left_speed_mm_s;
 int32_t right_speed_mm_s;
 int32_t distance_mm;
 uint32_t target_speed_valid;
 uint32_t left_speed_valid;
 uint32_t right_speed_valid;
 uint32_t distance_valid;
 uint32_t odometry_valid;
 uint32_t motion_active;
 uint32_t result_pending;
 uint32_t sequence;
 uint32_t uptime_ms;
 uint32_t system_mode;
 uint32_t mission_state;
 uint32_t terrain_state;
 uint32_t route_command;
 uint32_t route_lifecycle;
 uint32_t motion_source;
 uint32_t last_motion_result;
 uint32_t communication_state;
 uint32_t hardware_faults;
 uint32_t stop_reasons;
 uint32_t last_command_id;
 uint32_t authority_generation;
} b1_s2_telemetry;
typedef struct {
 uint32_t left_black;
 uint32_t right_black;
 uint32_t line_state;
 uint32_t last_side;
 int32_t steering;
 uint32_t adc_valid;
 uint32_t adc;
 uint32_t calibration_low;
 uint32_t calibration_high;
 uint32_t calibrated;
 uint32_t black;
 uint32_t scan_valid;
 uint32_t bar_ms;
 uint32_t pattern_valid;
 uint32_t pattern;
 uint32_t route;
 uint32_t barcode_source;
 uint32_t reversed;
 uint32_t decoder_state;
 uint32_t route_counter;
 uint32_t junction_counter;
 uint32_t scan_counter;
} b1_s3_telemetry;
typedef struct {
 uint32_t state;
 uint32_t front_status;
 uint32_t front_mm;
 uint32_t front_at_ms;
 uint32_t last_status;
 uint32_t last_mm;
 uint32_t last_at_ms;
 int32_t direction_halfdeg;
 int32_t heading_deg;
 uint32_t servo_valid;
 uint32_t servo_deg;
 uint32_t encounter;
 uint32_t profile_valid;
 uint32_t ahead_mm;
 uint32_t width_mm;
 int32_t centre_x_mm;
 int32_t centre_y_mm;
 uint32_t left_found;
 int32_t left_angle_halfdeg;
 int32_t left_x_mm;
 int32_t left_y_mm;
 uint32_t right_found;
 int32_t right_angle_halfdeg;
 int32_t right_x_mm;
 int32_t right_y_mm;
 uint32_t decision_valid;
 uint32_t side;
 uint32_t left_room_mm;
 uint32_t right_room_mm;
 uint32_t plan_valid;
 uint32_t turn_deg;
 uint32_t diagonal_mm;
 uint32_t alongside_mm;
 uint32_t offset_mm;
 uint32_t leg;
 uint32_t leg_active;
 uint32_t rescans;
 uint32_t reverses;
 uint32_t reacquire_attempts;
 uint32_t coarse_index;
 uint32_t fine_offset_deg;
 uint32_t turn_magnitude_deg;
 uint32_t refining;
 uint32_t failure_code;
 uint32_t event_seq;
 uint32_t dropped_events;
 uint32_t detections;
 uint32_t cleared;
 uint32_t bypass_left;
 uint32_t bypass_right;
 uint32_t total_rescans;
 uint32_t total_reverses;
 uint32_t emergency_stops;
 uint32_t line_search_ok;
 uint32_t line_search_retry;
 uint32_t line_search_fail;
 uint32_t motion_timeouts;
 uint32_t sensor_faults;
 uint32_t failures;
 uint32_t car_turns;
 uint32_t profiles_complete;
 uint32_t profiles_partial;
 uint32_t last_scan_ms;
 uint32_t last_encounter_ms;
 uint32_t max_encounter_ms;
} b1_s5_telemetry;
typedef struct {
 uint32_t base_valid, speed_percent, distance_cm, fault_mask, present;
 uint32_t s2_at_ms, s3_at_ms, s4_at_ms, s5_at_ms;
 b1_s2_telemetry s2;
 b1_s3_telemetry s3;
 unsigned char s4[17]; /* original Buddy 4 frame, including 0x21 header */
 b1_s5_telemetry s5;
} b1_binary_snapshot;
typedef struct {
 uint32_t section, field, reason; /* 1-based field index; reason 1 missing, 2 range, 3 format */
 int64_t value, minimum, maximum;
} b1_validation_error;
int b1_binary_validate(const b1_binary_snapshot *s,b1_validation_error *error);
int b1_binary_valid(const b1_binary_snapshot *s);
unsigned int b1_encode_telemetry(unsigned char *out, unsigned int capacity,
 const b1_binary_snapshot *s, uint32_t seq, uint32_t uptime_ms, int demo);
#endif
