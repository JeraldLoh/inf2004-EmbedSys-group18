#ifndef CONTROL_MESSAGES_H
#define CONTROL_MESSAGES_H

#include "system_types.h"

typedef struct
{
    command_id_t command_id;
    authority_generation_t authority_generation;
    motion_source_t source;
    int32_t speed_mm_s;
    int32_t curvature_u1_m;
    uint32_t speed_cap_mm_s;
    system_time_ms_t created_ms;
    system_time_ms_t expires_ms;
} motion_continuous_request_t;

typedef struct
{
    goal_id_t goal_id;
    authority_generation_t authority_generation;
    motion_source_t source;
    motion_goal_type_t type;
    int32_t target_mm;
    int32_t target_mdeg;
    uint32_t max_speed_mm_s;
    uint32_t timeout_ms;
} motion_goal_request_t;

typedef struct
{
    goal_id_t goal_id;
    authority_generation_t authority_generation;
    motion_result_code_t code;
    int32_t travelled_mm;
    int32_t turned_mdeg;
    bool odometry_valid;
} motion_goal_result_t;

typedef struct
{
    command_id_t command_id;
    route_command_t command;
    char decoded_character;
    system_time_ms_t decoded_ms;
    system_time_ms_t expires_ms;
    bool valid;
} route_instruction_t;

typedef struct
{
    bool candidate;
    bool confirmed;
    bool left_available;
    bool straight_available;
    bool right_available;
    uint8_t confidence_percent;
    system_time_ms_t observed_ms;
} junction_observation_t;

typedef struct
{
    terrain_state_t state;
    uint32_t speed_cap_mm_s;
    int32_t estimated_height_mm;
    uint32_t uncertainty_mm;
    bool estimate_valid;
    system_time_ms_t observed_ms;
} terrain_report_t;

#endif
