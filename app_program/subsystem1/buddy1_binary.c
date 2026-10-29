#include "buddy1_binary.h"
#include <string.h>
/* Fixed widths and checked bounds: zero bytes and endianness are unambiguous. */
static void put32(unsigned char *p,uint32_t v){for(unsigned int i=0;i<4;i++)p[i]=(unsigned char)(v>>(8*i));}
static void bits(unsigned char *p,unsigned int *at,uint32_t value,unsigned int n){
 for(unsigned int i=0;i<n;i++,(*at)++)if((value>>i)&1u)p[*at/8]|=(unsigned char)(1u<<(*at%8));
}
static int reject(b1_validation_error *e,uint32_t section,uint32_t field,uint32_t reason,
 int64_t value,int64_t minimum,int64_t maximum){
 if(e){e->section=section;e->field=field;e->reason=reason;e->value=value;e->minimum=minimum;e->maximum=maximum;}
 return 0;
}
static int valid2(const b1_s2_telemetry *s,b1_validation_error *e){
 if(s->target_speed_valid > 1u)return reject(e,2,5,2,s->target_speed_valid,0,1);
 if(s->left_speed_valid > 1u)return reject(e,2,6,2,s->left_speed_valid,0,1);
 if(s->right_speed_valid > 1u)return reject(e,2,7,2,s->right_speed_valid,0,1);
 if(s->distance_valid > 1u)return reject(e,2,8,2,s->distance_valid,0,1);
 if(s->odometry_valid > 1u)return reject(e,2,9,2,s->odometry_valid,0,1);
 if(s->motion_active > 1u)return reject(e,2,10,2,s->motion_active,0,1);
 if(s->result_pending > 1u)return reject(e,2,11,2,s->result_pending,0,1);
 if(s->sequence > 255u)return reject(e,2,12,2,s->sequence,0,255);
 if(s->system_mode > 6u)return reject(e,2,14,2,s->system_mode,0,6);
 if(s->mission_state > 10u)return reject(e,2,15,2,s->mission_state,0,10);
 if(s->terrain_state > 6u)return reject(e,2,16,2,s->terrain_state,0,6);
 if(s->route_command > 4u)return reject(e,2,17,2,s->route_command,0,4);
 if(s->route_lifecycle > 5u)return reject(e,2,18,2,s->route_lifecycle,0,5);
 if(s->motion_source > 5u)return reject(e,2,19,2,s->motion_source,0,5);
 if(s->last_motion_result > 7u)return reject(e,2,20,2,s->last_motion_result,0,7);
 if(s->communication_state > 3u)return reject(e,2,21,2,s->communication_state,0,3);
 if(s->hardware_faults > 1048575u)return reject(e,2,22,2,s->hardware_faults,0,1048575);
 if(s->stop_reasons > 511u)return reject(e,2,23,2,s->stop_reasons,0,511);
 return 1;
}
static void pack2(unsigned char *p,const b1_s2_telemetry *s){
 unsigned int at=0;
 bits(p,&at,(uint32_t)s->target_speed_mm_s,32);
 bits(p,&at,(uint32_t)s->left_speed_mm_s,32);
 bits(p,&at,(uint32_t)s->right_speed_mm_s,32);
 bits(p,&at,(uint32_t)s->distance_mm,32);
 bits(p,&at,(uint32_t)s->target_speed_valid,1);
 bits(p,&at,(uint32_t)s->left_speed_valid,1);
 bits(p,&at,(uint32_t)s->right_speed_valid,1);
 bits(p,&at,(uint32_t)s->distance_valid,1);
 bits(p,&at,(uint32_t)s->odometry_valid,1);
 bits(p,&at,(uint32_t)s->motion_active,1);
 bits(p,&at,(uint32_t)s->result_pending,1);
 bits(p,&at,(uint32_t)s->sequence,8);
 bits(p,&at,(uint32_t)s->uptime_ms,32);
 bits(p,&at,(uint32_t)s->system_mode,3);
 bits(p,&at,(uint32_t)s->mission_state,4);
 bits(p,&at,(uint32_t)s->terrain_state,3);
 bits(p,&at,(uint32_t)s->route_command,3);
 bits(p,&at,(uint32_t)s->route_lifecycle,3);
 bits(p,&at,(uint32_t)s->motion_source,3);
 bits(p,&at,(uint32_t)s->last_motion_result,3);
 bits(p,&at,(uint32_t)s->communication_state,2);
 bits(p,&at,(uint32_t)s->hardware_faults,20);
 bits(p,&at,(uint32_t)s->stop_reasons,9);
 bits(p,&at,(uint32_t)s->last_command_id,32);
 bits(p,&at,(uint32_t)s->authority_generation,32);
}
static int valid3(const b1_s3_telemetry *s,b1_validation_error *e){
 if(s->left_black > 1u)return reject(e,3,1,2,s->left_black,0,1);
 if(s->right_black > 1u)return reject(e,3,2,2,s->right_black,0,1);
 if(s->line_state > 4u)return reject(e,3,3,2,s->line_state,0,4);
 if(s->last_side > 2u)return reject(e,3,4,2,s->last_side,0,2);
 if(s->steering < (-100) || s->steering > 100)return reject(e,3,5,2,s->steering,-100,100);
 if(s->adc_valid > 1u)return reject(e,3,6,2,s->adc_valid,0,1);
 if(s->adc > 4095u)return reject(e,3,7,2,s->adc,0,4095);
 if(s->calibration_low > 4095u)return reject(e,3,8,2,s->calibration_low,0,4095);
 if(s->calibration_high > 4095u)return reject(e,3,9,2,s->calibration_high,0,4095);
 if(s->calibrated > 1u)return reject(e,3,10,2,s->calibrated,0,1);
 if(s->black > 1u)return reject(e,3,11,2,s->black,0,1);
 if(s->scan_valid > 1u)return reject(e,3,12,2,s->scan_valid,0,1);
 if(s->bar_ms > 6000u)return reject(e,3,13,2,s->bar_ms,0,6000);
 if(s->pattern_valid > 1u)return reject(e,3,14,2,s->pattern_valid,0,1);
 if(s->pattern > 511u)return reject(e,3,15,2,s->pattern,0,511);
 if(s->route > 4u)return reject(e,3,16,2,s->route,0,4);
 if(s->barcode_source > 1u)return reject(e,3,17,2,s->barcode_source,0,1);
 if(s->reversed > 1u)return reject(e,3,18,2,s->reversed,0,1);
 if(s->decoder_state > 2u)return reject(e,3,19,2,s->decoder_state,0,2);
 if(s->route_counter > 65535u)return reject(e,3,20,2,s->route_counter,0,65535);
 if(s->junction_counter > 65535u)return reject(e,3,21,2,s->junction_counter,0,65535);
 if(s->scan_counter > 65535u)return reject(e,3,22,2,s->scan_counter,0,65535);
 return 1;
}
static void pack3(unsigned char *p,const b1_s3_telemetry *s){
 unsigned int at=0;
 bits(p,&at,(uint32_t)s->left_black,1);
 bits(p,&at,(uint32_t)s->right_black,1);
 bits(p,&at,(uint32_t)s->line_state,3);
 bits(p,&at,(uint32_t)s->last_side,2);
 bits(p,&at,(uint32_t)s->steering,8);
 bits(p,&at,(uint32_t)s->adc_valid,1);
 bits(p,&at,(uint32_t)s->adc,12);
 bits(p,&at,(uint32_t)s->calibration_low,12);
 bits(p,&at,(uint32_t)s->calibration_high,12);
 bits(p,&at,(uint32_t)s->calibrated,1);
 bits(p,&at,(uint32_t)s->black,1);
 bits(p,&at,(uint32_t)s->scan_valid,1);
 bits(p,&at,(uint32_t)s->bar_ms,13);
 bits(p,&at,(uint32_t)s->pattern_valid,1);
 bits(p,&at,(uint32_t)s->pattern,9);
 bits(p,&at,(uint32_t)s->route,3);
 bits(p,&at,(uint32_t)s->barcode_source,1);
 bits(p,&at,(uint32_t)s->reversed,1);
 bits(p,&at,(uint32_t)s->decoder_state,2);
 bits(p,&at,(uint32_t)s->route_counter,16);
 bits(p,&at,(uint32_t)s->junction_counter,16);
 bits(p,&at,(uint32_t)s->scan_counter,16);
}
static int valid5(const b1_s5_telemetry *s,b1_validation_error *e){
 if(s->state > 11u)return reject(e,5,1,2,s->state,0,11);
 if(s->front_status > 4u)return reject(e,5,2,2,s->front_status,0,4);
 if(s->front_mm > 4000u)return reject(e,5,3,2,s->front_mm,0,4000);
 if(s->last_status > 4u)return reject(e,5,5,2,s->last_status,0,4);
 if(s->last_mm > 4000u)return reject(e,5,6,2,s->last_mm,0,4000);
 if(s->direction_halfdeg < (-1024) || s->direction_halfdeg > 1023)return reject(e,5,8,2,s->direction_halfdeg,-1024,1023);
 if(s->heading_deg < (-512) || s->heading_deg > 511)return reject(e,5,9,2,s->heading_deg,-512,511);
 if(s->servo_valid > 1u)return reject(e,5,10,2,s->servo_valid,0,1);
 if(s->servo_deg > 180u)return reject(e,5,11,2,s->servo_deg,0,180);
 if(s->profile_valid > 1u)return reject(e,5,13,2,s->profile_valid,0,1);
 if(s->ahead_mm > 65535u)return reject(e,5,14,2,s->ahead_mm,0,65535);
 if(s->width_mm > 65535u)return reject(e,5,15,2,s->width_mm,0,65535);
 if(s->centre_x_mm < (-65536) || s->centre_x_mm > 65535)return reject(e,5,16,2,s->centre_x_mm,-65536,65535);
 if(s->centre_y_mm < (-65536) || s->centre_y_mm > 65535)return reject(e,5,17,2,s->centre_y_mm,-65536,65535);
 if(s->left_found > 1u)return reject(e,5,18,2,s->left_found,0,1);
 if(s->left_angle_halfdeg < (-1024) || s->left_angle_halfdeg > 1023)return reject(e,5,19,2,s->left_angle_halfdeg,-1024,1023);
 if(s->left_x_mm < (-65536) || s->left_x_mm > 65535)return reject(e,5,20,2,s->left_x_mm,-65536,65535);
 if(s->left_y_mm < (-65536) || s->left_y_mm > 65535)return reject(e,5,21,2,s->left_y_mm,-65536,65535);
 if(s->right_found > 1u)return reject(e,5,22,2,s->right_found,0,1);
 if(s->right_angle_halfdeg < (-1024) || s->right_angle_halfdeg > 1023)return reject(e,5,23,2,s->right_angle_halfdeg,-1024,1023);
 if(s->right_x_mm < (-65536) || s->right_x_mm > 65535)return reject(e,5,24,2,s->right_x_mm,-65536,65535);
 if(s->right_y_mm < (-65536) || s->right_y_mm > 65535)return reject(e,5,25,2,s->right_y_mm,-65536,65535);
 if(s->decision_valid > 1u)return reject(e,5,26,2,s->decision_valid,0,1);
 if(s->side > 2u)return reject(e,5,27,2,s->side,0,2);
 if(s->left_room_mm > 4000u)return reject(e,5,28,2,s->left_room_mm,0,4000);
 if(s->right_room_mm > 4000u)return reject(e,5,29,2,s->right_room_mm,0,4000);
 if(s->plan_valid > 1u)return reject(e,5,30,2,s->plan_valid,0,1);
 if(s->turn_deg > 75u)return reject(e,5,31,2,s->turn_deg,0,75);
 if(s->diagonal_mm > 65535u)return reject(e,5,32,2,s->diagonal_mm,0,65535);
 if(s->alongside_mm > 65535u)return reject(e,5,33,2,s->alongside_mm,0,65535);
 if(s->offset_mm > 65535u)return reject(e,5,34,2,s->offset_mm,0,65535);
 if(s->leg > 7u)return reject(e,5,35,2,s->leg,0,7);
 if(s->leg_active > 1u)return reject(e,5,36,2,s->leg_active,0,1);
 if(s->rescans > 2u)return reject(e,5,37,2,s->rescans,0,2);
 if(s->reverses > 2u)return reject(e,5,38,2,s->reverses,0,2);
 if(s->reacquire_attempts > 3u)return reject(e,5,39,2,s->reacquire_attempts,0,3);
 if(s->coarse_index > 5u)return reject(e,5,40,2,s->coarse_index,0,5);
 if(s->fine_offset_deg > 10u)return reject(e,5,41,2,s->fine_offset_deg,0,10);
 if(s->turn_magnitude_deg > 70u)return reject(e,5,42,2,s->turn_magnitude_deg,0,70);
 if(s->refining > 1u)return reject(e,5,43,2,s->refining,0,1);
 if(s->failure_code > 6u)return reject(e,5,44,2,s->failure_code,0,6);
 return 1;
}
static void pack5(unsigned char *p,const b1_s5_telemetry *s){
 unsigned int at=0;
 bits(p,&at,(uint32_t)s->state,4);
 bits(p,&at,(uint32_t)s->front_status,3);
 bits(p,&at,(uint32_t)s->front_mm,12);
 bits(p,&at,(uint32_t)s->front_at_ms,32);
 bits(p,&at,(uint32_t)s->last_status,3);
 bits(p,&at,(uint32_t)s->last_mm,12);
 bits(p,&at,(uint32_t)s->last_at_ms,32);
 bits(p,&at,(uint32_t)s->direction_halfdeg,11);
 bits(p,&at,(uint32_t)s->heading_deg,10);
 bits(p,&at,(uint32_t)s->servo_valid,1);
 bits(p,&at,(uint32_t)s->servo_deg,8);
 bits(p,&at,(uint32_t)s->encounter,32);
 bits(p,&at,(uint32_t)s->profile_valid,1);
 bits(p,&at,(uint32_t)s->ahead_mm,16);
 bits(p,&at,(uint32_t)s->width_mm,16);
 bits(p,&at,(uint32_t)s->centre_x_mm,17);
 bits(p,&at,(uint32_t)s->centre_y_mm,17);
 bits(p,&at,(uint32_t)s->left_found,1);
 bits(p,&at,(uint32_t)s->left_angle_halfdeg,11);
 bits(p,&at,(uint32_t)s->left_x_mm,17);
 bits(p,&at,(uint32_t)s->left_y_mm,17);
 bits(p,&at,(uint32_t)s->right_found,1);
 bits(p,&at,(uint32_t)s->right_angle_halfdeg,11);
 bits(p,&at,(uint32_t)s->right_x_mm,17);
 bits(p,&at,(uint32_t)s->right_y_mm,17);
 bits(p,&at,(uint32_t)s->decision_valid,1);
 bits(p,&at,(uint32_t)s->side,2);
 bits(p,&at,(uint32_t)s->left_room_mm,12);
 bits(p,&at,(uint32_t)s->right_room_mm,12);
 bits(p,&at,(uint32_t)s->plan_valid,1);
 bits(p,&at,(uint32_t)s->turn_deg,7);
 bits(p,&at,(uint32_t)s->diagonal_mm,16);
 bits(p,&at,(uint32_t)s->alongside_mm,16);
 bits(p,&at,(uint32_t)s->offset_mm,16);
 bits(p,&at,(uint32_t)s->leg,3);
 bits(p,&at,(uint32_t)s->leg_active,1);
 bits(p,&at,(uint32_t)s->rescans,2);
 bits(p,&at,(uint32_t)s->reverses,2);
 bits(p,&at,(uint32_t)s->reacquire_attempts,2);
 bits(p,&at,(uint32_t)s->coarse_index,3);
 bits(p,&at,(uint32_t)s->fine_offset_deg,4);
 bits(p,&at,(uint32_t)s->turn_magnitude_deg,7);
 bits(p,&at,(uint32_t)s->refining,1);
 bits(p,&at,(uint32_t)s->failure_code,3);
 bits(p,&at,(uint32_t)s->event_seq,32);
 bits(p,&at,(uint32_t)s->dropped_events,32);
 bits(p,&at,(uint32_t)s->detections,32);
 bits(p,&at,(uint32_t)s->cleared,32);
 bits(p,&at,(uint32_t)s->bypass_left,32);
 bits(p,&at,(uint32_t)s->bypass_right,32);
 bits(p,&at,(uint32_t)s->total_rescans,32);
 bits(p,&at,(uint32_t)s->total_reverses,32);
 bits(p,&at,(uint32_t)s->emergency_stops,32);
 bits(p,&at,(uint32_t)s->line_search_ok,32);
 bits(p,&at,(uint32_t)s->line_search_retry,32);
 bits(p,&at,(uint32_t)s->line_search_fail,32);
 bits(p,&at,(uint32_t)s->motion_timeouts,32);
 bits(p,&at,(uint32_t)s->sensor_faults,32);
 bits(p,&at,(uint32_t)s->failures,32);
 bits(p,&at,(uint32_t)s->car_turns,32);
 bits(p,&at,(uint32_t)s->profiles_complete,32);
 bits(p,&at,(uint32_t)s->profiles_partial,32);
 bits(p,&at,(uint32_t)s->last_scan_ms,32);
 bits(p,&at,(uint32_t)s->last_encounter_ms,32);
 bits(p,&at,(uint32_t)s->max_encounter_ms,32);
}
int b1_binary_validate(const b1_binary_snapshot *s,b1_validation_error *e){
 if(e)memset(e,0,sizeof(*e));
 if(!s)return reject(e,1,0,1,0,0,0);
 if(s->base_valid>1)return reject(e,1,1,2,s->base_valid,0,1);
 if(s->speed_percent>100)return reject(e,1,2,2,s->speed_percent,0,100);
 if(s->distance_cm>100000)return reject(e,1,3,2,s->distance_cm,0,100000);
 if(s->fault_mask>1048575)return reject(e,1,4,2,s->fault_mask,0,1048575);
 if(s->present>15)return reject(e,1,5,2,s->present,0,15);
 if((s->present&B1_SECTION_2) && !valid2(&s->s2,e))return 0;
 if((s->present&B1_SECTION_3) && !valid3(&s->s3,e))return 0;
 if((s->present&B1_SECTION_4) && s->s4[0]!=0x21)return reject(e,4,1,3,s->s4[0],0x21,0x21);
 if((s->present&B1_SECTION_5) && !valid5(&s->s5,e))return 0;
 return 1;
}
int b1_binary_valid(const b1_binary_snapshot *s){return b1_binary_validate(s,0);}
unsigned int b1_encode_telemetry(unsigned char *out,unsigned int capacity,
 const b1_binary_snapshot *s,uint32_t seq,uint32_t uptime_ms,int demo){
 if(!out || !b1_binary_valid(s))return 0;
 unsigned int n=19;
 if(s->present&8)n+=43;
 if(s->present&1)n+=23;
 if(s->present&2)n+=23;
 if(s->present&4)n+=144;
 if(n>capacity)return 0;
 memset(out,0,n);out[0]=0xb1;out[1]=3;out[2]=1;
 out[3]=(unsigned char)((demo?1:0)|(s->base_valid<<1)|(s->present<<2));
 put32(out+4,seq);put32(out+8,uptime_ms);
 out[12]=(unsigned char)s->fault_mask;out[13]=(unsigned char)(s->fault_mask>>8);
 out[14]=(unsigned char)(s->fault_mask>>16);
 out[15]=(unsigned char)s->speed_percent;
 for(unsigned int i=0;i<3;i++)out[16+i]=(unsigned char)(s->distance_cm>>(8*i));
 unsigned int at=19;
 if(s->present&8){out[at]=2;out[at+1]=37;put32(out+at+2,s->s2_at_ms);pack2(out+at+6,&s->s2);at+=43;}
 if(s->present&1){out[at]=3;out[at+1]=17;put32(out+at+2,s->s3_at_ms);pack3(out+at+6,&s->s3);at+=23;}
 if(s->present&2){out[at]=4;out[at+1]=17;put32(out+at+2,s->s4_at_ms);memcpy(out+at+6,s->s4,17);at+=23;}
 if(s->present&4){out[at]=5;out[at+1]=138;put32(out+at+2,s->s5_at_ms);pack5(out+at+6,&s->s5);at+=144;}
 return n;
}
