#ifndef BUDDY1_LIVE_STATUS_H
#define BUDDY1_LIVE_STATUS_H
#include <stdint.h>
#define B1_LIVE_SIZE 96U
/* Owner-produced supplementary status. No direct motor/sensor access here. */
typedef struct { uint32_t at_ms; int32_t left_target,right_target; uint32_t targets_valid,qualified,searching,searches,search_fails,accepted,rejected,driver_errors,encoder_left_counts,encoder_right_counts,wheel_diameter_mm; } b1_live_motion;
/* Barcode state: 0 waiting, 1 scanning, 2 decoded, 3 unreadable, 4 unmapped.
 * Result and sequence persist so dashboard polling cannot miss a short read. */
typedef struct { uint32_t at_ms,paused,calibration_state,calibration_reason;
 uint32_t barcode_state,barcode_character,barcode_route,barcode_sequence,barcode_failures,barcode_event_ms,barcode_digital;
} b1_live_line;
typedef struct { uint32_t at_ms,fast,active,backing_off,segment,segments,side,backoffs,elapsed_ms; } b1_live_bypass;
typedef struct { uint32_t present,uptime_ms; b1_live_motion motion; b1_live_line line; b1_live_bypass bypass; } b1_live_status;
int b1_live_validate(const b1_live_status *s);
unsigned b1_encode_live_status(unsigned char *out,unsigned cap,const b1_live_status *s,int demo);
#endif
