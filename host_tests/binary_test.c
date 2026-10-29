#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../app_program/subsystem1/buddy1_binary.h"
int main(int argc, char **argv) {
 b1_binary_snapshot s={0}; unsigned char wire[B1_BINARY_MAX];
 s.base_valid=1;s.speed_percent=42;s.distance_cm=93;s.present=15;
 s.s2_at_ms=880;s.s2.target_speed_mm_s=30;s.s2.left_speed_mm_s=-10;s.s2.right_speed_mm_s=20;s.s2.distance_mm=-500;
 s.s2.left_speed_valid=1;s.s2.right_speed_valid=1;s.s2.sequence=255;s.fault_mask=0x80000;
 s.s3_at_ms=900;s.s3.line_state=2;s.s3.steering=-100;s.s3.adc=4095;
 s.s3.route=4;s.s3.calibration_high=4095;
 s.s4_at_ms=950;s.s4[0]=0x21;s.s4[4]=0x80;
 s.s5_at_ms=980;s.s5.state=9;s.s5.front_status=0;s.s5.front_mm=930;
 s.s5.front_at_ms=970;s.s5.last_status=4;s.s5.direction_halfdeg=-160;
 s.s5.heading_deg=-150;s.s5.event_seq=0xffffffffu;s.s5.dropped_events=2;
 unsigned int n=b1_encode_telemetry(wire,sizeof(wire),&s,7,1000,1);
 assert(n>18 && wire[0]==0xb1 && wire[1]==3 && wire[2]==1);
 assert(wire[3]==63 && wire[15]==42 && wire[16]==93 && wire[17]==0);
 assert(b1_encode_telemetry(wire,n-1,&s,7,1000,1)==0);
 s.s3.steering=-101;assert(b1_encode_telemetry(wire,sizeof(wire),&s,7,1000,1)==0);
 s.s3.steering=-100;s.s5.last_status=5;assert(b1_encode_telemetry(wire,sizeof(wire),&s,7,1000,1)==0);
 s.s5.last_status=4;s.present=16;assert(!b1_binary_valid(&s));s.present=15;
 n=b1_encode_telemetry(wire,sizeof(wire),&s,7,1000,1);assert(n);
 if(argc==2){FILE *f=fopen(argv[1],"wb");assert(f);assert(fwrite(wire,1,n,f)==n);fclose(f);}
 puts("PASS binary telemetry boundaries and golden fixture");return 0;
}
