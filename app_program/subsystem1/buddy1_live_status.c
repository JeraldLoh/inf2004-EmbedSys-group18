#include "buddy1_live_status.h"
#include <string.h>
static void u32(unsigned char *p,uint32_t v){for(unsigned i=0;i<4;i++)p[i]=(unsigned char)(v>>(8*i));}
int b1_live_validate(const b1_live_status *s){
 return s && s->present<=7 && s->motion.targets_valid<=1 && s->motion.qualified<=1 &&
 s->motion.searching<=1 && s->line.paused<=1 && s->line.calibration_state<=4 && s->line.calibration_reason<=1 &&
 s->line.barcode_state<=4 && s->line.barcode_character<=127 && s->line.barcode_route<=4 && s->line.barcode_digital<=1 && s->bypass.fast<=1 && s->bypass.active<=1 &&
 s->bypass.backing_off<=1 && s->bypass.segments<=7 && s->bypass.segment<=s->bypass.segments && s->bypass.side<=2;
}
unsigned b1_encode_live_status(unsigned char *p,unsigned cap,const b1_live_status *s,int demo){
 if(!p || cap<B1_LIVE_SIZE || !b1_live_validate(s) || (demo!=0 && demo!=1))return 0;
 memset(p,0,B1_LIVE_SIZE);p[0]=0xb1;p[1]=4;p[2]=4;p[3]=(unsigned char)demo;
 u32(p+4,s->uptime_ms);p[8]=(unsigned char)s->present;
 p[9]=(unsigned char)(s->motion.targets_valid|(s->motion.qualified<<1)|(s->motion.searching<<2));
 p[10]=(unsigned char)s->line.paused;p[11]=(unsigned char)(s->bypass.fast|(s->bypass.active<<1)|(s->bypass.backing_off<<2));
 u32(p+12,s->motion.at_ms);u32(p+16,(uint32_t)s->motion.left_target);u32(p+20,(uint32_t)s->motion.right_target);
 u32(p+24,s->motion.searches);u32(p+28,s->motion.search_fails);u32(p+32,s->line.at_ms);u32(p+36,s->bypass.at_ms);
 p[40]=(unsigned char)(s->bypass.segment|(s->bypass.segments<<3));p[41]=(unsigned char)s->bypass.side;
 u32(p+44,s->bypass.backoffs);u32(p+48,s->bypass.elapsed_ms);u32(p+52,s->motion.accepted);u32(p+56,s->motion.rejected);u32(p+60,s->motion.driver_errors);
 u32(p+64,s->motion.encoder_left_counts);u32(p+68,s->motion.encoder_right_counts);u32(p+72,s->motion.wheel_diameter_mm);
 p[76]=(unsigned char)s->line.calibration_state;p[77]=(unsigned char)s->line.calibration_reason;
 p[80]=(unsigned char)s->line.barcode_state;p[81]=(unsigned char)s->line.barcode_character;p[82]=(unsigned char)s->line.barcode_route;
 p[83]=(unsigned char)s->line.barcode_digital;
 u32(p+84,s->line.barcode_sequence);u32(p+88,s->line.barcode_failures);u32(p+92,s->line.barcode_event_ms);
 return B1_LIVE_SIZE;
}
