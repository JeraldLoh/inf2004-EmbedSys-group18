#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem1/buddy1_live_status.h"
int main(int argc,char **argv){
 b1_live_status s={0};unsigned char out[B1_LIVE_SIZE];
 s.present=7;s.uptime_ms=1200;
 s.motion=(b1_live_motion){.at_ms=1000,.left_target=-40,.right_target=80,.targets_valid=1,.qualified=1,.searching=1,.searches=4,.search_fails=1,.accepted=10,.rejected=2,.driver_errors=0,.encoder_left_counts=1271,.encoder_right_counts=1264,.wheel_diameter_mm=65};
 s.line=(b1_live_line){.at_ms=1100,.paused=1};
 s.bypass=(b1_live_bypass){.at_ms=1150,.fast=1,.active=1,.segment=2,.segments=5,.side=1,.backoffs=1,.elapsed_ms=1200};
 assert(b1_encode_live_status(out,sizeof(out),&s,0)==76);
 assert(out[0]==0xb1 && out[1]==2 && out[2]==4 && out[8]==7);
 assert(out[9]==7 && out[10]==1 && out[11]==3 && out[40]==42);
 assert(b1_encode_live_status(out,75,&s,0)==0);
 s.bypass.segment=6;assert(!b1_encode_live_status(out,sizeof(out),&s,0));s.bypass.segment=2;
 s.line.paused=2;assert(!b1_encode_live_status(out,sizeof(out),&s,0));s.line.paused=1;
 s.present=8;assert(!b1_encode_live_status(out,sizeof(out),&s,0));s.present=7;
 assert(b1_encode_live_status(out,sizeof(out),&s,0)==76);
 if(argc>1){FILE *f=fopen(argv[1],"w");assert(f);for(unsigned i=0;i<sizeof(out);++i)fprintf(f,"%02x",out[i]);fputc(10,f);assert(!fclose(f));}
 puts("PASS live status signed targets, flags, timestamps, progress bounds and explicit length");return 0;
}
