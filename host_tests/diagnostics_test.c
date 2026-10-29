#include <assert.h>
#include <stdio.h>
#include "../app_program/subsystem1/buddy1_binary.h"
#include "../app_program/subsystem1/buddy1_diagnostics.h"
int main(int argc,char **argv){
 b1_binary_snapshot s={0};b1_validation_error e={0};
 s.present=B1_SECTION_3;s.s3.steering=-101;
 assert(!b1_binary_validate(&s,&e));assert(e.section==3 && e.field==5 && e.reason==2);
 assert(e.value==-101 && e.minimum==-100 && e.maximum==100);
 s.s3.steering=0;s.present=B1_SECTION_5;s.s5.last_status=5;
 assert(!b1_binary_validate(&s,&e));assert(e.section==5 && e.value==5);
 b1_diagnostics d={0};d.uptime_ms=1000;d.rejected=3;d.out_of_range=2;d.busy=1;d.last_at_ms=900;
 s.present=B1_SECTION_3;s.s3.steering=-101;assert(!b1_binary_validate(&s,&d.last));
 unsigned char wire[52];assert(b1_encode_diagnostics(wire,sizeof(wire),&d,0)==52);
 assert(!b1_encode_diagnostics(wire,51,&d,0));
 if(argc==2){FILE*f=fopen(argv[1],"wb");assert(f);assert(fwrite(wire,1,52,f)==52);fclose(f);}
 puts("PASS structured validation errors and diagnostic wire");return 0;
}
