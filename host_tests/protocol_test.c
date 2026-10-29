#include <assert.h>
#include <string.h>
#include "buddy1_protocol.h"
int main(void){
 b1_command c;unsigned char b[8]={0xb1,2,3,1,0x78,0x56,0x34,0x12};
 assert(b1_parse((char*)b,8,&c)&&c.kind==B1_START&&c.command_id==0x12345678u);
 b[3]=2;assert(b1_parse((char*)b,8,&c)&&c.kind==B1_STOP);
 for(unsigned int n=0;n<8;n++)assert(!b1_parse((char*)b,n,&c));
 b[3]=3;assert(!b1_parse((char*)b,8,&c));b[3]=1;
 b[1]=9;assert(!b1_parse((char*)b,8,&c));b[1]=2;memset(b+4,0,4);assert(!b1_parse((char*)b,8,&c));
 assert(!b1_parse("START",5,&c));assert(!b1_parse("SET_SPEED 100",13,&c));assert(!b1_parse(0,8,&c));return 0;
}
