#include <assert.h>
#include <string.h>
#include "buddy1_protocol.h"
int main(void) {
 b1_command c;
 assert(b1_parse("START",5,&c) && c.kind==B1_START);
 assert(b1_parse("STOP",4,&c) && c.kind==B1_STOP);
 assert(b1_parse("SET_SPEED 0",11,&c) && c.speed==0);
 assert(b1_parse("SET_SPEED 100",13,&c) && c.speed==100);
 const char *bad[]={"", "START x", "SET_SPEED -1", "SET_SPEED 101", "SET_SPEED 1x", "SET_SPEED ", "SET_SPEED 00000000000000000000000000001"};
 for(unsigned i=0;i<sizeof(bad)/sizeof(bad[0]);i++) assert(!b1_parse(bad[i],strlen(bad[i]),&c));
 assert(!b1_parse("START\0junk",10,&c));
 assert(!b1_parse(0,1,&c));
 return 0;
}
