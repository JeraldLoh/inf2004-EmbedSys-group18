#include "buddy1_protocol.h"
int b1_parse(const char *data,unsigned int n,b1_command *out){
 if(!data||!out||n!=8)return 0;
 const unsigned char *p=(const unsigned char*)data;
 if(p[0]!=0xb1||p[1]!=2||p[2]!=3||(p[3]!=B1_START&&p[3]!=B1_STOP))return 0;
 uint32_t id=0;for(unsigned int i=0;i<4;i++)id|=(uint32_t)p[4+i]<<(8*i);
 if(!id)return 0;
 out->kind=p[3];out->command_id=id;return 1;
}
