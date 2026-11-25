#include "buddy1_protocol.h"

static void put_u32(uint8_t *p,uint32_t value){
 for(unsigned i=0;i<4U;i++)p[i]=(uint8_t)(value>>(8U*i));
}

static int valid_map(const uint8_t map[B1_MAP_BYTES]){
 if(map[B1_MAP_BYTES-1]&0xf0u)return 0;
 for(unsigned i=0;i<B1_MAP_ENTRIES;i++)if(b1_map_entry(map,i)>B1_MAP_MAX_VALUE)return 0;
 return 1;
}
static int header(const unsigned char *p,uint32_t *id){
 if(p[0]!=0xb1||p[1]!=2||p[2]!=3)return 0;
 *id=0;for(unsigned int i=0;i<4;i++)*id|=(uint32_t)p[4+i]<<(8*i);
 return *id!=0;
}
int b1_parse(const char *data,unsigned int n,b1_command *out){
 if(!data||!out||n!=8)return 0;
 const unsigned char *p=(const unsigned char*)data;uint32_t id;
 if(!header(p,&id)||(p[3]!=B1_START&&p[3]!=B1_STOP))return 0;
 out->kind=p[3];out->command_id=id;return 1;
}
unsigned int b1_map_entry(const uint8_t packed[B1_MAP_BYTES],unsigned int index){
 unsigned int value=0,at=index*3u;
 for(unsigned int i=0;i<3u;i++,at++)if(packed[at/8u]&(1u<<(at%8u)))value|=1u<<i;
 return value;
}
void b1_map_put(uint8_t packed[B1_MAP_BYTES],unsigned int index,unsigned int value){
 unsigned int at=index*3u;
 for(unsigned int i=0;i<3u;i++,at++){
  uint8_t bit=(uint8_t)(1u<<(at%8u));
  if((value>>i)&1u)packed[at/8u]|=bit;else packed[at/8u]&=(uint8_t)~bit;
 }
}
char b1_map_character(unsigned int index){
 return index<26u?(char)('A'+index):(char)('0'+(index-26u));
}
int b1_parse_map(const char *data,unsigned int n,b1_map_command *out){
 if(!data||!out||n<8)return 0;
 const unsigned char *p=(const unsigned char*)data;uint32_t id;
 if(!header(p,&id))return 0;
 if(p[3]==B1_MAP_SHOW){
  if(n!=8)return 0;
  for(unsigned int i=0;i<B1_MAP_BYTES;i++)out->packed[i]=0;
 }else if(p[3]==B1_MAP_SET){
  if(n!=8u+B1_MAP_BYTES)return 0;
  /* Bits 108..111 are spare and must be zero. */
  if(!valid_map(p+8))return 0;
  for(unsigned int i=0;i<B1_MAP_BYTES;i++)out->packed[i]=p[8+i];
 }else return 0;
 out->kind=p[3];out->command_id=id;return 1;
}

unsigned b1_encode_heartbeat(uint8_t *out,unsigned cap,uint32_t seq,uint32_t uptime,unsigned demo){
 if(!out||cap<B1_HEARTBEAT_SIZE||demo>1U)return 0;
 out[0]=0xb1;out[1]=1;out[2]=5;out[3]=(uint8_t)demo;
 put_u32(out+4,seq);put_u32(out+8,uptime);
 return B1_HEARTBEAT_SIZE;
}

unsigned b1_encode_result(uint8_t *out,unsigned cap,uint32_t id,unsigned status,unsigned demo,
 const uint8_t map[B1_MAP_BYTES],uint32_t revision){
 unsigned size=map?B1_RESULT_MAP_SIZE:B1_RESULT_SIZE;
 if(!out||cap<size||status>B1_RESULT_APPLIED||demo>1U)return 0;
 if(map){
  unsigned assigned=0U;
  if(status!=B1_RESULT_APPLIED||id==0U||revision==0U||(revision&1U)||!valid_map(map))return 0;
  for(unsigned i=0;i<B1_MAP_ENTRIES;i++){
   unsigned action=b1_map_entry(map,i);
   if(action==0U)continue;
   if(assigned&(1U<<action))return 0;
   assigned|=1U<<action;
  }
 }else if(status==B1_RESULT_APPLIED||revision!=0U)return 0;
 out[0]=0xb1;out[1]=1;out[2]=6;
 out[3]=(uint8_t)(status|(demo<<3)|(map?16U:0U));
 put_u32(out+4,id);
 if(map){
  put_u32(out+8,revision);
  for(unsigned i=0;i<B1_MAP_BYTES;i++)out[12+i]=map[i];
 }
 return size;
}
