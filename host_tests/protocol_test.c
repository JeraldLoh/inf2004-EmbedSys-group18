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
 assert(!b1_parse("START",5,&c));assert(!b1_parse("SET_SPEED 100",13,&c));assert(!b1_parse(0,8,&c));
 /* Barcode route map: K=left, Q=right, Z=straight, 7=U-turn. */
 b1_map_command m;unsigned char set[22]={0xb1,2,3,3,9,0,0,0};
 b1_map_put(set+8,'K'-'A',1);b1_map_put(set+8,'Q'-'A',2);b1_map_put(set+8,'Z'-'A',3);b1_map_put(set+8,26+7,4);
 assert(b1_parse_map((char*)set,22,&m)&&m.kind==B1_MAP_SET&&m.command_id==9u);
 assert(b1_map_entry(m.packed,'K'-'A')==1&&b1_map_entry(m.packed,'Q'-'A')==2&&b1_map_entry(m.packed,'Z'-'A')==3);
 assert(b1_map_entry(m.packed,33)==4&&b1_map_entry(m.packed,0)==0&&b1_map_character(33)=='7'&&b1_map_character(10)=='K');
 b1_map_put(set+8,'K'-'A',1);assert(b1_map_entry(set+8,'K'-'A')==1);  /* put overwrites, not ORs */
 assert(!b1_parse((char*)set,22,&c));                 /* START/STOP parser ignores it */
 assert(!b1_parse_map((char*)set,21,&m)&&!b1_parse_map((char*)set,23,&m));
 b1_map_put(set+8,3,5);assert(!b1_parse_map((char*)set,22,&m));b1_map_put(set+8,3,0);   /* value 5 */
 set[21]|=0x10;assert(!b1_parse_map((char*)set,22,&m));set[21]&=0x0f;                   /* spare bits */
 memset(set+4,0,4);assert(!b1_parse_map((char*)set,22,&m));                            /* id 0 */
 unsigned char show[8]={0xb1,2,3,4,1,0,0,0};
 assert(b1_parse_map((char*)show,8,&m)&&m.kind==B1_MAP_SHOW&&m.command_id==1u);
 assert(!b1_parse_map((char*)show,22,&m));show[3]=1;assert(!b1_parse_map((char*)show,8,&m));
 return 0;
}
