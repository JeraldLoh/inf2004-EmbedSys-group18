#ifndef BUDDY1_PROTOCOL_H
#define BUDDY1_PROTOCOL_H
#include <stdint.h>
#define B1_START 1u
#define B1_STOP 2u
typedef struct { uint32_t kind, command_id; } b1_command;
int b1_parse(const char *data,unsigned int length,b1_command *out);

/* Barcode route map (Buddy 3), set live from the dashboard.
 *
 * MAP_SET  (22 bytes): B1 02 03 03 id0 id1 id2 id3 + 14 map bytes
 * MAP_SHOW  (8 bytes): B1 02 03 04 id0 id1 id2 id3
 *
 * The 14 map bytes hold 36 entries of 3 bits, packed LSB-first: entry i
 * (A..Z then 0..9) is bits 3i..3i+2. Values: 0 none, 1 left, 2 right,
 * 3 straight, 4 U-turn (route_command_t). 5..7 and the 4 spare bits at
 * the end must be zero, otherwise the whole command is rejected.
 * MAP_SET replaces the whole table, so the dashboard and car always agree. */
#define B1_MAP_SET 3u
#define B1_MAP_SHOW 4u
#define B1_MAP_ENTRIES 36u
#define B1_MAP_BYTES 14u   /* 36 x 3 bits = 108 bits */
#define B1_MAP_MAX_VALUE 4u
typedef struct { uint32_t kind, command_id; uint8_t packed[B1_MAP_BYTES]; } b1_map_command;
int b1_parse_map(const char *data,unsigned int length,b1_map_command *out);
unsigned int b1_map_entry(const uint8_t packed[B1_MAP_BYTES],unsigned int index);
void b1_map_put(uint8_t packed[B1_MAP_BYTES],unsigned int index,unsigned int value);
char b1_map_character(unsigned int index);   /* 0..25 'A'..'Z', 26..35 '0'..'9' */
#endif
