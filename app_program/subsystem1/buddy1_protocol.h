#ifndef BUDDY1_PROTOCOL_H
#define BUDDY1_PROTOCOL_H
#include <stdint.h>
#define B1_START 1u
#define B1_STOP 2u
typedef struct { uint32_t kind, command_id; } b1_command;
int b1_parse(const char *data,unsigned int length,b1_command *out);
#endif
