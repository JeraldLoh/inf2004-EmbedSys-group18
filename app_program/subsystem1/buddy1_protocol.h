#ifndef BUDDY1_PROTOCOL_H
#define BUDDY1_PROTOCOL_H
#define B1_START 1
#define B1_STOP 2
#define B1_SPEED 3
typedef struct { unsigned int kind, speed; } b1_command;
int b1_parse(const char *data, unsigned int length, b1_command *out);
#endif
