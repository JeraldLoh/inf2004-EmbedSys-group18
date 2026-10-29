#ifndef BUDDY1_DIAGNOSTICS_H
#define BUDDY1_DIAGNOSTICS_H
#include "buddy1_binary.h"
#define B1_DIAGNOSTICS_SIZE 52u
typedef struct {
 uint32_t uptime_ms,rejected,out_of_range,busy,last_at_ms;
 b1_validation_error last;
} b1_diagnostics;
unsigned int b1_encode_diagnostics(unsigned char *out,unsigned int capacity,
 const b1_diagnostics *d,int demo);
#endif
