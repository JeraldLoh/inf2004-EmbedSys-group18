#ifndef B2_TEST_SYSDEF_H
#define B2_TEST_SYSDEF_H
#include <tk/tkernel.h>
#define IO_BANK0_BASE 0x40014000U
#define SIO_CPUID_ADDR 0xD0000000U
#define SCB_ICSR 0xE000ED04U
#define TIMER_TIMERAWL 0x40054028U
#define TIMER_ALARM0 0x40054010U
#define TIMER_ARMED 0x40054020U
#define TIMER_INTR 0x40054034U
#define TIMER_INTE 0x40054038U
#define GPIO(n) (0x4001C004U + (4U * (n)))
#define GPIO_PUE 8U
#define GPIO_PDE 4U
UW in_w(UW address);
void out_w(UW address, UW value);
#define set_w(address, bits) out_w((address), in_w(address) | (bits))
#define clr_w(address, bits) out_w((address), in_w(address) & ~(bits))
#endif
