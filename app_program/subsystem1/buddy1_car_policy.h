#ifndef BUDDY1_CAR_POLICY_H
#define BUDDY1_CAR_POLICY_H
#include "buddy1_protocol.h"
#include "../contracts/system_types.h"
/* Remote START never clears physical, sensor, motion or other safety stops. */
static inline int b1_car_start_allowed(uint32_t reasons, int stop_pressed,
                                     int running, uint32_t hardware_faults)
{
 return !stop_pressed && !running && !hardware_faults &&
        !(reasons & ~((uint32_t)STOP_REASON_REMOTE));
}
/* Keep the supervisor run timer active while queued mode events are applied.
 * 0=still waiting, 1=AUTO observed, -1=refused/timed out. */
static inline int b1_car_start_progress(uint32_t mode,uint32_t reasons,
                                      uint32_t since,uint32_t now)
{
 if(mode==SYSTEM_MODE_AUTO) return 1;
 if((reasons & ~((uint32_t)STOP_REASON_REMOTE)) ||
    (mode!=SYSTEM_MODE_STARTUP && mode!=SYSTEM_MODE_IDLE && mode!=SYSTEM_MODE_STOPPED) ||
    (uint32_t)(now-since)>=1000U) return -1;
 return 0;
}
/* Within a bounded ingress batch, STOP cannot be overwritten by START. */
static inline unsigned b1_car_command_choice(unsigned current,unsigned next)
{
 return current==B1_STOP || next==B1_STOP ? B1_STOP : next;
}
#endif
