#include "circuit_controls.h"
#include <stddef.h>
bool circuit_controls_update(circuit_controls_t *c,uint32_t now,bool start,bool stop)
{
    if(c==NULL) { return false; }
    if(c->effort!=150U && c->effort!=200U && c->effort!=250U && c->effort!=300U)
    { c->effort=150U; }
    if(!stop) { c->phase=0U; return false; }
    if(c->phase==0U) { if(!start) { c->phase=1U; } }
    else if(c->phase==1U) { if(start) { c->phase=2U; c->since=now; } }
    else if(c->phase==2U && !start) {
        c->phase=now-c->since>=30000U?3U:1U; c->since=now;
    }
    else if(c->phase==3U) {
        if(start) { c->phase=2U; c->since=now; }
        else if(now-c->since>=30000U) {
            c->effort=c->effort==300U?150U:(uint16_t)(c->effort+50U);
            c->phase=1U; return true;
        }
    }
    return false;
}
