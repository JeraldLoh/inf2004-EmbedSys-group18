#ifndef GROUP18_MQTT_CONSOLE_H
#define GROUP18_MQTT_CONSOLE_H

#include <tk/typedef.h>

/* Radio task only. Removing bytes is separate from copying so a full lwIP
 * output ring can defer the batch without losing it. */
UW tm_mqtt_console_peek(UB *out, UW capacity, UW *dropped);
void tm_mqtt_console_consume(UW count);

#endif
