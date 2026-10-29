#ifndef MQTT_UTK_H
#define MQTT_UTK_H
/* Must run only in the radio-owned task, after sys_check_timeouts(). */
void mqtt_utk_poll(int link_up);
void mqtt_utk_start(void);
#endif
