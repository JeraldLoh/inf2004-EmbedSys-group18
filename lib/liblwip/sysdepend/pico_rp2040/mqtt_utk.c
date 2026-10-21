/* Buddy 1 transport. Only cyw43_utk_task calls this module (NO_SYS=1). */
#include <string.h>
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/buddy1_bridge.h"
#include <tm/tmonitor.h>
#ifndef B1_TOPIC_PREFIX
#define B1_TOPIC_PREFIX "group18/robot1"
#endif
#define COMMAND_TOPIC B1_TOPIC_PREFIX "/command"
static const char *topics[3]={B1_TOPIC_PREFIX "/heartbeat",B1_TOPIC_PREFIX "/telemetry",B1_TOPIC_PREFIX "/command_result"};
static mqtt_client_t *client;
static unsigned int next_try, attempt_at, last_pump;
static int connecting, subscribed, active, rx_valid;
static char rx[128];
static unsigned int rx_length, rx_expected;
static void offline(void) {
 connecting=0;subscribed=0;active=0;b1_connection(0);next_try=sys_now()+3000;
}
static void incoming_publish(void *arg,const char *topic,u32_t n) {
 (void)arg;rx_length=0;rx_expected=n;
 rx_valid=strcmp(topic,COMMAND_TOPIC)==0 && n>0 && n<=sizeof(rx);
}
static void incoming_data(void *arg,const u8_t *data,u16_t n,u8_t flags) {
 (void)arg;
 if(!rx_valid) return;
 if(rx_length+n>sizeof(rx) || rx_length+n>rx_expected) {rx_valid=0;return;}
 memcpy(rx+rx_length,data,n);rx_length+=n;
 if(flags & MQTT_DATA_FLAG_LAST) {
  if(rx_length==rx_expected && !b1_network_receive(rx,rx_length)) tm_printf((UB*)"buddy1: command dropped (queue full/not ready)\n");
  rx_valid=0;
 }
}
static void subscription_done(void *arg,err_t result) {
 (void)arg;
 if(result==ERR_OK) {subscribed=1;b1_connection(1);tm_printf((UB*)"buddy1: MQTT ready\n");}
 else {mqtt_disconnect(client);offline();tm_printf((UB*)"buddy1: subscribe failed %d\n",result);}
}
static void connection_done(mqtt_client_t *c,void *arg,mqtt_connection_status_t status) {
 (void)arg;connecting=0;
 if(status!=MQTT_CONNECT_ACCEPTED) {offline();return;}
 mqtt_set_inpub_callback(c,incoming_publish,incoming_data,0);
 if(mqtt_subscribe(c,COMMAND_TOPIC,0,subscription_done,0)!=ERR_OK) {mqtt_disconnect(c);offline();}
}
void mqtt_utk_poll(int link_up) {
 unsigned int now=sys_now();
 if(!link_up) {
  if(active) {mqtt_disconnect(client);offline();}
  return;
 }
 if(active && !subscribed && (unsigned int)(now-attempt_at)>15000) {mqtt_disconnect(client);offline();}
 if(!active && (int)(now-next_try)>=0) {
  ip_addr_t broker;
  struct mqtt_connect_client_info_t info={0};
  if(!client) client=mqtt_client_new();
  if(!client || !ip4addr_aton(MQTT_BROKER_IP,ip_2_ip4(&broker))) {next_try=now+10000;return;}
  IP_SET_TYPE(&broker,IPADDR_TYPE_V4);
  info.client_id=MQTT_CLIENT_ID;info.keep_alive=15;
  active=1;connecting=1;attempt_at=now;
  if(mqtt_client_connect(client,&broker,MQTT_BROKER_PORT,connection_done,0,&info)!=ERR_OK) offline();
 }
 if(!subscribed || !mqtt_client_is_connected(client) || (unsigned int)(now-last_pump)<20) return;
 last_pump=now;
 b1_connection(1); /* Also publishes ready state if application init finished after SUBACK. */
 /* One latest slot per type: a broker outage never creates a backlog. */
 for(unsigned int k=0;k<3;k++) {
  b1_packet packet;
  if(b1_network_take(k,&packet)) {
   err_t result=mqtt_publish(client,topics[k],packet.text,(u16_t)strlen(packet.text),0,0,0,0);
   if(result!=ERR_OK) b1_network_restore(&packet);
  }
 }
}
void mqtt_utk_start(void) {mqtt_utk_poll(1);}
