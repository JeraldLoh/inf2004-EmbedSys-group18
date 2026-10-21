#ifndef BUDDY1_RETRY_H
#define BUDDY1_RETRY_H
/* CYW43 link states: UP=3, JOIN=1, NOIP=2, DOWN=0, errors negative. */
static inline int b1_wifi_retry_due(int status, unsigned int elapsed_ms) {
 return status!=3 && elapsed_ms>=((status==1 || status==2)?30000u:5000u);
}
#endif
