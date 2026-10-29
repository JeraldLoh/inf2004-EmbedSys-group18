#include <assert.h>
#include "buddy1_retry.h"
int main(void) {
 assert(!b1_wifi_retry_due(3,60000));
 assert(!b1_wifi_retry_due(0,4999));
 assert(b1_wifi_retry_due(0,5000));
 assert(b1_wifi_retry_due(-3,5000));
 assert(!b1_wifi_retry_due(1,29999));
 assert(b1_wifi_retry_due(1,30000));
 assert(b1_wifi_retry_due(2,30000));
 return 0;
}
