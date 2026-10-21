#include "subsystem1.h"
#include "buddy1_protocol.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem1_test(void) {
 b1_command c;
 TEST_INIT();
 TEST_CHECK("B1-U01",b1_parse("START",5,&c) && c.kind==B1_START,"valid START");
 TEST_CHECK("B1-U02",!b1_parse("SET_SPEED 101",13,&c),"reject speed above 100");
 TEST_CHECK("B1-U03",!b1_parse("STOP junk",9,&c),"reject trailing bytes");
 TEST_SUMMARY("buddy1");
}
#endif
