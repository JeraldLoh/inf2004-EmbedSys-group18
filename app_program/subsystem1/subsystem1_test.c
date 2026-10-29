#include "subsystem1.h"
#include "buddy1_protocol.h"
#include "../test_report.h"
#if APP_TEST
EXPORT void subsystem1_test(void) {
 b1_command c;const unsigned char start[8]={0xb1,2,3,1,1,0,0,0};
 TEST_INIT();
 TEST_CHECK("B1-U01",b1_parse((const char*)start,8,&c) && c.kind==B1_START,"valid START");
 TEST_CHECK("B1-U02",!b1_parse("SET_SPEED 101",13,&c),"reject unsupported speed command");
 TEST_CHECK("B1-U03",!b1_parse("STOP junk",9,&c),"reject trailing bytes");
 TEST_SUMMARY("buddy1");
}
#endif
