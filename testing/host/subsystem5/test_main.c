#include "test.h"

int g_checks = 0;
int g_failures = 0;

int main(void) {
    printf("range\n");
    run_range_tests();
    printf("obstacle\n");
    run_obstacle_tests();
    printf("avoidance\n");
    run_avoidance_tests();

    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
