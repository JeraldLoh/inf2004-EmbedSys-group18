#ifndef BUDDY5_TEST_H
#define BUDDY5_TEST_H

#include <math.h>
#include <stdio.h>

extern int g_checks;
extern int g_failures;

#define CHECK(cond)                                                            \
    do {                                                                       \
        g_checks++;                                                            \
        if (!(cond)) {                                                         \
            g_failures++;                                                      \
            printf("    FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);         \
        }                                                                      \
    } while (0)

#define CHECK_INT(actual, expected)                                            \
    do {                                                                       \
        long long a_ = (long long)(actual), e_ = (long long)(expected);        \
        g_checks++;                                                            \
        if (a_ != e_) {                                                        \
            g_failures++;                                                      \
            printf("    FAIL %s:%d: %s == %lld, expected %lld\n", __FILE__,    \
                   __LINE__, #actual, a_, e_);                                 \
        }                                                                      \
    } while (0)

#define CHECK_NEAR(actual, expected, tol)                                      \
    do {                                                                       \
        double a_ = (double)(actual), e_ = (double)(expected);                 \
        g_checks++;                                                            \
        if (!(fabs(a_ - e_) <= (double)(tol))) {                               \
            g_failures++;                                                      \
            printf("    FAIL %s:%d: %s == %.4f, expected %.4f +/- %.4f\n",     \
                   __FILE__, __LINE__, #actual, a_, e_, (double)(tol));        \
        }                                                                      \
    } while (0)

#define RUN(fn)                                                                \
    do {                                                                       \
        int before_ = g_failures;                                              \
        fn();                                                                  \
        printf("  %-4s %s\n", g_failures == before_ ? "ok" : "FAIL", #fn);    \
    } while (0)

void run_range_tests(void);
void run_obstacle_tests(void);
void run_avoidance_tests(void);

#endif
