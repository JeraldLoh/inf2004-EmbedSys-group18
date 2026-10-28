#include "range.h"
#include "test.h"

static range_reading_t ok(float cm) { return (range_reading_t){RANGE_OK, cm}; }
static range_reading_t st(range_status_t s) { return (range_reading_t){s, 0.0f}; }

static void median_rejects_single_spike(void) {
    range_reading_t s[] = {ok(10.0f), ok(300.0f), ok(11.0f)};
    range_reading_t r = range_combine(s, 3);
    CHECK_INT(r.status, RANGE_OK);
    CHECK_NEAR(r.cm, 11.0f, 1e-6);
}

static void median_of_unsorted_input(void) {
    range_reading_t s[] = {ok(30.0f), ok(10.0f), ok(20.0f), ok(50.0f), ok(40.0f)};
    CHECK_NEAR(range_combine(s, 5).cm, 30.0f, 1e-6);
}

static void even_count_averages_middle_pair(void) {
    range_reading_t s[] = {ok(12.0f), st(RANGE_NO_ECHO), ok(10.0f), st(RANGE_NO_ECHO)};
    range_reading_t r = range_combine(s, 4);
    CHECK_INT(r.status, RANGE_OK); // exactly half OK is accepted
    CHECK_NEAR(r.cm, 11.0f, 1e-6);
}

static void failure_majority_wins(void) {
    range_reading_t s[] = {ok(10.0f), st(RANGE_NO_ECHO), st(RANGE_NO_ECHO)};
    CHECK_INT(range_combine(s, 3).status, RANGE_NO_ECHO);
}

static void failure_tie_is_conservative(void) {
    range_reading_t a[] = {st(RANGE_NO_ECHO), st(RANGE_TOO_CLOSE), ok(5.0f)};
    CHECK_INT(range_combine(a, 3).status, RANGE_TOO_CLOSE);
    range_reading_t b[] = {st(RANGE_NO_ECHO), st(RANGE_FAULT), ok(5.0f)};
    CHECK_INT(range_combine(b, 3).status, RANGE_FAULT);
}

static void invalid_inputs_are_faults(void) {
    range_reading_t s[RANGE_MAX_SAMPLES + 1] = {0};
    CHECK_INT(range_combine(NULL, 3).status, RANGE_FAULT);
    CHECK_INT(range_combine(s, 0).status, RANGE_FAULT);
    CHECK_INT(range_combine(s, RANGE_MAX_SAMPLES + 1).status, RANGE_FAULT);
}

static void non_finite_or_negative_samples_rejected(void) {
    range_reading_t s[] = {ok(NAN), ok(10.0f), ok(12.0f)};
    range_reading_t r = range_combine(s, 3);
    CHECK_INT(r.status, RANGE_OK);
    CHECK_NEAR(r.cm, 11.0f, 1e-6);

    range_reading_t n[] = {ok(-5.0f), ok(-1.0f), ok(10.0f)};
    CHECK_INT(range_combine(n, 3).status, RANGE_FAULT);

    range_reading_t bad[] = {{(range_status_t)42, 1.0f}, ok(7.0f), ok(9.0f)};
    CHECK_NEAR(range_combine(bad, 3).cm, 8.0f, 1e-6);
}

static void effective_distance_mapping(void) {
    CHECK_NEAR(range_effective_cm(ok(17.5f), 400.0f), 17.5f, 1e-6);
    CHECK_NEAR(range_effective_cm(st(RANGE_NO_ECHO), 400.0f), 400.0f, 1e-6);
    CHECK_NEAR(range_effective_cm(st(RANGE_TOO_CLOSE), 400.0f), 0.0f, 1e-6);
    CHECK_NEAR(range_effective_cm(st(RANGE_FAULT), 400.0f), 0.0f, 1e-6);
}

static void echo_width_to_distance(void) {
    float k = range_cm_per_us(20.0f);
    CHECK_NEAR(k, 0.017172f, 1e-5);             // 343.4 m/s round trip
    CHECK_NEAR(range_cm_per_us(0.0f), 0.016565f, 1e-5);

    range_reading_t r = range_from_echo_us(583U, k, 2.0f, 400.0f); // ~10 cm
    CHECK_INT(r.status, RANGE_OK);
    CHECK_NEAR(r.cm, 10.01f, 0.02);
    CHECK_INT(range_from_echo_us(116U, k, 2.0f, 400.0f).status, RANGE_TOO_CLOSE); // 1.99 cm
    CHECK_INT(range_from_echo_us(117U, k, 2.0f, 400.0f).status, RANGE_OK);        // 2.01 cm
    CHECK_INT(range_from_echo_us(23200U, k, 2.0f, 400.0f).status, RANGE_OK);      // 398.4 cm
    CHECK_INT(range_from_echo_us(23400U, k, 2.0f, 400.0f).status, RANGE_NO_ECHO); // 401.8 cm
    CHECK_INT(range_from_echo_us(38000U, k, 2.0f, 400.0f).status, RANGE_NO_ECHO); // sensor timeout
    CHECK_INT(range_from_echo_us(0U, k, 2.0f, 400.0f).status, RANGE_TOO_CLOSE);
}

void run_range_tests(void) {
    RUN(median_rejects_single_spike);
    RUN(median_of_unsorted_input);
    RUN(even_count_averages_middle_pair);
    RUN(failure_majority_wins);
    RUN(failure_tie_is_conservative);
    RUN(invalid_inputs_are_faults);
    RUN(non_finite_or_negative_samples_rejected);
    RUN(effective_distance_mapping);
    RUN(echo_width_to_distance);
}
