#ifndef GROUP18_MOTION_CALIBRATION_H
#define GROUP18_MOTION_CALIBRATION_H

/*
 * Measured on the assembled vehicle on 2026-10-22.
 * Each value is the mean of three manually observed 10-wheel-revolution runs.
 * The production encoder input must count BOTH rising and falling edges of
 * channel B.  GP27 is left and GP5 is right.
 */
#define MOTION_LEFT_BOTH_EDGE_COUNTS_PER_REV   1271u
#define MOTION_RIGHT_BOTH_EDGE_COUNTS_PER_REV  1264u

/* Candidate provenance: three completed 10-turn totals, raw evidence in
 * testing/subsystem2/logs/2026-10-22_encoder-ten-revolution-raw.txt.
 * L:12656/12864/12603; R:12723/12630/12558. Original acquisition was 50 us
 * polling, not the new dispatcher. Requalify edge/filter convention on target.
 * Diameter is user-measured. No effective pivot-width calibration is implied. */
#define MOTION_CALIBRATION_VERSION 1u
#define MOTION_WHEEL_DIAMETER_MM_CANDIDATE 65u

#endif
