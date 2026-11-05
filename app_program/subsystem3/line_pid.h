#ifndef GROUP18_LINE_PID_H
#define GROUP18_LINE_PID_H

/* Buddy 3: PID line steering from the analogue output of ONE line sensor
 * (hardware independent). Opt-in: built into the car only with LINE_PID=1.
 *
 * Wiring: no extra sensor. The LEFT line module's A0 pin goes to GP28 while
 * its D0 stays on GP7; the module drives both at once.
 *
 * The PID steers so the left sensor sits over the LEFT EDGE of the tape,
 * half on black, half on white (the car runs a few mm right of where the
 * digital steering holds it; the right sensor stays on white). The analogue
 * reading then changes smoothly as the car drifts, a real position error:
 *
 *    white <------- darkness -------> black
 *    car drifted left  |  car drifted right
 *                   target (on the edge)
 *
 *   error = darkness - midpoint, scaled to -100 (white) .. +100 (black)
 *   steer = Kp*e + Ki*sum(e*dt) + Kd*de/dt, clamped to -100..+100
 *
 * Output uses the same contract as line_follow_steer(): -100 full left,
 * +100 full right. The two digital sensors still detect junctions and line
 * loss; this file only replaces the steering number.
 *
 * Limit: if the car crosses the whole tape, the sensor sees white on the far
 * side and the error has the wrong sign. The digital sensors catch that case.
 */
#include <stdbool.h>
#include <stdint.h>

/* Same convention as BARCODE_BLACK_IS_HIGH: TCRT5000 A0 reads higher on
 * black. Set to 0 if your module measures the other way. */
#ifndef LINE_PID_BLACK_IS_HIGH
#define LINE_PID_BLACK_IS_HIGH 1
#endif

/* 1 = A0 taken from the LEFT line sensor (follows the tape's left edge),
 * 0 = from the RIGHT line sensor (follows the right edge). */
#ifndef LINE_PID_LEFT_EDGE
#define LINE_PID_LEFT_EDGE 1
#endif

/* Minimum black/white difference (ADC counts) to trust the calibration. */
#ifndef LINE_PID_MIN_CONTRAST
#define LINE_PID_MIN_CONTRAST 300U
#endif

/* Gains x100 (integer maths, the M0+ has no FPU). Tune on the track:
 * Kp first with Ki = Kd = 0 until it follows but weaves, then Kd until the
 * weave stops, then a little Ki only if it sits off the edge on curves.
 *
 * The edge is sharp: 0..100 % error is only ~5 mm of drift, so a small Kp
 * already steers hard. 0.80 weaved on a straight; 0.25 was smooth on the
 * straight but too weak on bends (2026-11-04 loop tests). */
#ifndef LINE_PID_KP_X100
#define LINE_PID_KP_X100 35     /* 0.35 steer per % error */
#endif
#ifndef LINE_PID_KI_X100
#define LINE_PID_KI_X100 0      /* per % error per second */
#endif
#ifndef LINE_PID_KD_X100
#define LINE_PID_KD_X100 0      /* per % error per ms; try 50..100 later */
#endif

/* D works on a smoothed error measured over this window, not sample to
 * sample: at 1 ms, ADC noise alone made the D term twitch. */
#ifndef LINE_PID_D_WINDOW_MS
#define LINE_PID_D_WINDOW_MS 10U
#endif

/* Most steering the I term alone may contribute (anti-windup). */
#ifndef LINE_PID_I_LIMIT
#define LINE_PID_I_LIMIT 30
#endif

/* At or past this |error| the sensor has run off the edge (fully on white or
 * fully on tape) and the PID cannot tell how far, so its output stops
 * growing. On a bend the digital ramp may then steer harder: see
 * line_pid_blend(). */
#ifndef LINE_PID_SAT_PERCENT
#define LINE_PID_SAT_PERCENT 90
#endif

/* A gap longer than this between steps (probe, bypass) restarts the PID so
 * stale history cannot kick the steering. */
#ifndef LINE_PID_GAP_MS
#define LINE_PID_GAP_MS 100U
#endif

#define LINE_PID_ADC_MAX 4095U

typedef struct
{
    uint16_t white_level;       /* darkness of white floor */
    uint16_t black_level;       /* darkness of black tape */
    bool calibrated;
    int32_t integral;           /* sum of error x ms */
    int32_t filtered_x16;       /* smoothed error x16 (for D) */
    int32_t window_start_x16;   /* filtered_x16 when the D window opened */
    uint32_t window_start_ms;
    int32_t d_term;             /* held between D windows */
    uint32_t previous_ms;
    bool primed;
} line_pid_t;

/* Clears calibration and history. Steps return 0 until calibrated. */
void line_pid_init(line_pid_t *pid);

/* Clears history only (keeps calibration). Call whenever the PID is not
 * steering, so it starts fresh when it takes over again. */
void line_pid_reset(line_pid_t *pid);

/* Feed raw samples during a sweep across the tape (both black and white). */
void line_pid_calibrate_start(line_pid_t *pid);
void line_pid_calibrate_sample(line_pid_t *pid, uint16_t raw);

/* Returns false if contrast is too low (sensor too high or not swept). */
bool line_pid_calibrate_finish(line_pid_t *pid);

/* Raw ADC value to darkness (higher = more black). */
uint16_t line_pid_darkness(uint16_t raw);

/* Error in percent: -100 (all white) .. 0 (on the edge) .. +100 (all black). */
int32_t line_pid_error(const line_pid_t *pid, uint16_t raw);

/* One control step. Returns steering -100 (left) .. +100 (right). */
int8_t line_pid_step(line_pid_t *pid, uint16_t raw, uint32_t now_ms);

/* True when |error| >= LINE_PID_SAT_PERCENT: the sensor is off the edge. */
bool line_pid_saturated(const line_pid_t *pid, uint16_t raw);

/* Final steering. Normally the PID. When the PID is saturated and the
 * digital steering (line_follow_steer) turns the SAME way but harder, use
 * the digital value: its ramp keeps growing on a bend, the PID cannot.
 * An opposite-sign digital value (e.g. its hold toward a stale side) is
 * ignored. */
int8_t line_pid_blend(int8_t pid_steer, int8_t digital_steer, bool saturated);

#endif
