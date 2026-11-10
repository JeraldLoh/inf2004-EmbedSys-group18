#include "line_pid.h"

#include <stddef.h>

/* Sensor on the left edge: more black = car drifted right = steer left. */
#if LINE_PID_LEFT_EDGE
#define LINE_PID_SIGN (-1)
#else
#define LINE_PID_SIGN 1
#endif

static int32_t clamp(int32_t value, int32_t limit)
{
    if (value > limit)
    {
        return limit;
    }
    if (value < -limit)
    {
        return -limit;
    }
    return value;
}

void line_pid_init(line_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->white_level = 0U;
    pid->black_level = 0U;
    pid->calibrated = false;
    line_pid_reset(pid);
}

void line_pid_reset(line_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    pid->integral = 0;
    pid->filtered_x16 = 0;
    pid->window_start_x16 = 0;
    pid->window_start_ms = 0U;
    pid->d_term = 0;
    pid->previous_ms = 0U;
    pid->primed = false;
}

void line_pid_calibrate_start(line_pid_t *pid)
{
    if (pid == NULL)
    {
        return;
    }
    /* Start inverted so the first sample sets both levels. */
    pid->white_level = (uint16_t)LINE_PID_ADC_MAX;
    pid->black_level = 0U;
    pid->calibrated = false;
    line_pid_reset(pid);
}

void line_pid_calibrate_sample(line_pid_t *pid, uint16_t raw)
{
    uint16_t dark = line_pid_darkness(raw);

    if (pid == NULL)
    {
        return;
    }
    if (dark < pid->white_level)
    {
        pid->white_level = dark;
    }
    if (dark > pid->black_level)
    {
        pid->black_level = dark;
    }
}

bool line_pid_calibrate_finish(line_pid_t *pid)
{
    if (pid == NULL)
    {
        return false;
    }
    pid->calibrated = (pid->black_level > pid->white_level) &&
        ((uint16_t)(pid->black_level - pid->white_level) >= LINE_PID_MIN_CONTRAST);
    line_pid_reset(pid);
    return pid->calibrated;
}

uint16_t line_pid_darkness(uint16_t raw)
{
    if (raw > LINE_PID_ADC_MAX)
    {
        raw = LINE_PID_ADC_MAX;
    }
#if LINE_PID_BLACK_IS_HIGH
    return raw;
#else
    return (uint16_t)(LINE_PID_ADC_MAX - raw);
#endif
}

int32_t line_pid_error(const line_pid_t *pid, uint16_t raw)
{
    int32_t span;
    int32_t mid;

    if ((pid == NULL) || !pid->calibrated)
    {
        return 0;
    }
    span = (int32_t)pid->black_level - (int32_t)pid->white_level;
    mid = ((int32_t)pid->black_level + (int32_t)pid->white_level) / 2;

    /* White = -100, black = +100; readings past either level clamp. */
    return clamp((((int32_t)line_pid_darkness(raw) - mid) * 200) / span, 100);
}

int8_t line_pid_step(line_pid_t *pid, uint16_t raw, uint32_t now_ms)
{
    int32_t error;
    int32_t p;
    int32_t i;
    uint32_t window_ms;
#if LINE_PID_KI_X100 != 0
    int32_t dt_ms;
    int32_t i_limit;
#endif

    if ((pid == NULL) || !pid->calibrated)
    {
        return 0;
    }
    error = line_pid_error(pid, raw);

    if (pid->primed && ((now_ms - pid->previous_ms) > LINE_PID_GAP_MS))
    {
        line_pid_reset(pid);   /* stale history: start over */
    }

    p = (error * LINE_PID_KP_X100) / 100;

#if LINE_PID_KI_X100 != 0
    dt_ms = pid->primed ? (int32_t)(now_ms - pid->previous_ms) : 0;
    if (dt_ms > 0)
    {
        /* Clamp the stored sum so I alone never exceeds I_LIMIT. */
        i_limit = (LINE_PID_I_LIMIT * 100 * 1000) / LINE_PID_KI_X100;
        pid->integral = clamp(pid->integral + (error * dt_ms), i_limit);
    }
#endif
    i = (pid->integral * LINE_PID_KI_X100) / (100 * 1000);

    /* D: smooth the error (1/4 of the gap per step), then take its rate of
     * change over a whole window and hold it until the next window. */
    if (!pid->primed)
    {
        pid->filtered_x16 = error * 16;
        pid->window_start_x16 = pid->filtered_x16;
        pid->window_start_ms = now_ms;
        pid->d_term = 0;
    }
    else
    {
        pid->filtered_x16 += ((error * 16) - pid->filtered_x16) / 4;
        window_ms = now_ms - pid->window_start_ms;
        if (window_ms >= LINE_PID_D_WINDOW_MS)
        {
            /* Change of error per ms, x Kd. */
            pid->d_term = ((pid->filtered_x16 - pid->window_start_x16) *
                LINE_PID_KD_X100) / (100 * 16 * (int32_t)window_ms);
            pid->window_start_x16 = pid->filtered_x16;
            pid->window_start_ms = now_ms;
        }
    }

    pid->previous_ms = now_ms;
    pid->primed = true;
    return (int8_t)clamp(LINE_PID_SIGN * (p + i + pid->d_term), 100);
}

bool line_pid_saturated(const line_pid_t *pid, uint16_t raw)
{
    int32_t error;

    if ((pid == NULL) || !pid->calibrated)
    {
        return false;
    }
    error = line_pid_error(pid, raw);
    return (error >= LINE_PID_SAT_PERCENT) || (error <= -LINE_PID_SAT_PERCENT);
}

int8_t line_pid_blend(int8_t pid_steer, int8_t digital_steer, bool saturated)
{
    if (!saturated)
    {
        return pid_steer;
    }
    if ((pid_steer > 0) && (digital_steer > pid_steer))
    {
        return digital_steer;
    }
    if ((pid_steer < 0) && (digital_steer < pid_steer))
    {
        return digital_steer;
    }
    return pid_steer;
}
