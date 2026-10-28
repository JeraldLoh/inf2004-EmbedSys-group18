#include "line_drive.h"

#include <stddef.h>

#ifndef S3_BENCH_BASE_EFFORT
#define S3_BENCH_BASE_EFFORT 200      /* 20% */
#endif
#ifndef S3_BENCH_MAX_EFFORT
#define S3_BENCH_MAX_EFFORT 350       /* 35% */
#endif
#ifndef S3_BENCH_GAIN
#define S3_BENCH_GAIN 2               /* steer 100 -> +/-200 effort */
#endif
#ifndef S3_BENCH_JUNCTION_WAIT_MS
#define S3_BENCH_JUNCTION_WAIT_MS 1500U
#endif
#ifndef S3_BENCH_CROSS_MS
#define S3_BENCH_CROSS_MS 400U
#endif
#ifndef S3_BENCH_TURN_MS
#define S3_BENCH_TURN_MS 700U
#endif

static int16_t clamp(int32_t value, int16_t max)
{
    if (value < 0)
    {
        return 0;
    }
    if (value > max)
    {
        return max;
    }
    return (int16_t)value;
}

void line_drive_default_config(line_drive_config_t *config)
{
    if (config == NULL)
    {
        return;
    }
    config->base_effort = S3_BENCH_BASE_EFFORT;
    config->max_effort = S3_BENCH_MAX_EFFORT;
    config->gain = S3_BENCH_GAIN;
    config->junction_wait_ms = S3_BENCH_JUNCTION_WAIT_MS;
    config->cross_ms = S3_BENCH_CROSS_MS;
    config->turn_ms = S3_BENCH_TURN_MS;
}

void line_drive_init(line_drive_t *drive, const line_drive_config_t *config)
{
    if ((drive == NULL) || (config == NULL))
    {
        return;
    }
    drive->config = *config;
    if (drive->config.max_effort < 0)
    {
        drive->config.max_effort = 0;
    }
    drive->state = LINE_DRIVE_FOLLOW;
    drive->since_ms = 0U;
    drive->announced = false;
    drive->pending_route = ROUTE_COMMAND_NONE;
    drive->pending_character = '\0';
    drive->go_command = ROUTE_COMMAND_NONE;
    drive->go_character = '\0';
}

void line_drive_mix(int8_t steer, int16_t base, int16_t gain, int16_t max,
    int16_t *left, int16_t *right)
{
    int32_t turn = (int32_t)gain * (int32_t)steer;

    if (left != NULL)
    {
        *left = clamp((int32_t)base + turn, max);
    }
    if (right != NULL)
    {
        *right = clamp((int32_t)base - turn, max);
    }
}

void line_drive_route(line_drive_t *drive, route_command_t command, char character)
{
    if ((drive == NULL) || (command == ROUTE_COMMAND_NONE))
    {
        return;
    }
    drive->pending_route = command;
    drive->pending_character = character;
}

void line_drive_junction(line_drive_t *drive, uint32_t now_ms)
{
    if ((drive == NULL) || (drive->state != LINE_DRIVE_FOLLOW))
    {
        return;
    }
    drive->state = LINE_DRIVE_JUNCTION_WAIT;
    drive->since_ms = now_ms;
    drive->announced = false;
}

line_drive_event_t line_drive_step(line_drive_t *drive, int8_t steer,
    uint32_t now_ms, int16_t *left, int16_t *right)
{
    line_drive_event_t event = LINE_DRIVE_EVENT_NONE;
    int16_t base;
    int16_t l = 0;
    int16_t r = 0;
    uint32_t age;

    if (drive == NULL)
    {
        if (left != NULL) { *left = 0; }
        if (right != NULL) { *right = 0; }
        return LINE_DRIVE_EVENT_NONE;
    }
    base = clamp(drive->config.base_effort, drive->config.max_effort);
    age = now_ms - drive->since_ms;

    switch (drive->state)
    {
        case LINE_DRIVE_FOLLOW:
            line_drive_mix(steer, base, drive->config.gain,
                drive->config.max_effort, &l, &r);
            break;

        case LINE_DRIVE_JUNCTION_WAIT:
            if (!drive->announced)
            {
                drive->announced = true;
                event = LINE_DRIVE_EVENT_JUNCTION_STOP;
            }
            if (age >= drive->config.junction_wait_ms)
            {
                /* No barcode before this junction: go straight. */
                drive->go_command = (drive->pending_route != ROUTE_COMMAND_NONE)
                    ? drive->pending_route : ROUTE_COMMAND_STRAIGHT;
                drive->go_character = (drive->pending_route != ROUTE_COMMAND_NONE)
                    ? drive->pending_character : '-';
                drive->pending_route = ROUTE_COMMAND_NONE;
                drive->since_ms = now_ms;
                event = LINE_DRIVE_EVENT_GO;
                switch (drive->go_command)
                {
                    case ROUTE_COMMAND_LEFT:
                        drive->state = LINE_DRIVE_TURN_LEFT;
                        r = base;
                        break;
                    case ROUTE_COMMAND_RIGHT:
                        drive->state = LINE_DRIVE_TURN_RIGHT;
                        l = base;
                        break;
                    case ROUTE_COMMAND_UTURN:
                        drive->state = LINE_DRIVE_DONE;
                        break;
                    default:
                        drive->state = LINE_DRIVE_CROSS;
                        l = base;
                        r = base;
                        break;
                }
            }
            break;

        case LINE_DRIVE_CROSS:
        case LINE_DRIVE_TURN_LEFT:
        case LINE_DRIVE_TURN_RIGHT:
        {
            uint32_t limit = (drive->state == LINE_DRIVE_CROSS)
                ? drive->config.cross_ms : drive->config.turn_ms;
            if (age >= limit)
            {
                drive->state = LINE_DRIVE_FOLLOW;
                event = LINE_DRIVE_EVENT_RESUME;
                line_drive_mix(steer, base, drive->config.gain,
                    drive->config.max_effort, &l, &r);
            }
            else
            {
                l = (drive->state == LINE_DRIVE_TURN_LEFT) ? 0 : base;
                r = (drive->state == LINE_DRIVE_TURN_RIGHT) ? 0 : base;
            }
            break;
        }

        default:
            break;   /* DONE: stay stopped */
    }

    if ((drive->state == LINE_DRIVE_DONE) && (event == LINE_DRIVE_EVENT_GO))
    {
        event = LINE_DRIVE_EVENT_DONE;
    }
    if (left != NULL) { *left = l; }
    if (right != NULL) { *right = r; }
    return event;
}
