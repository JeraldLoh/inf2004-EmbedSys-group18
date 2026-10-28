/* Buddy 5 mission bridge: avoidance states -> vehicle controller missions.
 * Separate executable: links the real controller and controller service. */
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "b5_mission_bridge.h"
#include "test.h"

int g_checks = 0;
int g_failures = 0;

/* ---------------------------------------------------------------------------
 * Bridge + real controller service
 * ------------------------------------------------------------------------- */

typedef struct
{
    controller_service_t service;
    b5_bridge_t bridge;
    int stops;
    int events;
} rig_t;

static bool rig_stop(void *context)
{
    ((rig_t *)context)->stops++;
    return true;
}

static void rig_init(rig_t *r)
{
    controller_outputs_t outputs = {0};
    controller_event_t auto_mode = {CONTROLLER_EVENT_SET_MODE, SYSTEM_MODE_AUTO,
        STOP_REASON_NONE, MISSION_IDLE, false};

    memset(r, 0, sizeof *r);
    outputs.context = r;
    outputs.stop_motion = rig_stop;
    CHECK(controller_service_init(&r->service, &outputs));
    CHECK(controller_service_handle(&r->service, &auto_mode));
    b5_bridge_init(&r->bridge);
    r->stops = 0;
}

static void rig_feed(rig_t *r, av_state_t state, bool sensor_fault)
{
    controller_event_t out[B5_BRIDGE_MAX_EVENTS];
    size_t n = b5_bridge_update(&r->bridge, state, sensor_fault, out);
    for (size_t i = 0; i < n; i++)
    {
        (void)controller_service_handle(&r->service, &out[i]);
        r->events++;
    }
}

static void rig_feed_all(rig_t *r, const av_state_t *states, size_t count)
{
    for (size_t i = 0; i < count; i++)
    {
        rig_feed(r, states[i], false);
        rig_feed(r, states[i], false); /* repeated ticks emit nothing new */
    }
}

#define FEED(r, ...)                                                          \
    do {                                                                      \
        const av_state_t s_[] = {__VA_ARGS__};                                \
        rig_feed_all((r), s_, sizeof s_ / sizeof s_[0]);                      \
    } while (0)

static void full_bypass_follows_missions(void)
{
    rig_t r;
    rig_init(&r);
    FEED(&r, AV_STATE_MONITOR, AV_STATE_COARSE_SCAN);
    CHECK_INT(r.service.status.mission, MISSION_SCAN);
    CHECK_INT(r.stops, 1);
    FEED(&r, AV_STATE_FINE_SCAN_RIGHT, AV_STATE_FINE_SCAN_LEFT,
         AV_STATE_TURN_SCAN_RIGHT, AV_STATE_TURN_SCAN_LEFT);
    CHECK_INT(r.service.status.mission, MISSION_SCAN);
    FEED(&r, AV_STATE_DECIDE);
    CHECK_INT(r.service.status.mission, MISSION_PLAN_BYPASS);
    FEED(&r, AV_STATE_BYPASS);
    CHECK_INT(r.service.status.mission, MISSION_BYPASS);
    FEED(&r, AV_STATE_REACQUIRE);
    CHECK_INT(r.service.status.mission, MISSION_RECOVERY);
    FEED(&r, AV_STATE_MONITOR);
    CHECK_INT(r.service.status.mission, MISSION_VERIFY_LINE);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.events, 6); /* detected, scan, plan, bypass, recovery, cleared */
    CHECK_INT(r.stops, 1);
}

static void obstacle_gone_resumes_tracking(void)
{
    rig_t r;
    rig_init(&r);
    FEED(&r, AV_STATE_MONITOR, AV_STATE_COARSE_SCAN, AV_STATE_MONITOR);
    CHECK_INT(r.service.status.mission, MISSION_TRACK_LINE);
    CHECK_INT(r.service.status.events_rejected, 0);
}

static void rescan_reverse_and_replans_are_accepted(void)
{
    rig_t r;
    rig_init(&r);
    FEED(&r, AV_STATE_COARSE_SCAN, AV_STATE_FINE_SCAN_RIGHT, AV_STATE_DECIDE,
         AV_STATE_WAIT_RESCAN, AV_STATE_COARSE_SCAN, AV_STATE_DECIDE,
         AV_STATE_REVERSE, AV_STATE_COARSE_SCAN, AV_STATE_DECIDE,
         AV_STATE_BYPASS,
         AV_STATE_COARSE_SCAN,              /* emergency stop mid-bypass */
         AV_STATE_DECIDE, AV_STATE_BYPASS, AV_STATE_REACQUIRE,
         AV_STATE_COARSE_SCAN,              /* emergency stop while searching */
         AV_STATE_REACQUIRE,                /* gone, already off the line */
         AV_STATE_MONITOR);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.mission, MISSION_VERIFY_LINE);
}

static void skipped_steps_are_filled_in(void)
{
    rig_t r;
    rig_init(&r);
    FEED(&r, AV_STATE_MONITOR, AV_STATE_BYPASS); /* worst case: all at once */
    CHECK_INT(r.events, 4);
    CHECK_INT(r.service.status.mission, MISSION_BYPASS);
    FEED(&r, AV_STATE_COARSE_SCAN, AV_STATE_BYPASS);
    CHECK_INT(r.service.status.mission, MISSION_BYPASS);
    CHECK_INT(r.service.status.events_rejected, 0);
}

static void failures_latch_once(void)
{
    rig_t r;
    rig_init(&r);
    FEED(&r, AV_STATE_COARSE_SCAN, AV_STATE_FINE_SCAN_RIGHT);
    rig_feed(&r, AV_STATE_FAILED, true);
    rig_feed(&r, AV_STATE_FAILED, true);
    CHECK_INT(r.service.status.mission, MISSION_SAFE_STOP);
    CHECK_INT(r.service.status.stop_reasons, STOP_REASON_SENSOR_CRITICAL);
    CHECK_INT(r.events, 3); /* detected, scan, failed */

    /* Buddy 5 reset: no event, the latched stop stays until cleared. */
    FEED(&r, AV_STATE_MONITOR);
    CHECK_INT(r.events, 3);
    CHECK_INT(r.service.status.mission, MISSION_SAFE_STOP);

    /* A sensor fault while just monitoring is still reported. */
    rig_init(&r);
    rig_feed(&r, AV_STATE_MONITOR, false);
    rig_feed(&r, AV_STATE_FAILED, false);
    CHECK_INT(r.service.status.stop_reasons, STOP_REASON_RECOVERY_FAILED);
    CHECK_INT(r.service.status.events_rejected, 0);
}

static void no_mission_restart_after_failure(void)
{
    b5_bridge_t b;
    controller_event_t out[B5_BRIDGE_MAX_EVENTS];
    b5_bridge_init(&b);
    CHECK_INT(b5_bridge_update(&b, AV_STATE_FAILED, false, out), 1);
    /* Not via MONITOR: nothing is started. */
    CHECK_INT(b5_bridge_update(&b, AV_STATE_COARSE_SCAN, false, out), 0);
    CHECK_INT(b5_bridge_update(&b, AV_STATE_MONITOR, false, out), 0);
    CHECK_INT(b5_bridge_update(&b, AV_STATE_COARSE_SCAN, false, out), 2);
    CHECK_INT(out[0].type, CONTROLLER_EVENT_OBSTACLE_DETECTED);
    CHECK_INT(b5_bridge_update(NULL, AV_STATE_COARSE_SCAN, false, out), 0);
    CHECK_INT(b5_bridge_update(&b, AV_STATE_COARSE_SCAN, false, NULL), 0);
}

/* ---------------------------------------------------------------------------
 * Real avoidance state machine in a small world, bridged to the controller
 * ------------------------------------------------------------------------- */

typedef struct
{
    uint32_t t;
    int servo;
    int heading;
    int lo, hi;          /* obstacle, world directions (90 = ahead, + = left) */
    float obstacle_cm;
    float bg_cm;
    bool fault;
    bool moving;
    uint32_t motion_end;
    bool searching;
    uint32_t search_start;
    bool remove_on_stop;
    int stops;
} world_t;

static uint32_t w_now(void *c) { return ((world_t *)c)->t; }
static void w_servo(void *c, int deg) { ((world_t *)c)->servo = deg; }
static range_reading_t w_measure(void *c)
{
    world_t *w = c;
    int dir = w->servo + w->heading; /* default config: higher angle = left */
    w->t += 90;
    if (w->fault) return (range_reading_t){RANGE_FAULT, 0.0f};
    if (dir >= w->lo && dir <= w->hi) return (range_reading_t){RANGE_OK, w->obstacle_cm};
    return (range_reading_t){RANGE_OK, w->bg_cm};
}
static void w_move(world_t *w) { w->moving = true; w->motion_end = w->t + 200; }
static void w_stop(void *c)
{
    world_t *w = c;
    w->moving = false;
    w->searching = false;
    if (w->remove_on_stop && w->stops++ == 0) w->lo = w->hi = -1000;
}
static void w_turn(void *c, float deg) { world_t *w = c; w->heading += (int)deg; w_move(w); }
static void w_fwd(void *c, float cm) { (void)cm; w_move(c); }
static void w_search(void *c, side_t s)
{
    world_t *w = c;
    (void)s;
    w->searching = true;
    w->search_start = w->t;
}
static void w_resume(void *c) { ((world_t *)c)->searching = false; }
static bool w_done(void *c)
{
    world_t *w = c;
    return !w->moving || (int32_t)(w->t - w->motion_end) >= 0;
}
static bool w_line(void *c)
{
    world_t *w = c;
    return w->searching && w->t - w->search_start >= 300;
}

static void run_world(world_t *w, rig_t *r, av_state_t until, uint32_t max_ms)
{
    av_t av;
    av_config_t cfg;
    av_platform_t p = {w, w_now, w_servo, w_measure, w_stop, w_turn, w_fwd, w_fwd,
        w_search, w_resume, w_done, NULL, w_line, NULL};
    bool left_monitor = false;

    av_config_default(&cfg);
    CHECK(av_init(&av, &cfg, &p));
    rig_init(r);
    for (uint32_t end = w->t + max_ms; (int32_t)(w->t - end) < 0; w->t += 10)
    {
        av_tick(&av);
        rig_feed(r, av_state(&av), av.fault_streak >= cfg.max_consecutive_faults);
        if (av_state(&av) != AV_STATE_MONITOR) left_monitor = true;
        if (left_monitor && av_state(&av) == until) break;
    }
    CHECK_INT(av_state(&av), until);
}

static world_t world(int lo, int hi, float cm, float bg)
{
    world_t w;
    memset(&w, 0, sizeof w);
    w.lo = lo;
    w.hi = hi;
    w.obstacle_cm = cm;
    w.bg_cm = bg;
    return w;
}

static void real_encounters_never_rejected(void)
{
    rig_t r;
    world_t w;

    w = world(87, 93, 12.0f, 150.0f); /* narrow: servo only */
    run_world(&w, &r, AV_STATE_MONITOR, 60000);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.mission, MISSION_VERIFY_LINE);
    CHECK_INT(r.stops, 1);

    w = world(60, 120, 15.0f, 150.0f); /* wide: car turns to find edges */
    run_world(&w, &r, AV_STATE_MONITOR, 60000);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.mission, MISSION_VERIFY_LINE);

    w = world(80, 100, 12.0f, 150.0f);
    w.remove_on_stop = true; /* gone by the time it scans */
    run_world(&w, &r, AV_STATE_MONITOR, 60000);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.mission, MISSION_TRACK_LINE);

    w = world(-1000, -1000, 0.0f, 10.0f); /* boxed in */
    run_world(&w, &r, AV_STATE_FAILED, 600000);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.mission, MISSION_SAFE_STOP);
    CHECK_INT(r.service.status.stop_reasons, STOP_REASON_RECOVERY_FAILED);

    w = world(-1000, -1000, 0.0f, 150.0f);
    w.fault = true; /* sensor dead */
    run_world(&w, &r, AV_STATE_FAILED, 10000);
    CHECK_INT(r.service.status.events_rejected, 0);
    CHECK_INT(r.service.status.stop_reasons, STOP_REASON_SENSOR_CRITICAL);
}

int main(void)
{
    printf("mission bridge\n");
    RUN(full_bypass_follows_missions);
    RUN(obstacle_gone_resumes_tracking);
    RUN(rescan_reverse_and_replans_are_accepted);
    RUN(skipped_steps_are_filled_in);
    RUN(failures_latch_once);
    RUN(no_mission_restart_after_failure);
    RUN(real_encounters_never_rejected);
    printf("\n%d checks, %d failures\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
