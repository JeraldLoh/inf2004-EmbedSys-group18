#ifndef GROUP18_S4_IMU_H
#define GROUP18_S4_IMU_H

/*
 * Buddy 4 public interface: IMU, terrain, hump, motion and turn monitoring.
 * Other subsystems include "../subsystem4/s4_imu.h".
 *
 * Outputs (all minimum-bit packed, see s4_packed.h / DATA_REPRESENTATION.md)
 *   - Snapshot, any task:   s4_shared_read(words)       (lock-free seqlock)
 *   - IF-03 contract form:  s4_terrain_report_from_words()
 *   - Fault evidence:       snapshot detail mask + S4_EV_FAULT events; the
 *                           controller owns contracts/hardware_faults.h
 *   - Discrete events:      s4_mbf_events, one 4-byte s4_event_msg_t each,
 *                           intended for the Vehicle Controller (single reader)
 *   - MQTT telemetry:       s4_telemetry_pack() -> 17-byte binary frame
 * Inputs
 *   - Buddy 2 odometry:     s4_post_odometry(cumulative_mm), every cycle
 *   - Requests/commands:    s4_request(S4_REQ_*) or s4_command(byte)
 *
 * Lifecycle: s4_imu_init() creates every Buddy 4 kernel object; s4_imu_start()
 * starts the task and the cyclic handler. They are meant to be called from
 * subsystem4_init() / subsystem4_start() (all inits before any start).
 */
#include "../app.h"
#include "s4_shared.h"

/* Request bits, set from any task or handler (tk_set_flg is atomic). */
#define S4_REQ_RECALIBRATE 0x0001U
#define S4_REQ_CLEAR_LATCHED 0x0002U
#define S4_REQ_MAG_CAL_START 0x0004U
#define S4_REQ_MAG_CAL_STOP 0x0008U
#define S4_REQ_REPORT_RANGES 0x0010U  /* print observed ranges + timing, reset */
#define S4_REQ_ALL 0x001FU

/* Kernel objects owned by Buddy 4. Valid after s4_imu_init() succeeds. */
IMPORT ID s4_flg_request;   /* request bits + private tick bit */
IMPORT ID s4_mbf_events;    /* 4-byte events, depth S4_EVENT_QUEUE_DEPTH */

/* Diagnostics, written only by the IMU task. */
IMPORT UW s4_events_dropped;
IMPORT UW s4_bus_errors;
IMPORT UW s4_sample_time_us;        /* last acquire+estimate time */
IMPORT UW s4_sample_time_worst_us;  /* worst since the last range report */

/*
 * Liveness evidence. s4_loop_count rises every time the task wakes (about
 * 50 per second); s4_stage is the step it is in now, so a task that stops
 * shows where it stopped. Written only by the IMU task.
 */
enum
{
    S4_STAGE_START = 0,  /* before the first wait */
    S4_STAGE_WAIT,       /* waiting for the 20 ms tick (normal when idle) */
    S4_STAGE_REQUEST,    /* handling request bits */
    S4_STAGE_OPEN,       /* opening the I2C device / sensor setup */
    S4_STAGE_ACCEL,      /* I2C accelerometer read */
    S4_STAGE_MAG,        /* I2C magnetometer read */
    S4_STAGE_ODOM,       /* Buddy 2 odometry callback */
    S4_STAGE_ESTIMATE,   /* estimator step */
    S4_STAGE_PUBLISH,    /* snapshot publish, failure tracking, cal retry */
    S4_STAGE_DISPATCH,   /* forwarding events */
    S4_STAGE_LOG,        /* printing an event to the console */
    S4_STAGE_COUNT
};
IMPORT volatile UW s4_loop_count;
IMPORT volatile UW s4_accel_samples;   /* good accelerometer reads */
IMPORT volatile UB s4_stage;

/* Print the IMU task's liveness and kernel state (one line). Any task. */
IMPORT void s4_print_task_state(void);

/*
 * Optional inputs, bound by the integration code before s4_imu_start().
 * read_odometry_mm: cumulative signed path distance in mm (average of both
 * wheels, reverse counts down); false when unavailable. Called by the IMU
 * task once per period, so it is the only writer of the odometry word.
 */
typedef struct
{
    bool (*read_odometry_mm)(int32_t *cumulative_mm);
} s4_bindings_t;

/* Persistent sensor failures, S4_FAILURE_* bits. Set when the evidence has
 * lasted S4_FAULT_PERSIST_MS, then latched until reset. Any task or core. */
#define S4_FAILURE_IMU 0x1U   /* accelerometer unreachable or stale */
#define S4_FAILURE_MAG 0x2U   /* magnetometer missing or failing */

IMPORT ER s4_imu_init(void);
IMPORT ER s4_imu_bind(const s4_bindings_t *bindings);
IMPORT ER s4_imu_start(void);
IMPORT UW s4_sensor_failures(void);

/* Post request bits (S4_REQ_*). Safe from tasks and handlers. */
IMPORT ER s4_request(UINT bits);

/* Apply a validated MQTT command byte (S4_CMD_*). Returns E_PAR if invalid. */
IMPORT ER s4_command(UB byte);

#if APP_TEST
/* On-target checks; car must be still and level. */
IMPORT void s4_imu_selftest(void);
#endif

#endif
