#ifndef GROUP18_S4_IMU_H
#define GROUP18_S4_IMU_H

/*
 * Buddy 4 public interface: IMU, terrain, hump, motion and turn monitoring.
 * Other subsystems include "../subsystem4/s4_imu.h".
 *
 * Outputs (all minimum-bit packed, see s4_packed.h / DATA_REPRESENTATION.md)
 *   - Snapshot, any task:   s4_shared_read(words)       (lock-free seqlock)
 *   - IF-03 contract form:  s4_terrain_report_from_words()
 *   - Shared fault word:    FAULT_B4_* bits in contracts/fault_mask.h
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

IMPORT ER s4_imu_init(void);
IMPORT ER s4_imu_start(void);

/* Post request bits (S4_REQ_*). Safe from tasks and handlers. */
IMPORT ER s4_request(UINT bits);

/* Apply a validated MQTT command byte (S4_CMD_*). Returns E_PAR if invalid. */
IMPORT ER s4_command(UB byte);

#if APP_TEST
/* On-target checks; car must be still and level. */
IMPORT void s4_imu_selftest(void);
#endif

#endif
