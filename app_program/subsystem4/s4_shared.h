#ifndef GROUP18_S4_SHARED_H
#define GROUP18_S4_SHARED_H

/*
 * Buddy 4 data shared with other tasks, the controller and Buddy 1.
 *
 * Snapshot (Buddy 4 -> everyone): the four packed words of s4_packed.h behind
 * a sequence lock. The IMU task is the ONLY writer. Readers never write, so
 * there is no read-modify-write on shared packed data. A reader copies all
 * four words and retries if the writer was mid-update, which gives a coherent
 * snapshot across both RP2040 cores without blocking the writer.
 *
 * Odometry (Buddy 2 -> Buddy 4): one packed 32-bit word, a single aligned
 * store by its single writer.
 *
 * Portable: no kernel dependency, so the host tests exercise it. Does not
 * include <stddef.h>: the kernel headers define their own size_t.
 */
#include "../contracts/control_messages.h"
#include "s4_packed.h"

/* --- Snapshot ----------------------------------------------------------- */

/* Writer side: IMU task only. */
void s4_shared_publish(const uint32_t words[S4_WORDS]);

/*
 * Reader side, any task or handler. Returns false if no coherent copy was
 * obtained in a bounded number of attempts (e.g. called from an interrupt
 * that preempted the writer on the same core), or before the first publish.
 */
bool s4_shared_read(uint32_t words[S4_WORDS]);

/* --- Odometry input ------------------------------------------------------ */

/*
 * Buddy 2 calls this from ONE task each control cycle with the cumulative
 * travelled path distance in mm (average of both wheels, signed; reverse
 * counts down). Only the low 16 bits are kept, which is unambiguous while
 * Buddy 4 reads at least once per 32 m of travel.
 */
void s4_post_odometry(int32_t cumulative_mm);

/* Mark odometry unavailable (e.g. encoder fault). Same single caller. */
void s4_post_odometry_invalid(void);

typedef struct
{
    uint16_t last_distance;
    uint8_t last_seq;
    unsigned int primed : 1;
    unsigned int stale_ticks : 5;
} s4_odom_reader_t;

/*
 * IMU task side: distance travelled since the previous call. Returns false
 * (delta 0) when odometry is invalid, never posted, or has not changed its
 * post counter for stale_limit calls.
 */
bool s4_take_odometry(
    s4_odom_reader_t *reader,
    uint8_t stale_limit,
    int16_t *delta_mm);

/* --- MQTT payloads (for Buddy 1) ---------------------------------------- */

/*
 * Build the 17-byte binary telemetry frame from the latest snapshot:
 * byte 0 = version (3 bits) | frame type (5 bits), bytes 1..16 = words 0..3
 * little-endian. Returns the frame length, or 0 if capacity is too small or
 * no snapshot is available.
 */
uint32_t s4_telemetry_pack(uint8_t *out, uint32_t capacity);

/* Decode a received command byte into an S4_CMD_* code; false if invalid. */
bool s4_command_decode(uint8_t byte, uint8_t *command);

/* --- Compatibility with the existing shared contract (IF-03) ------------ */

/* Expand the packed snapshot into the existing terrain_report_t. */
void s4_terrain_report_from_words(
    const uint32_t words[S4_WORDS],
    system_time_ms_t observed_ms,
    terrain_report_t *report);

#endif
