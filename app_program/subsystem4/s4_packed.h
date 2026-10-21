#ifndef GROUP18_S4_PACKED_H
#define GROUP18_S4_PACKED_H

/*
 * Buddy 4 minimum-bit data formats (Change Requirement 1).
 *
 * Every field shared between tasks, sent in a message or published over MQTT
 * uses only the bits its valid range needs. Layouts are defined with explicit
 * shifts and masks (not C bit-fields) so the bit positions are fixed on every
 * compiler and can be decoded on the dashboard. DATA_REPRESENTATION.md lists
 * every field with its range and width.
 *
 * Portable: no kernel or hardware dependency.
 */
#include <stdbool.h>
#include <stdint.h>

#include "../contracts/system_types.h"

/* ---------------------------------------------------------------------
 *  Generic field helpers
 * --------------------------------------------------------------------- */
#define S4_MASK(width) ((uint32_t)((1UL << (width)) - 1UL))

static inline uint32_t s4_get(uint32_t word, unsigned shift, unsigned width)
{
    return (word >> shift) & S4_MASK(width);
}

static inline uint32_t s4_put(
    uint32_t word, unsigned shift, unsigned width, uint32_t value)
{
    return (word & ~(S4_MASK(width) << shift)) |
        ((value & S4_MASK(width)) << shift);
}

/* Sign-extend a two's-complement field of the given width. */
static inline int32_t s4_sext(uint32_t value, unsigned width)
{
    uint32_t sign = 1UL << (width - 1U);
    value &= S4_MASK(width);
    return (int32_t)(value ^ sign) - (int32_t)sign;
}

static inline int32_t s4_get_signed(uint32_t word, unsigned shift, unsigned width)
{
    return s4_sext(s4_get(word, shift, width), width);
}

/* ---------------------------------------------------------------------
 *  Enumerations, sized to their field widths below
 * --------------------------------------------------------------------- */
enum /* 3-bit motion class (Week 8 tribunal), highest priority wins */
{
    S4_MC_STATIONARY = 0,
    S4_MC_MOVING = 1,        /* steady driving */
    S4_MC_ACCELERATING = 2,
    S4_MC_BRAKING = 3,
    S4_MC_TURN_LEFT = 4,
    S4_MC_TURN_RIGHT = 5,
    S4_MC_ROUGH = 6,
    S4_MC_IMPACT = 7
};

enum /* 2-bit calibration status (Week 8 tribunal) */
{
    S4_CAL_NONE = 0,         /* never calibrated, or the last attempt failed */
    S4_CAL_RUNNING = 1,      /* level calibration in progress: keep still */
    S4_CAL_LEVEL = 2,        /* level calibrated, magnetometer not */
    S4_CAL_LEVEL_MAG = 3     /* level and magnetometer calibrated */
};

enum /* 2-bit collision direction (side of the car that was hit) */
{
    S4_HIT_FRONT = 0,
    S4_HIT_REAR = 1,
    S4_HIT_LEFT = 2,
    S4_HIT_RIGHT = 3
};

/*
 * 7-bit Buddy 4 detail mask: one bit per condition, never separate ints.
 * The team-wide fault word (contracts/fault_mask.h) carries the summary bits
 * other buddies act on; this mask keeps the detail for telemetry.
 */
enum
{
    S4_FAULT_I2C = 1U << 0,        /* accelerometer bus error */
    S4_FAULT_MAG = 1U << 1,        /* magnetometer absent or failing */
    S4_FAULT_STALE = 1U << 2,      /* no fresh accelerometer sample */
    S4_FAULT_UNCAL = 1U << 3,      /* no valid level calibration */
    S4_FAULT_SATURATED = 1U << 4,  /* an axis reached full scale */
    S4_FAULT_TILT = 1U << 5,       /* pitch or roll beyond the limit */
    S4_FAULT_COLLISION = 1U << 6   /* collision latched until cleared */
};
#define S4_FAULT_WIDTH 7U
#define S4_FAULT_BLOCKING (S4_FAULT_I2C | S4_FAULT_STALE)

/* ---------------------------------------------------------------------
 *  Shared snapshot: four 32-bit words published together (seqlock)
 * --------------------------------------------------------------------- */
#define S4_WORDS 4U
#define S4_W_STATUS 0U
#define S4_W_ATTITUDE 1U
#define S4_W_MOTION 2U
#define S4_W_HUMP 3U

/*
 * Word 0: status. DATA_VALID is the single validity bit for the whole
 * message (Week 8 tribunal): set when level calibration is valid and the
 * accelerometer has neither a bus nor a stale fault. No field reserves an
 * "invalid" value. Heading is meaningful when DATA_VALID is set and the
 * MAG fault bit is clear. CLAMPED is set when any field of this message had
 * to be saturated to fit its width (explicit out-of-range status).
 */
#define S4_ST_TERRAIN_SHIFT 0U   /* terrain_state_t 0..6 */
#define S4_ST_TERRAIN_WIDTH 3U
#define S4_ST_CAP_SHIFT 3U       /* speed cap, 10 mm/s units, 0 = none */
#define S4_ST_CAP_WIDTH 7U
#define S4_ST_MOTION_SHIFT 10U   /* S4_MC_* */
#define S4_ST_MOTION_WIDTH 3U
#define S4_ST_FAULT_SHIFT 13U
#define S4_ST_FAULT_WIDTH S4_FAULT_WIDTH
#define S4_ST_CAL_SHIFT 20U      /* S4_CAL_* */
#define S4_ST_CAL_WIDTH 2U
#define S4_ST_ODOM_SHIFT 22U     /* distance from Buddy 2 odometry */
#define S4_ST_HUMP_SHIFT 23U     /* on a hump now: Buddy 3 pauses sensing */
#define S4_ST_COLL_SHIFT 24U     /* collision impact in progress */
#define S4_ST_COLLCNT_SHIFT 25U  /* collision counter mod 8 */
#define S4_ST_COLLCNT_WIDTH 3U
#define S4_ST_COLLDIR_SHIFT 28U
#define S4_ST_COLLDIR_WIDTH 2U
#define S4_ST_CLAMPED_SHIFT 30U  /* a field in this message was saturated */
#define S4_ST_VALID_SHIFT 31U    /* DATA_VALID */

/* Word 1: attitude */
#define S4_AT_PITCH_SHIFT 0U     /* signed, 0.5 deg units, +nose up */
#define S4_AT_PITCH_WIDTH 9U
#define S4_AT_ROLL_SHIFT 9U      /* signed, 0.5 deg units, +right side down */
#define S4_AT_ROLL_WIDTH 9U
#define S4_AT_HEADING_SHIFT 18U  /* 0..359 deg, magnetic */
#define S4_AT_HEADING_WIDTH 9U
#define S4_AT_SEQ_SHIFT 27U      /* sample counter mod 32, freshness */
#define S4_AT_SEQ_WIDTH 5U

/* Word 2: motion. Bits 31..30 reserved (zero). */
#define S4_MO_TURNRATE_SHIFT 0U  /* signed, 2 deg/s units: +/-250 deg/s */
#define S4_MO_TURNRATE_WIDTH 8U
#define S4_MO_LIVEH_SHIFT 8U     /* height in the current hump, mm */
#define S4_MO_LIVEH_WIDTH 8U
#define S4_MO_SHOCK_SHIFT 16U    /* last collision magnitude, 16 mg units */
#define S4_MO_SHOCK_WIDTH 8U
#define S4_MO_VIB_SHIFT 24U      /* vibration level, 8 mg units */
#define S4_MO_VIB_WIDTH 6U
#define S4_MO_RESERVED_SHIFT 30U

/* Word 3: last completed hump */
#define S4_HU_HEIGHT_SHIFT 0U    /* peak height 0..255 mm */
#define S4_HU_HEIGHT_WIDTH 8U
#define S4_HU_UNC_SHIFT 8U       /* uncertainty 0..63 mm */
#define S4_HU_UNC_WIDTH 6U
#define S4_HU_PITCH_SHIFT 14U    /* peak |pitch| 0..63 deg */
#define S4_HU_PITCH_WIDTH 6U
#define S4_HU_LEN_SHIFT 20U      /* path length 0..255 cm */
#define S4_HU_LEN_WIDTH 8U
#define S4_HU_VALID_SHIFT 28U
#define S4_HU_SEQ_SHIFT 29U      /* result counter mod 8, new-result check */
#define S4_HU_SEQ_WIDTH 3U
#define S4_HU_RESULT_WIDTH 29U   /* bits 0..28 carried in a HUMP event */

/* Conversions for packed physical units. */
#define S4_CAP_UNIT_MM_S 10U
#define S4_TURNRATE_UNIT_DPS 2U
#define S4_SHOCK_UNIT_MG 16U
#define S4_VIB_UNIT_MG 8U

/* ---------------------------------------------------------------------
 *  Inter-task event message: one 32-bit word through a message buffer
 * --------------------------------------------------------------------- */
typedef uint32_t s4_event_msg_t;

#define S4_EV_TYPE_SHIFT 0U
#define S4_EV_TYPE_WIDTH 3U
#define S4_EV_PAYLOAD_SHIFT 3U

enum /* 3-bit event type */
{
    S4_EV_TERRAIN = 0,   /* state 3b @3, cap 7b @6 */
    S4_EV_HUMP = 1,      /* hump word bits 0..28 @3 */
    S4_EV_COLLISION = 2, /* direction 2b @3, magnitude 8b @5 */
    S4_EV_TURN = 3,      /* right 1b @3, angle deg 9b @4 */
    S4_EV_FAULT = 4,     /* fault mask 7b @3 */
    S4_EV_CAL = 5        /* kind 1b @3 (0 accel, 1 mag), ok 1b @4 */
};

#define S4_EV_TERRAIN_STATE_SHIFT 3U
#define S4_EV_TERRAIN_CAP_SHIFT 6U
#define S4_EV_COLL_DIR_SHIFT 3U
#define S4_EV_COLL_MAG_SHIFT 5U
#define S4_EV_TURN_RIGHT_SHIFT 3U
#define S4_EV_TURN_ANGLE_SHIFT 4U
#define S4_EV_TURN_ANGLE_WIDTH 9U
#define S4_EV_FAULT_SHIFT 3U
#define S4_EV_CAL_KIND_SHIFT 3U
#define S4_EV_CAL_OK_SHIFT 4U

/* ---------------------------------------------------------------------
 *  Odometry input word written by Buddy 2 (single writer)
 * --------------------------------------------------------------------- */
#define S4_OD_DIST_SHIFT 0U      /* cumulative distance mod 65536 mm */
#define S4_OD_DIST_WIDTH 16U
#define S4_OD_SEQ_SHIFT 16U      /* post counter mod 256 */
#define S4_OD_SEQ_WIDTH 8U
#define S4_OD_VALID_SHIFT 24U

/* ---------------------------------------------------------------------
 *  MQTT payloads (compact binary, little-endian)
 * --------------------------------------------------------------------- */
/* Telemetry frame: 1 header byte + the 4 snapshot words. */
#define S4_TLM_VERSION 1U
#define S4_TLM_TYPE 4U           /* frame type 4 = Buddy 4 terrain */
#define S4_TLM_HDR_VERSION_SHIFT 0U
#define S4_TLM_HDR_VERSION_WIDTH 3U
#define S4_TLM_HDR_TYPE_SHIFT 3U
#define S4_TLM_HDR_TYPE_WIDTH 5U
#define S4_TLM_SIZE (1U + (4U * S4_WORDS))

/* Command byte: 3-bit command code (0..5 used), bits 7..3 must be zero. */
#define S4_CMD_CODE_WIDTH 3U
enum
{
    S4_CMD_NONE = 0,
    S4_CMD_RECALIBRATE = 1,
    S4_CMD_CLEAR_LATCHED = 2,
    S4_CMD_MAG_CAL_START = 3,
    S4_CMD_MAG_CAL_STOP = 4,
    S4_CMD_REPORT_RANGES = 5
};

/* ---------------------------------------------------------------------
 *  Compile-time checks: each word's fields end within 32 bits and every
 *  enum fits its field.
 * --------------------------------------------------------------------- */
_Static_assert(S4_ST_MOTION_SHIFT + S4_ST_MOTION_WIDTH == S4_ST_FAULT_SHIFT,
               "motion field overlaps");
_Static_assert(S4_ST_FAULT_SHIFT + S4_ST_FAULT_WIDTH == S4_ST_CAL_SHIFT,
               "fault field overlaps");
_Static_assert(S4_ST_CAL_SHIFT + S4_ST_CAL_WIDTH == S4_ST_ODOM_SHIFT,
               "calibration field overlaps");
_Static_assert(S4_ST_COLLDIR_SHIFT + S4_ST_COLLDIR_WIDTH == S4_ST_CLAMPED_SHIFT,
               "status fields overlap");
_Static_assert(S4_ST_VALID_SHIFT == 31U, "status word layout");
_Static_assert(S4_AT_SEQ_SHIFT + S4_AT_SEQ_WIDTH == 32U, "attitude word layout");
_Static_assert(S4_MO_VIB_SHIFT + S4_MO_VIB_WIDTH == S4_MO_RESERVED_SHIFT,
               "motion word layout");
_Static_assert(S4_MC_IMPACT < (1 << S4_ST_MOTION_WIDTH), "motion class fits 3 bits");
_Static_assert(S4_HU_SEQ_SHIFT + S4_HU_SEQ_WIDTH == 32U, "hump word layout");
_Static_assert(S4_HU_VALID_SHIFT + 1U == S4_HU_RESULT_WIDTH, "hump result width");
_Static_assert(S4_EV_PAYLOAD_SHIFT + S4_HU_RESULT_WIDTH == 32U, "hump event fits");
_Static_assert(TERRAIN_INVALID < (1 << S4_ST_TERRAIN_WIDTH), "terrain fits 3 bits");
_Static_assert(S4_FAULT_COLLISION < (1U << S4_FAULT_WIDTH), "faults fit 7 bits");
_Static_assert(S4_EV_CAL < (1 << S4_EV_TYPE_WIDTH), "event type fits 3 bits");
_Static_assert(S4_CMD_REPORT_RANGES < (1 << S4_CMD_CODE_WIDTH), "command fits 3 bits");
_Static_assert(sizeof(s4_event_msg_t) == 4U, "event message is one word");

#endif
