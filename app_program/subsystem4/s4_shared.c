#include "s4_shared.h"

#include <stddef.h>

#include "s4_atomic.h"

/* Enough for a writer on the other core to finish its ~10 stores. */
#define S4_READ_ATTEMPTS 16U

/* Sequence lock: odd while the writer is updating, 0 = never published. */
static volatile uint32_t s4_snapshot_seq;
static volatile uint32_t s4_snapshot_words[S4_WORDS];

/* Odometry word and its writer-private post counter (Buddy 2's task). */
static volatile uint32_t s4_odometry_word;
static uint8_t s4_odometry_post_seq;

void s4_shared_publish(const uint32_t words[S4_WORDS])
{
    uint32_t seq = S4_LOAD32(&s4_snapshot_seq);
    uint32_t next = seq + 2U;
    unsigned i;

    if (next == 0U)
    {
        next = 2U;
    }
    S4_STORE32(&s4_snapshot_seq, seq + 1U);
    S4_FENCE();
    for (i = 0U; i < S4_WORDS; i++)
    {
        S4_STORE32(&s4_snapshot_words[i], words[i]);
    }
    S4_FENCE();
    S4_STORE32(&s4_snapshot_seq, next);
}

bool s4_shared_read(uint32_t words[S4_WORDS])
{
    unsigned attempt;
    unsigned i;

    if (words == NULL)
    {
        return false;
    }
    for (attempt = 0U; attempt < S4_READ_ATTEMPTS; attempt++)
    {
        uint32_t before = S4_LOAD32(&s4_snapshot_seq);
        uint32_t after;

        if (before == 0U)
        {
            return false;
        }
        if ((before & 1U) != 0U)
        {
            continue;
        }
        S4_FENCE();
        for (i = 0U; i < S4_WORDS; i++)
        {
            words[i] = S4_LOAD32(&s4_snapshot_words[i]);
        }
        S4_FENCE();
        after = S4_LOAD32(&s4_snapshot_seq);
        if (before == after)
        {
            return true;
        }
    }
    return false;
}

void s4_post_odometry(int32_t cumulative_mm)
{
    uint32_t word = 0U;

    s4_odometry_post_seq++;
    word = s4_put(word, S4_OD_DIST_SHIFT, S4_OD_DIST_WIDTH, (uint32_t)cumulative_mm);
    word = s4_put(word, S4_OD_SEQ_SHIFT, S4_OD_SEQ_WIDTH, s4_odometry_post_seq);
    word = s4_put(word, S4_OD_VALID_SHIFT, 1U, 1U);
    S4_STORE32(&s4_odometry_word, word);
}

void s4_post_odometry_invalid(void)
{
    S4_STORE32(&s4_odometry_word, 0U);
}

bool s4_take_odometry(
    s4_odom_reader_t *reader,
    uint8_t stale_limit,
    int16_t *delta_mm)
{
    uint32_t word = S4_LOAD32(&s4_odometry_word);
    uint16_t distance;
    uint8_t seq;

    if ((reader == NULL) || (delta_mm == NULL))
    {
        return false;
    }
    *delta_mm = 0;
    if (s4_get(word, S4_OD_VALID_SHIFT, 1U) == 0U)
    {
        reader->primed = 0U;
        return false;
    }

    distance = (uint16_t)s4_get(word, S4_OD_DIST_SHIFT, S4_OD_DIST_WIDTH);
    seq = (uint8_t)s4_get(word, S4_OD_SEQ_SHIFT, S4_OD_SEQ_WIDTH);
    if (!reader->primed)
    {
        reader->last_distance = distance;
        reader->last_seq = seq;
        reader->stale_ticks = 0U;
        reader->primed = 1U;
        return true;
    }
    if (seq == reader->last_seq)
    {
        if (reader->stale_ticks < 31U)
        {
            reader->stale_ticks++;
        }
        return reader->stale_ticks < stale_limit;
    }

    /* Modular difference: correct across the 16-bit wrap. */
    *delta_mm = (int16_t)(uint16_t)(distance - reader->last_distance);
    reader->last_distance = distance;
    reader->last_seq = seq;
    reader->stale_ticks = 0U;
    return true;
}

uint32_t s4_telemetry_pack(uint8_t *out, uint32_t capacity)
{
    uint32_t words[S4_WORDS];
    unsigned i;

    if ((out == NULL) || (capacity < S4_TLM_SIZE) || !s4_shared_read(words))
    {
        return 0U;
    }
    out[0] = (uint8_t)(s4_put(0U, S4_TLM_HDR_VERSION_SHIFT, S4_TLM_HDR_VERSION_WIDTH,
                              S4_TLM_VERSION) |
                       s4_put(0U, S4_TLM_HDR_TYPE_SHIFT, S4_TLM_HDR_TYPE_WIDTH,
                              S4_TLM_TYPE));
    for (i = 0U; i < S4_WORDS; i++)
    {
        out[1U + (4U * i)] = (uint8_t)(words[i]);
        out[2U + (4U * i)] = (uint8_t)(words[i] >> 8);
        out[3U + (4U * i)] = (uint8_t)(words[i] >> 16);
        out[4U + (4U * i)] = (uint8_t)(words[i] >> 24);
    }
    return S4_TLM_SIZE;
}

bool s4_command_decode(uint8_t byte, uint8_t *command)
{
    uint8_t code = (uint8_t)(byte & S4_MASK(S4_CMD_CODE_WIDTH));

    if ((command == NULL) || ((byte >> S4_CMD_CODE_WIDTH) != 0U) ||
        (code > S4_CMD_REPORT_RANGES))
    {
        return false;
    }
    *command = code;
    return true;
}

void s4_terrain_report_from_words(
    const uint32_t words[S4_WORDS],
    system_time_ms_t observed_ms,
    terrain_report_t *report)
{
    uint32_t status;
    uint32_t hump;

    if ((words == NULL) || (report == NULL))
    {
        return;
    }
    status = words[S4_W_STATUS];
    hump = words[S4_W_HUMP];
    report->state = (terrain_state_t)s4_get(status, S4_ST_TERRAIN_SHIFT,
                                            S4_ST_TERRAIN_WIDTH);
    report->speed_cap_mm_s =
        s4_get(status, S4_ST_CAP_SHIFT, S4_ST_CAP_WIDTH) * S4_CAP_UNIT_MM_S;
    report->estimated_height_mm =
        (int32_t)s4_get(hump, S4_HU_HEIGHT_SHIFT, S4_HU_HEIGHT_WIDTH);
    report->uncertainty_mm = s4_get(hump, S4_HU_UNC_SHIFT, S4_HU_UNC_WIDTH);
    report->estimate_valid = s4_get(hump, S4_HU_VALID_SHIFT, 1U) != 0U;
    report->observed_ms = observed_ms;
}
