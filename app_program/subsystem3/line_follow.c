#include "line_follow.h"

#include <stddef.h>

/* 2-of-3 majority vote over the last three samples. A single noisy sample
 * cannot flip the result; the cost is at most 2 samples (2 ms) of delay. */
static bool majority(uint8_t *history, bool sample)
{
    uint8_t bits;

    *history = (uint8_t)(((*history << 1) | (sample ? 1U : 0U)) & 0x07U);
    bits = *history;
    return ((bits & 1U) + ((bits >> 1) & 1U) + ((bits >> 2) & 1U)) >= 2U;
}

static void set_state(line_follow_t *lf, line_state_t state, uint32_t now_ms)
{
    if (lf->status.state != (uint8_t)state)
    {
        lf->previous_state = lf->status.state;
        lf->status.state = (uint8_t)state;
        lf->state_since_ms = now_ms;
    }
}

static int8_t clamp_steer(int32_t value)
{
    if (value > LINE_STEER_MAX)
    {
        return (int8_t)LINE_STEER_MAX;
    }
    if (value < -LINE_STEER_MAX)
    {
        return (int8_t)(-LINE_STEER_MAX);
    }
    return (int8_t)value;
}

void line_follow_reset(line_follow_t *lf)
{
    line_status_t cleared = {0};

    if (lf == NULL)
    {
        return;
    }
    lf->status = cleared;
    lf->history_left = 0U;
    lf->history_right = 0U;
    lf->previous_state = (uint8_t)LINE_STATE_CENTRED;
    lf->both_black_since_ms = 0U;
    lf->state_since_ms = 0U;
    lf->both_black_active = false;
    lf->junction_reported = false;
    lf->probing = false;
    lf->probe_saw_line = false;
}

line_status_t line_follow_update(
    line_follow_t *lf,
    bool left_raw,
    bool right_raw,
    uint32_t now_ms)
{
    line_status_t empty = {0};
    bool left_black;
    bool right_black;

    if (lf == NULL)
    {
        return empty;
    }

    left_black = majority(&lf->history_left, left_raw);
    right_black = majority(&lf->history_right, right_raw);
    lf->status.left_black = left_black ? 1U : 0U;
    lf->status.right_black = right_black ? 1U : 0U;

    if (lf->status.state == (uint8_t)LINE_STATE_REACQUIRE)
    {
        if (!left_black && !right_black)
        {
            return lf->status;           /* keep searching */
        }
        /* Found it. If we hit it square-on (both black), this is the line
         * itself, not a junction, so suppress a junction report. */
        lf->junction_reported = left_black && right_black;
        lf->both_black_active = left_black && right_black;
        lf->both_black_since_ms = now_ms;
        set_state(lf, LINE_STATE_CENTRED, now_ms);
    }

    if (left_black && right_black)
    {
        /* Debounce: a short both-black blip must not look like a junction. */
        if (!lf->both_black_active)
        {
            lf->both_black_active = true;
            lf->both_black_since_ms = now_ms;
        }
        if (!lf->junction_reported &&
            ((now_ms - lf->both_black_since_ms) >= LINE_JUNCTION_CONFIRM_MS))
        {
            set_state(lf, LINE_STATE_JUNCTION, now_ms);
        }
        /* Until confirmed, keep the previous steering state. */
        return lf->status;
    }

    /* Leaving the crossbar ends this junction; the next one can report. */
    lf->both_black_active = false;
    lf->junction_reported = false;

    if (left_black)
    {
        set_state(lf, LINE_STATE_STEER_LEFT, now_ms);
        lf->status.last_side = (uint8_t)LINE_SIDE_LEFT;
    }
    else if (right_black)
    {
        set_state(lf, LINE_STATE_STEER_RIGHT, now_ms);
        lf->status.last_side = (uint8_t)LINE_SIDE_RIGHT;
    }
    else
    {
        set_state(lf, LINE_STATE_CENTRED, now_ms);
    }
    return lf->status;
}

int8_t line_follow_steer(const line_follow_t *lf, uint32_t now_ms)
{
    uint32_t age;
    int32_t magnitude;

    if (lf == NULL)
    {
        return 0;
    }
    age = now_ms - lf->state_since_ms;
    magnitude = LINE_STEER_BASE + (int32_t)(age / LINE_STEER_RAMP_MS);

    switch ((line_state_t)lf->status.state)
    {
        case LINE_STATE_STEER_LEFT:
            return clamp_steer(-magnitude);
        case LINE_STATE_STEER_RIGHT:
            return clamp_steer(magnitude);
        case LINE_STATE_CENTRED:
            /* Damping after a correction: steer slightly the other way. */
            if (age < LINE_DAMP_MS)
            {
                if (lf->previous_state == (uint8_t)LINE_STATE_STEER_LEFT)
                {
                    return (int8_t)LINE_DAMP_STEER;
                }
                if (lf->previous_state == (uint8_t)LINE_STATE_STEER_RIGHT)
                {
                    return (int8_t)(-LINE_DAMP_STEER);
                }
            }
            return 0;
        case LINE_STATE_REACQUIRE:
            if (lf->status.last_side == (uint8_t)LINE_SIDE_LEFT)
            {
                return (int8_t)(-LINE_REACQUIRE_STEER);
            }
            if (lf->status.last_side == (uint8_t)LINE_SIDE_RIGHT)
            {
                return (int8_t)LINE_REACQUIRE_STEER;
            }
            return 0;                    /* no idea: drive straight */
        default:
            return 0;                    /* junction: controller decides */
    }
}

uint32_t line_follow_state_age_ms(const line_follow_t *lf, uint32_t now_ms)
{
    return (lf == NULL) ? 0U : (now_ms - lf->state_since_ms);
}

void line_follow_set_barcode_active(line_follow_t *lf, bool active)
{
    if (lf != NULL)
    {
        lf->status.barcode_active = active ? 1U : 0U;
    }
}

void line_follow_reacquire_start(
    line_follow_t *lf,
    line_side_t side_hint,
    uint32_t now_ms)
{
    if (lf == NULL)
    {
        return;
    }
    if (side_hint != LINE_SIDE_NONE)
    {
        lf->status.last_side = (uint8_t)side_hint;
    }
    lf->history_left = 0U;
    lf->history_right = 0U;
    lf->both_black_active = false;
    set_state(lf, LINE_STATE_REACQUIRE, now_ms);
}

bool line_follow_reacquiring(const line_follow_t *lf)
{
    return (lf != NULL) && (lf->status.state == (uint8_t)LINE_STATE_REACQUIRE);
}

bool line_follow_take_junction(
    line_follow_t *lf,
    junction_observation_t *observation,
    uint32_t now_ms)
{
    if ((lf == NULL) || (observation == NULL) ||
        (lf->status.state != (uint8_t)LINE_STATE_JUNCTION) ||
        lf->junction_reported)
    {
        return false;
    }

    lf->junction_reported = true;
    /* The barcode before this junction already says which exit to take, and
     * the track is laid out so that exit exists. Two straddling sensors
     * cannot see the branches themselves, so we report every exit as
     * available and let the barcode decide. (U-turn is always allowed by the
     * controller.) The no-barcode case uses the probe instead. */
    observation->candidate = true;
    observation->confirmed = true;
    observation->left_available = true;
    observation->right_available = true;
    observation->straight_available = true;
    observation->confidence_percent = 80U;
    observation->observed_ms = now_ms;
    return true;
}

void line_follow_probe_start(line_follow_t *lf)
{
    if (lf == NULL)
    {
        return;
    }
    lf->probing = true;
    lf->probe_saw_line = false;
}

void line_follow_probe_sample(line_follow_t *lf, bool left_black, bool right_black)
{
    (void)left_black;
    if ((lf != NULL) && lf->probing && right_black)
    {
        lf->probe_saw_line = true;   /* line ahead swung under right sensor */
    }
}

junction_type_t line_follow_probe_finish(line_follow_t *lf)
{
    if ((lf == NULL) || !lf->probing)
    {
        return JUNCTION_TYPE_UNKNOWN;
    }
    lf->probing = false;
    return lf->probe_saw_line ? JUNCTION_TYPE_CROSS : JUNCTION_TYPE_T;
}

route_command_t junction_default_route(junction_type_t type)
{
    switch (type)
    {
        case JUNCTION_TYPE_CROSS: return ROUTE_COMMAND_STRAIGHT;
        case JUNCTION_TYPE_T: return ROUTE_COMMAND_RIGHT;
        default: return ROUTE_COMMAND_NONE;   /* unknown shape: controller stops */
    }
}

void junction_observation_for_type(
    junction_observation_t *observation,
    junction_type_t type,
    uint32_t now_ms)
{
    if (observation == NULL)
    {
        return;
    }
    observation->candidate = true;
    observation->confirmed = (type != JUNCTION_TYPE_UNKNOWN);
    observation->left_available = true;
    observation->right_available = true;
    observation->straight_available = (type == JUNCTION_TYPE_CROSS);
    observation->confidence_percent = (type == JUNCTION_TYPE_UNKNOWN) ? 0U : 80U;
    observation->observed_ms = now_ms;
}
