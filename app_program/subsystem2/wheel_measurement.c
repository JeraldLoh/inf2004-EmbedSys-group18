#include "wheel_measurement.h"
#include <stddef.h>
#include <string.h>

bool wheel_measurement_init(wheel_measurement_t *m,
    const wheel_measurement_config_t *c)
{
    if ((m == NULL) || (c == NULL) || (c->diameter_mm == 0U) ||
        (c->diameter_mm > 1000U) || (c->counts_per_rev[0] == 0U) ||
        (c->counts_per_rev[1] == 0U) || (c->counts_per_rev[0] > 100000U) ||
        (c->counts_per_rev[1] > 100000U) || (c->speed_window_us < 1000U) ||
        (c->speed_window_us > 1000000U) || (c->max_gap_us < 1000U) ||
        (c->max_gap_us > 1000000U) || (c->max_edge_rate == 0U) ||
        (c->max_edge_rate > 100000U) || c->max_batched_edge_permille>50U ||
        c->max_batches_per_100ms>2U ||
        ((c->max_batched_edge_permille==0U)!=(c->max_batches_per_100ms==0U))) { return false; }
    (void)memset(m, 0, sizeof(*m));
    m->config = *c;
    return true;
}

/* Rebase acquisition only: keep lifetime travel, never invent the lost
 * interval. Quality remains invalid until a complete subsequent good window. */
const char *wheel_reject_name(wheel_reject_reason_t reason)
{
    switch (reason) {
    case WHEEL_REJECT_ACQUISITION: return "ACQUISITION_INVALID";
    case WHEEL_REJECT_RESET: return "COUNTER_RESET";
    case WHEEL_REJECT_GAP: return "SAMPLE_INTERVAL";
    case WHEEL_REJECT_EDGE_RATE: return "EDGE_RATE";
    case WHEEL_REJECT_BATCH_COUNT: return "BATCH_COUNT";
    case WHEEL_REJECT_BATCH_RATE: return "BATCH_RATE";
    case WHEEL_REJECT_BATCH_FRACTION: return "BATCH_FRACTION";
    case WHEEL_REJECT_TRAVEL_RANGE: return "TRAVEL_RANGE";
    case WHEEL_REJECT_SPEED_RANGE: return "SPEED_RANGE";
    default: return "NONE";
    }
}

static motion_status_t unavailable(wheel_measurement_t *m, const motion_encoder_snapshot_t *s,
    wheel_reject_reason_t reason, uint8_t wheel_mask)
{
    m->reject.reason = reason;
    m->reject.wheel_mask = wheel_mask;
    m->sample.speed_valid=false; m->sample.signed_speed_valid=false;
    m->sample.speed_um_s[0]=0U; m->sample.speed_um_s[1]=0U;
    m->window_us=0U;
    memset(m->window_um,0,sizeof(m->window_um));
    memset(m->window_edges,0,sizeof(m->window_edges));
    memset(m->window_batches,0,sizeof(m->window_batches));
    if(s->acquisition_valid) {
        m->previous=*s; m->sample.acquired_us=s->acquired_us;
        m->sample.reset_epoch=s->reset_epoch;
        m->window_direction_epoch=s->direction_epoch; m->direction_changed=false;
        m->primed=true;
    } else { m->primed=false; }
    return MOTION_UNAVAILABLE;
}

motion_status_t wheel_measurement_update(wheel_measurement_t *m,
    const motion_encoder_snapshot_t *s)
{
    uint32_t dt;
    uint32_t delta[2], batches[2];
    uint64_t travel_delta[2], remainder[2], speed[2] = {0U, 0U};
    uint64_t max_delta;
    bool window_complete;
    unsigned i;
    if ((m == NULL) || (s == NULL) || (m->config.max_edge_rate == 0U))
    { return MOTION_INVALID; }
    m->reject = (wheel_reject_t){0};
    if (!s->acquisition_valid) { return unavailable(m,s,WHEEL_REJECT_ACQUISITION,0U); }
    if (!m->primed)
    {
        m->previous = *s;
        m->window_direction_epoch = s->direction_epoch;
        m->sample.reset_epoch = s->reset_epoch;
        m->sample.acquired_us = s->acquired_us;
        m->primed = true;
        return MOTION_OK;
    }
    dt = s->acquired_us - m->previous.acquired_us;
    delta[0] = s->left_count - m->previous.left_count;
    delta[1] = s->right_count - m->previous.right_count;
    batches[0]=s->coalesced_batches[0]-m->previous.coalesced_batches[0];
    batches[1]=s->coalesced_batches[1]-m->previous.coalesced_batches[1];
    m->reject.interval_us = dt;
    memcpy(m->reject.edges, delta, sizeof(delta));
    memcpy(m->reject.batches, batches, sizeof(batches));
    for (i=0U;i<2U;++i) {
        m->reject.window_edges[i] = m->window_edges[i] + delta[i];
        m->reject.window_batches[i] = m->window_batches[i] + batches[i];
    }
    max_delta = ((uint64_t)m->config.max_edge_rate * dt + 999999U) / 1000000U + 2U;
    if (s->reset_epoch != m->previous.reset_epoch)
    { return unavailable(m,s,WHEEL_REJECT_RESET,0U); }
    if (dt == 0U || dt > m->config.max_gap_us)
    { return unavailable(m,s,WHEEL_REJECT_GAP,0U); }
    if (delta[0] > max_delta || delta[1] > max_delta)
    { return unavailable(m,s,WHEEL_REJECT_EDGE_RATE,
        (uint8_t)((delta[0] > max_delta ? 1U : 0U) | (delta[1] > max_delta ? 2U : 0U))); }
    window_complete = m->window_us + dt >= m->config.speed_window_us;
    for(i=0U;i<2U;++i) {
        /* Every batch represents TWO distinct latched polarities. It cannot
         * exceed its observed edge delta, even across counter wrap. */
        if((uint64_t)batches[i]*2U>delta[i] ||
            (batches[i]!=0U && m->config.max_batches_per_100ms==0U))
        { return unavailable(m,s,WHEEL_REJECT_BATCH_COUNT,(uint8_t)(1U<<i)); }
        if((uint64_t)m->reject.window_batches[i]*100000U >
            (uint64_t)m->config.max_batches_per_100ms*m->config.speed_window_us)
        { return unavailable(m,s,WHEEL_REJECT_BATCH_RATE,(uint8_t)(1U<<i)); }
        if(window_complete && (uint64_t)m->reject.window_batches[i]*2000U >
            (uint64_t)m->config.max_batched_edge_permille*m->reject.window_edges[i])
        { return unavailable(m,s,WHEEL_REJECT_BATCH_FRACTION,(uint8_t)(1U<<i)); }
        uint64_t denominator = (uint64_t)m->config.counts_per_rev[i] * 1000U;
        uint64_t numerator = (uint64_t)delta[i] * m->config.diameter_mm * 3141593U + m->remainder[i];
        travel_delta[i] = numerator / denominator;
        remainder[i] = numerator % denominator;
        if (UINT64_MAX - m->sample.travel_um[i] < travel_delta[i])
        { return unavailable(m,s,WHEEL_REJECT_TRAVEL_RANGE,(uint8_t)(1U<<i)); }
        if (window_complete) {
            speed[i] = (m->window_um[i] + travel_delta[i]) * 1000000U / (m->window_us + dt);
            if (speed[i] > 2000000U)
            { return unavailable(m,s,WHEEL_REJECT_SPEED_RANGE,(uint8_t)(1U<<i)); }
        }
    }
    if (s->direction_epoch != m->window_direction_epoch)
    { m->direction_changed = true; m->sample.signed_speed_valid = false; }
    for (i = 0U; i < 2U; ++i)
    {
        m->remainder[i] = remainder[i];
        m->sample.travel_um[i] += travel_delta[i];
        m->window_um[i] += travel_delta[i];
        m->window_edges[i] = m->reject.window_edges[i];
        m->window_batches[i] = m->reject.window_batches[i];
    }
    m->window_us += dt;
    m->sample.command_sign[0] = s->left_command_sign;
    m->sample.command_sign[1] = s->right_command_sign;
    if (window_complete)
    {
        for (i = 0U; i < 2U; ++i)
        {
            m->sample.speed_um_s[i] = (uint32_t)speed[i];
            m->window_um[i] = 0U;
            m->sample.window_batches[i]=m->window_batches[i];
            m->window_batches[i]=0U; m->window_edges[i]=0U;
        }
        m->sample.speed_valid = true;
        m->sample.signed_speed_valid = !m->direction_changed && s->edge_times_valid &&
            m->sample.window_batches[0]==0U && m->sample.window_batches[1]==0U &&
            ((s->command_basis_valid && s->left_command_sign != 0 && s->right_command_sign != 0) ||
             (m->sample.speed_um_s[0] == 0U && m->sample.speed_um_s[1] == 0U));
        m->window_us = 0U;
        m->direction_changed = false;
        m->window_direction_epoch = s->direction_epoch;
    }
    m->sample.acquired_us = s->acquired_us;
    m->sample.reset_epoch = s->reset_epoch;
    m->previous = *s;
    return MOTION_OK;
}
