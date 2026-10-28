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
        (c->max_edge_rate > 100000U)) { return false; }
    (void)memset(m, 0, sizeof(*m));
    m->config = *c;
    return true;
}

motion_status_t wheel_measurement_update(wheel_measurement_t *m,
    const motion_encoder_snapshot_t *s)
{
    uint32_t dt;
    uint32_t delta[2];
    uint64_t max_delta;
    unsigned i;
    if ((m == NULL) || (s == NULL) || (m->config.max_edge_rate == 0U))
    { return MOTION_INVALID; }
    if (!s->acquisition_valid || s->coalesced_edges_seen)
    { m->sample.speed_valid = false; return MOTION_UNAVAILABLE; }
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
    max_delta = ((uint64_t)m->config.max_edge_rate * dt + 999999U) / 1000000U + 2U;
    if ((s->reset_epoch != m->previous.reset_epoch) ||
        (dt == 0U) || (dt > m->config.max_gap_us) ||
        ((uint64_t)delta[0] > max_delta) || ((uint64_t)delta[1] > max_delta))
    { m->sample.speed_valid = false; return MOTION_UNAVAILABLE; }
    if (s->direction_epoch != m->window_direction_epoch)
    { m->direction_changed = true; m->sample.signed_speed_valid = false; }
    for (i = 0U; i < 2U; ++i)
    {
        uint64_t denominator = (uint64_t)m->config.counts_per_rev[i] * 1000U;
        uint64_t numerator = (uint64_t)delta[i] * m->config.diameter_mm * 3141593U + m->remainder[i];
        uint64_t um = numerator / denominator;
        m->remainder[i] = numerator % denominator;
        if (UINT64_MAX - m->sample.travel_um[i] < um)
        { m->sample.speed_valid = false; return MOTION_UNAVAILABLE; }
        m->sample.travel_um[i] += um;
        m->window_um[i] += um;
    }
    m->window_us += dt;
    m->sample.command_sign[0] = s->left_command_sign;
    m->sample.command_sign[1] = s->right_command_sign;
    if (m->window_us >= m->config.speed_window_us)
    {
        for (i = 0U; i < 2U; ++i)
        {
            uint64_t speed = m->window_um[i] * 1000000U / m->window_us;
            if (speed > 2000000U)
            { m->sample.speed_valid = false; return MOTION_UNAVAILABLE; }
            m->sample.speed_um_s[i] = (uint32_t)speed;
            m->window_um[i] = 0U;
        }
        m->sample.speed_valid = true;
        m->sample.signed_speed_valid = !m->direction_changed && s->edge_times_valid &&
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
