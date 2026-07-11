/*
 * mesh.c — initialization, receive dispatch, poll loop, introspection.
 */
#include "mesh_internal.h"

/* xorshift32: small, adequate for jitter/backoff (not security). */
uint32_t mesh_rand(mesh_t *m)
{
    uint32_t x = m->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    m->rng = x;
    return x;
}

uint32_t mesh_rand_below(mesh_t *m, uint32_t bound)
{
    return bound ? (mesh_rand(m) % bound) : 0;
}

bool mesh_init(mesh_t *m, const mesh_cfg_t *cfg)
{
    if (!m || !cfg)
        return false;
    if (cfg->node_id == MESH_ID_BROADCAST)
        return false;
    if (!cfg->radio.transmit)
        return false;
    if (cfg->rng_seed == 0)
        return false;
    if (cfg->sched.enabled && !mesh_sched_mode_valid(&cfg->sched.mode))
        return false;

    memset(m, 0, sizeof(*m));
    m->node_id = cfg->node_id;
    m->network_id = cfg->network_id;
    m->is_root = (cfg->node_id == MESH_ID_ROOT);
    m->radio = cfg->radio;
    m->mic = cfg->mic.compute ? cfg->mic : mesh_mic_null;
    m->cb = cfg->cb;
    m->rng = cfg->rng_seed;

    m->parent = MESH_ID_NONE;
    m->cand_id = MESH_ID_NONE;
    m->path_cost = m->is_root ? 0 : MESH_COST_INF;
    m->hops = m->is_root ? 0 : 0xFF;
    m->next_cmd_id = 1;

    /* Scheduler (§4): superframe 0 of the initial mode starts at
     * absolute time 0, so a fleet sharing GPS time is aligned with no
     * negotiation. mesh_sched_poll() takes over set_listen from here. */
    m->sched = cfg->sched;
    if (m->sched.enabled && m->sched.guard_us == 0)
        m->sched.guard_us = MESH_SCHED_GUARD_US;
    m->mode_start_us = 0;
    m->pending_mode_at = MESH_NO_EVENT;
    m->listening = true;

    if (m->radio.set_listen)
        m->radio.set_listen(m->radio.ctx, true, MESH_PROFILE_CONTROL);

    m->initialized = true;
    return true;
}

void mesh_rx(mesh_t *m, const uint8_t *data, size_t len,
             int16_t rssi_dbm, int8_t snr_db, uint64_t now_us)
{
    if (!m || !m->initialized || !data)
        return;

    mesh_frame_t f;
    switch (mesh_frame_parse(m->network_id, &m->mic, data, len, &f)) {
    case MESH_FRAME_OK:
        break;
    case MESH_FRAME_E_LEN:
        m->stats.rx_drop_len++;
        return;
    case MESH_FRAME_E_NET:
        m->stats.rx_drop_net++;
        return;
    case MESH_FRAME_E_MIC:
        m->stats.rx_drop_mic++;
        return;
    default:
        return;
    }
    m->stats.rx_frames++;

    if (f.src == m->node_id)
        return; /* our own transmission echoed back */

    /* Every heard frame refreshes the neighbor's signal statistics. */
    mesh_nbr_heard(m, f.src, rssi_dbm, snr_db);

    switch (f.type) {
    case MESH_FT_BEACON:
        mesh_beacon_rx(m, &f, rssi_dbm, snr_db, now_us);
        break;
    case MESH_FT_CMD:
        mesh_flood_rx(m, &f, now_us);
        break;
    case MESH_FT_CMD_ACK:
    case MESH_FT_TELEM:
        mesh_up_rx(m, &f, now_us);
        break;
    case MESH_FT_HOP_ACK:
        mesh_hop_ack_rx(m, &f);
        break;
    default:
        /* DATA/DATA_ACK/SESSION are reserved for the bulk-transfer
         * milestone; unknown types are ignored. */
        break;
    }
}

uint64_t mesh_poll(mesh_t *m, uint64_t now_us)
{
    if (!m || !m->initialized)
        return MESH_NO_EVENT;
    /* Scheduler first: it may switch modes, toggle the radio, or
     * originate the root beacon that the queue pass then transmits. */
    uint64_t next = mesh_sched_poll(m, now_us);
    uint64_t tx = mesh_txq_poll(m, now_us);
    return tx < next ? tx : next;
}

uint8_t mesh_parent(const mesh_t *m)
{
    return m->parent;
}

uint16_t mesh_path_cost(const mesh_t *m)
{
    return m->path_cost;
}

uint8_t mesh_hops(const mesh_t *m)
{
    return m->hops;
}

const mesh_stats_t *mesh_stats_get(const mesh_t *m)
{
    return &m->stats;
}

const mesh_neighbor_t *mesh_neighbor_at(const mesh_t *m, unsigned idx)
{
    if (idx >= MESH_MAX_NEIGHBORS || !m->nbrs[idx].in_use)
        return NULL;
    return &m->nbrs[idx];
}
