/*
 * neighbor.c — neighbor table and ETX link estimation (§5.2).
 *
 * ETX is estimated from the beacon reception ratio over a sliding window
 * of the last MESH_ETX_WINDOW OBSERVED epochs (see the note on
 * mesh_neighbor_t), kept as a bitmap (bit 0 = the newest observed epoch
 * we heard this neighbor in). The instantaneous ratio is smoothed with
 * an EWMA at hear-time; at read-time the estimate is additionally
 * penalized for observed epochs of silence since the last beacon, so a
 * dying link degrades even while nothing is heard from it.
 *
 * A neighbor that reappears after sitting out past the staleness
 * threshold gets a fresh estimator (history reset, ETX rebuilt from its
 * new receptions): once a link has been written off, a revived node
 * should not drag around the ETX of its own death. Hysteresis in tree.c
 * keeps this optimism from causing route flapping.
 */
#include "mesh_internal.h"

static unsigned popcount16(uint16_t v)
{
    unsigned c = 0;
    while (v) {
        v &= (uint16_t)(v - 1);
        c++;
    }
    return c;
}

static uint16_t window_mask(unsigned window)
{
    return (window >= 16) ? 0xFFFFu : (uint16_t)((1u << window) - 1u);
}

mesh_neighbor_t *mesh_nbr_find(mesh_t *m, uint8_t id)
{
    for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
        if (m->nbrs[i].in_use && m->nbrs[i].id == id)
            return &m->nbrs[i];
    }
    return NULL;
}

/* Eviction preference: never the current parent; then entries we have
 * never heard a beacon from; then the worst current ETX; then the least
 * recently heard. */
static mesh_neighbor_t *nbr_evict_victim(mesh_t *m)
{
    mesh_neighbor_t *victim = NULL;
    for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
        mesh_neighbor_t *n = &m->nbrs[i];
        if (n->id == m->parent && m->parent != MESH_ID_NONE)
            continue;
        if (!victim) {
            victim = n;
            continue;
        }
        if (n->ever_beacon != victim->ever_beacon) {
            if (!n->ever_beacon)
                victim = n;
            continue;
        }
        uint16_t en = mesh_nbr_etx_now(m, n);
        uint16_t ev = mesh_nbr_etx_now(m, victim);
        if (en != ev) {
            if (en > ev)
                victim = n;
            continue;
        }
        if (mesh_seq_diff(victim->last_heard_seq, n->last_heard_seq) > 0)
            victim = n;
    }
    return victim;
}

mesh_neighbor_t *mesh_nbr_heard(mesh_t *m, uint8_t id,
                                int16_t rssi_dbm, int8_t snr_db)
{
    if (id == m->node_id || id == MESH_ID_BROADCAST)
        return NULL;

    mesh_neighbor_t *n = mesh_nbr_find(m, id);
    if (!n) {
        for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
            if (!m->nbrs[i].in_use) {
                n = &m->nbrs[i];
                break;
            }
        }
        if (!n) {
            n = nbr_evict_victim(m);
            if (!n)
                return NULL; /* table of size 1 holding the parent */
            m->stats.neighbors_evicted++;
        }
        memset(n, 0, sizeof(*n));
        n->in_use = true;
        n->id = id;
        n->hops = 0xFF;
        n->adv_cost = MESH_COST_INF;
        n->rssi_q4 = (int16_t)(rssi_dbm * 16);
        n->snr_q4 = (int16_t)(snr_db * 16);
    } else {
        n->rssi_q4 = (int16_t)(n->rssi_q4 +
                               (((int16_t)(rssi_dbm * 16) - n->rssi_q4) >> 2));
        n->snr_q4 = (int16_t)(n->snr_q4 +
                              (((int16_t)(snr_db * 16) - n->snr_q4) >> 2));
    }
    n->last_heard_seq = m->net_epoch;
    return n;
}

void mesh_nbr_beacon(mesh_t *m, mesh_neighbor_t *n, uint16_t seq,
                     uint16_t adv_cost, uint8_t hops)
{
    if (!n->ever_beacon) {
        n->ever_beacon = true;
        n->beacon_bitmap = 1;
        n->last_beacon_seq = seq;
        n->obs_at = m->obs_epochs;
        n->epochs_known = 1;
    } else {
        int16_t d = mesh_seq_diff(seq, n->last_beacon_seq);
        if (d > 0) {
            uint16_t shift = (uint16_t)(m->obs_epochs - n->obs_at);
            if (shift > MESH_STALE_EPOCHS) {
                /* The neighbor sat out past the staleness threshold —
                 * any route through it was already poisoned (§5.4). Now
                 * that it is back, re-probe it with a fresh estimator
                 * rather than letting the outage haunt the ETX window
                 * for another 16 epochs (a revived relay would otherwise
                 * take hours of standby superframes to be re-adopted). */
                n->beacon_bitmap = 1;
                n->epochs_known = 1;
                n->etx_q8 = 0;
            } else {
                n->beacon_bitmap =
                    (uint16_t)((n->beacon_bitmap << shift) | 1u);
                uint32_t known =
                    (uint32_t)n->epochs_known + (uint32_t)shift;
                n->epochs_known = (known > MESH_ETX_WINDOW)
                                      ? MESH_ETX_WINDOW
                                      : (uint16_t)known;
            }
            n->last_beacon_seq = seq;
            n->obs_at = m->obs_epochs;
        }
        /* d == 0: duplicate copy of the same beacon (bit already set);
         * d < 0: stale reordered copy — ignore for the estimator. */
    }

    n->adv_cost = adv_cost;
    n->hops = hops;

    unsigned window = n->epochs_known;
    unsigned recv = popcount16(n->beacon_bitmap & window_mask(window));
    /* recv >= 1: we just recorded a reception. */
    uint32_t raw = ((uint32_t)window * MESH_ETX_ONE) / recv;
    if (raw > MESH_ETX_MAX_Q8)
        raw = MESH_ETX_MAX_Q8;
    if (n->etx_q8 == 0)
        n->etx_q8 = (uint16_t)raw;
    else
        n->etx_q8 = (uint16_t)(n->etx_q8 +
                               (((int32_t)raw - (int32_t)n->etx_q8) >>
                                MESH_ETX_EWMA_SHIFT));
}

uint16_t mesh_nbr_etx_now(const mesh_t *m, const mesh_neighbor_t *n)
{
    if (!n->in_use || !n->ever_beacon || n->etx_q8 == 0)
        return MESH_COST_INF;

    uint16_t shift = (uint16_t)(m->obs_epochs - n->obs_at);
    /* The newest observed epoch is still in progress: a neighbor whose
     * randomized rebroadcast has not fired yet is not a miss. Counting
     * it would make link estimates oscillate within an epoch depending
     * on beacon arrival order (and reset parent hysteresis forever). */
    if (shift > 0)
        shift--;
    if (shift >= MESH_ETX_WINDOW)
        return MESH_COST_INF;

    uint16_t bm = (uint16_t)(n->beacon_bitmap << shift);
    unsigned window = n->epochs_known + (unsigned)shift;
    if (window > MESH_ETX_WINDOW)
        window = MESH_ETX_WINDOW;
    unsigned recv = popcount16(bm & window_mask(window));
    if (recv == 0)
        return MESH_COST_INF;

    uint32_t raw = ((uint32_t)window * MESH_ETX_ONE) / recv;
    if (raw > MESH_ETX_MAX_Q8)
        raw = MESH_ETX_MAX_Q8;

    /* Use the worse of the smoothed estimate and the silence-penalized
     * instantaneous one, so links decay while quiet but recover with the
     * EWMA once beacons resume. */
    return (n->etx_q8 > raw) ? n->etx_q8 : (uint16_t)raw;
}

/* Staleness is judged in raw beacon-sequence delta, not observed epochs:
 * the root's sequence number is ground truth for elapsed superframes, so
 * a parent that has not delivered a beacon for MESH_STALE_EPOCHS real
 * superframes is poisoned promptly (§5.4) even when this node's own
 * observation rate is degraded. */
bool mesh_nbr_fresh(const mesh_t *m, const mesh_neighbor_t *n)
{
    if (!n->in_use || !n->ever_beacon)
        return false;
    if (!m->have_epoch)
        return true;
    return mesh_seq_diff(m->net_epoch, n->last_beacon_seq) <
           MESH_STALE_EPOCHS;
}
