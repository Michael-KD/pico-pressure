/*
 * mesh_test_helpers.h — shared fixtures for unit tests: a capture radio
 * that records every transmitted frame, and neighbor-table fabrication
 * helpers for deterministic tree tests.
 */
#ifndef MESH_TEST_HELPERS_H
#define MESH_TEST_HELPERS_H

#include <string.h>

#include "mesh_internal.h"

#define TEST_NET 0xBEEF

#define CAP_MAX 64

typedef struct {
    uint8_t frames[CAP_MAX][MESH_MAX_FRAME];
    size_t lens[CAP_MAX];
    unsigned count;
} tx_capture_t;

static inline bool cap_transmit(void *ctx, const uint8_t *frame,
                                size_t len, mesh_profile_t profile)
{
    (void)profile;
    tx_capture_t *c = (tx_capture_t *)ctx;
    if (c->count < CAP_MAX) {
        memcpy(c->frames[c->count], frame, len);
        c->lens[c->count] = len;
        c->count++;
    }
    return true;
}

static inline mesh_cfg_t test_cfg(uint8_t id, tx_capture_t *cap)
{
    mesh_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.node_id = id;
    cfg.network_id = TEST_NET;
    cfg.radio.ctx = cap;
    cfg.radio.transmit = cap_transmit;
    cfg.rng_seed = 0x2468ACE0u + id;
    return cfg;
}

/* Count captured frames of a type; if `out` is non-NULL, parse the last
 * one found into it. */
static inline unsigned cap_find(const mesh_t *m, tx_capture_t *cap,
                                uint8_t type, mesh_frame_t *out)
{
    unsigned found = 0;
    for (unsigned i = 0; i < cap->count; i++) {
        mesh_frame_t f;
        if (mesh_frame_parse(m->network_id, &m->mic, cap->frames[i],
                             cap->lens[i], &f) == MESH_FRAME_OK &&
            f.type == type) {
            found++;
            if (out) {
                *out = f;
                /* payload points into cap->frames[i]: stays valid. */
            }
        }
    }
    return found;
}

/* Discard pending transmissions and captured frames between test steps
 * (prevents unicast retry exhaustion from mutating tree state). */
static inline void drain(mesh_t *m, tx_capture_t *cap)
{
    for (unsigned i = 0; i < MESH_TXQ_DEPTH; i++)
        m->txq[i].in_use = false;
    cap->count = 0;
}

/* Fabricate a healthy neighbor: perfect beacon history at the current
 * epoch, with the given advertised cost / smoothed ETX / hop count. */
static inline mesh_neighbor_t *fab_nbr(mesh_t *m, uint8_t id,
                                       uint16_t adv_cost, uint16_t etx_q8,
                                       uint8_t hops)
{
    mesh_neighbor_t *n = mesh_nbr_heard(m, id, -60, 8);
    n->ever_beacon = true;
    n->beacon_bitmap = 0xFFFF;
    n->epochs_known = MESH_ETX_WINDOW;
    n->last_beacon_seq = m->net_epoch;
    n->obs_at = m->obs_epochs;
    n->adv_cost = adv_cost;
    n->etx_q8 = etx_q8;
    n->hops = hops;
    return n;
}

/* Advance the beacon epoch; neighbors listed in `fresh_ids` are treated
 * as having beaconed this epoch. */
static inline void bump_epoch(mesh_t *m, const uint8_t *fresh_ids,
                              unsigned n_ids)
{
    m->net_epoch++;
    m->obs_epochs++;
    m->have_epoch = true;
    for (unsigned i = 0; i < n_ids; i++) {
        mesh_neighbor_t *n = mesh_nbr_find(m, fresh_ids[i]);
        if (n) {
            n->last_beacon_seq = m->net_epoch;
            n->obs_at = m->obs_epochs;
            n->beacon_bitmap = 0xFFFF;
        }
    }
}

#endif /* MESH_TEST_HELPERS_H */
