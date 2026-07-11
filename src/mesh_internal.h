/*
 * mesh_internal.h — module interfaces shared across the library sources.
 * Not part of the public API; unit tests may include it to reach codec
 * and table internals.
 */
#ifndef MESH_INTERNAL_H
#define MESH_INTERNAL_H

#include <string.h>

#include "mesh/mesh.h"

/* ---- Frame header byte offsets (§8) ---- */
#define MESH_OFF_NET  0 /* 2 B, LE */
#define MESH_OFF_TYPE 2
#define MESH_OFF_SRC  3
#define MESH_OFF_DST  4
#define MESH_OFF_SEQ  5 /* 2 B, LE */
#define MESH_OFF_TTL  7
#define MESH_OFF_PAYLOAD 8

/* Decoded frame view. `payload` points into the caller's rx buffer. */
typedef struct {
    uint8_t  type;
    uint8_t  src;
    uint8_t  dst;
    uint16_t seq;
    uint8_t  ttl;   /* hop count in beacons, TTL elsewhere */
    uint8_t  plen;
    const uint8_t *payload;
} mesh_frame_t;

typedef enum {
    MESH_FRAME_OK = 0,
    MESH_FRAME_E_LEN,  /* too short / too long / bad payload length */
    MESH_FRAME_E_NET,  /* foreign network ID */
    MESH_FRAME_E_MIC,
} mesh_frame_rc_t;

/* frame.c */
size_t mesh_frame_build(uint16_t net_id, const mesh_mic_ops_t *mic,
                        uint8_t buf[MESH_MAX_FRAME],
                        uint8_t type, uint8_t src, uint8_t dst,
                        uint16_t seq, uint8_t ttl,
                        const uint8_t *payload, uint8_t plen);
mesh_frame_rc_t mesh_frame_parse(uint16_t net_id, const mesh_mic_ops_t *mic,
                                 const uint8_t *buf, size_t len,
                                 mesh_frame_t *out);

/* ---- Sequence-number serial arithmetic (wrap-safe) ---- */
static inline int16_t mesh_seq_diff(uint16_t a, uint16_t b)
{
    return (int16_t)(uint16_t)(a - b);
}
static inline bool mesh_seq_gt(uint16_t a, uint16_t b)
{
    return mesh_seq_diff(a, b) > 0;
}

/* Saturating Q8.8 cost addition; infinity is absorbing. */
static inline uint16_t mesh_cost_add(uint16_t a, uint16_t b)
{
    uint32_t s = (uint32_t)a + (uint32_t)b;
    return (s >= MESH_COST_INF) ? MESH_COST_INF : (uint16_t)s;
}

/* mesh.c */
uint32_t mesh_rand(mesh_t *m);                       /* xorshift32 */
uint32_t mesh_rand_below(mesh_t *m, uint32_t bound); /* [0, bound) */

/* ---- txqueue.c ----
 * All transmissions flow through the queue; entries fire in mesh_poll. */
typedef enum {
    MESH_TXK_RAW = 0,     /* prebuilt, fire and forget */
    MESH_TXK_UNICAST = 1, /* prebuilt, HOP_ACK-tracked with retries */
    MESH_TXK_BEACON = 2,  /* built at fire time from current tree state */
} mesh_txk_t;

mesh_txq_entry_t *mesh_txq_alloc(mesh_t *m);
/* Schedule a prebuilt frame; returns false if the queue is full. */
bool mesh_txq_raw(mesh_t *m, const uint8_t *buf, uint8_t len, uint64_t due);
bool mesh_txq_unicast(mesh_t *m, const uint8_t *buf, uint8_t len,
                      uint64_t due, uint8_t ack_from, uint8_t ack_type,
                      uint16_t ack_seq);
bool mesh_txq_beacon(mesh_t *m, uint16_t beacon_seq, uint64_t due);
uint64_t mesh_txq_poll(mesh_t *m, uint64_t now);
/* HOP_ACK arrived from `from` covering (type, seq); returns true if a
 * pending entry was satisfied. */
bool mesh_txq_on_hop_ack(mesh_t *m, uint8_t from, uint8_t type,
                         uint16_t seq);

/* ---- neighbor.c ---- */
mesh_neighbor_t *mesh_nbr_find(mesh_t *m, uint8_t id);
/* Update RSSI/SNR and last-heard for any frame from `id` (adds an entry,
 * evicting if needed). Returns NULL only if id is invalid. */
mesh_neighbor_t *mesh_nbr_heard(mesh_t *m, uint8_t id,
                                int16_t rssi_dbm, int8_t snr_db);
/* Record a beacon from `id` and refresh its ETX estimate. */
void mesh_nbr_beacon(mesh_t *m, mesh_neighbor_t *n, uint16_t seq,
                     uint16_t adv_cost, uint8_t hops);
/* Current-effective link ETX (Q8.8), penalized for beacons missed since
 * last heard; MESH_COST_INF if unusable. */
uint16_t mesh_nbr_etx_now(const mesh_t *m, const mesh_neighbor_t *n);
/* True if the neighbor is a live routing candidate at the current epoch. */
bool mesh_nbr_fresh(const mesh_t *m, const mesh_neighbor_t *n);

/* ---- tree.c ---- */
/* Re-evaluate parent selection (freshness, loop guard, hysteresis).
 * Called on beacon receipt and on parent-link failure. */
void mesh_tree_evaluate(mesh_t *m);
/* Parent link declared dead by upward-send failures. */
void mesh_tree_parent_dead(mesh_t *m);

/* ---- beacon.c ---- */
void mesh_beacon_rx(mesh_t *m, const mesh_frame_t *f,
                    int16_t rssi_dbm, int8_t snr_db, uint64_t now);
/* Build this node's beacon rebroadcast; returns 0 to skip (unattached). */
uint8_t mesh_beacon_build(mesh_t *m, uint16_t beacon_seq,
                          uint8_t buf[MESH_MAX_FRAME]);

/* ---- flood.c ---- */
void mesh_flood_rx(mesh_t *m, const mesh_frame_t *f, uint64_t now);
bool mesh_flood_seen(mesh_t *m, uint16_t seq);   /* check + insert */

/* ---- command.c (execution + upward unicast engine) ---- */
void mesh_cmd_execute(mesh_t *m, const mesh_frame_t *f, uint64_t now);
/* Send a tree-unicast frame (CMD_ACK/TELEM payload) toward the root. */
bool mesh_up_send(mesh_t *m, uint8_t type, const uint8_t *payload,
                  uint8_t plen, uint8_t ttl, uint64_t now);
/* Handle a received CMD_ACK/TELEM frame (hop-ack, dedup, deliver/relay). */
void mesh_up_rx(mesh_t *m, const mesh_frame_t *f, uint64_t now);
void mesh_hop_ack_rx(mesh_t *m, const mesh_frame_t *f);

/* ---- telemetry.c ---- */
void mesh_telem_deliver_root(mesh_t *m, const mesh_frame_t *f);

#endif /* MESH_INTERNAL_H */
