/*
 * beacon.c — beacon flood and rebroadcast (§5.1).
 *
 * The root originates one beacon per superframe (the application decides
 * when — superframe scheduling arrives in a later milestone). Every
 * attached node rebroadcasts each beacon sequence number exactly once,
 * after a randomized offset, carrying its own ID, hop count, and current
 * path cost — so the flood costs O(N) transmissions and doubles as the
 * link-probe traffic that feeds ETX estimation.
 *
 * Beacon payload: path cost (2 B LE, Q8.8). Hop count travels in the
 * header TTL field.
 */
#include "mesh_internal.h"

void mesh_beacon_rx(mesh_t *m, const mesh_frame_t *f,
                    int16_t rssi_dbm, int8_t snr_db, uint64_t now)
{
    (void)rssi_dbm;
    (void)snr_db;

    if (f->plen < 2)
        return; /* malformed */
    m->stats.beacons_rx++;

    /* Epoch = highest beacon seq seen. Each distinct new epoch counts
     * as one OBSERVED epoch — the unit of the ETX windows.
     *
     * Resync guard: a beacon whose seq is *far* below our held epoch
     * cannot be a genuinely stale copy (relays only rebroadcast a few
     * epochs deep) — it means OUR epoch view is bogus: a restored
     * flash cache that aged past half the seq space (§5.3), or a base
     * whose counter blob was lost and restarted near 0. Adopting the
     * live seq lets the node re-join instead of wedging: its old
     * neighbor state then reads as stale and is repaired normally. */
    if (m->have_epoch &&
        mesh_seq_diff(f->seq, m->net_epoch) <= -(int16_t)MESH_EPOCH_RESYNC)
        m->have_epoch = false;
    if (!m->have_epoch || mesh_seq_gt(f->seq, m->net_epoch)) {
        m->net_epoch = f->seq;
        m->have_epoch = true;
        m->obs_epochs++;
    }

    uint16_t adv_cost = (uint16_t)(f->payload[0] |
                                   ((uint16_t)f->payload[1] << 8));
    mesh_neighbor_t *n = mesh_nbr_find(m, f->src);
    if (!n)
        return; /* bookkeeping in mesh_rx failed (invalid src) */
    mesh_nbr_beacon(m, n, f->seq, adv_cost, f->ttl);

    mesh_tree_evaluate(m);

    /* Schedule our single rebroadcast of this sequence number. The frame
     * is built at fire time so it carries whatever cost we hold then;
     * if we are unattached at fire time it is silently skipped. */
    if (!m->is_root &&
        (!m->have_rebc || mesh_seq_gt(f->seq, m->last_rebc_seq))) {
        uint64_t due = now + mesh_rand_below(m, MESH_BEACON_BACKOFF_US);
        if (mesh_txq_beacon(m, f->seq, due)) {
            m->last_rebc_seq = f->seq;
            m->have_rebc = true;
        }
    }
}

uint8_t mesh_beacon_build(mesh_t *m, uint16_t beacon_seq,
                          uint8_t buf[MESH_MAX_FRAME])
{
    if (m->is_root)
        return 0;
    if (m->parent == MESH_ID_NONE || m->path_cost == MESH_COST_INF)
        return 0; /* unattached nodes stop relaying (§5.4) */

    uint8_t payload[2] = {
        (uint8_t)(m->path_cost & 0xFF),
        (uint8_t)(m->path_cost >> 8),
    };
    size_t len = mesh_frame_build(m->network_id, &m->mic, buf,
                                  MESH_FT_BEACON, m->node_id,
                                  MESH_ID_BROADCAST, beacon_seq, m->hops,
                                  payload, sizeof(payload));
    if (len)
        m->stats.beacons_rebroadcast++;
    return (uint8_t)len;
}

bool mesh_root_beacon(mesh_t *m, uint64_t now_us)
{
    if (!m || !m->initialized || !m->is_root)
        return false;

    uint16_t seq = ++m->beacon_seq;
    m->net_epoch = seq;
    m->have_epoch = true;
    m->obs_epochs++;

    uint8_t payload[2] = { 0, 0 }; /* root path cost is zero */
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(m->network_id, &m->mic, buf,
                                  MESH_FT_BEACON, m->node_id,
                                  MESH_ID_BROADCAST, seq, 0,
                                  payload, sizeof(payload));
    if (!len)
        return false;
    return mesh_txq_raw(m, buf, (uint8_t)len, now_us);
}
