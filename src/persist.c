/*
 * persist.c — route/state snapshot for flash persistence (§5.3).
 *
 * Fixed deployments should not have to re-form the tree after a watchdog
 * or brown-out reboot. The application (which owns the flash) saves a
 * small self-checking blob and hands it back after the next boot:
 *
 *   Field node: parent + path cost + the neighbor table's smoothed ETX
 *   estimates + the beacon-epoch position. On load the node resumes its
 *   last-known route immediately — it can tree-unicast telemetry before
 *   hearing a single beacon — and repairs lazily: estimator windows
 *   restart (seeded with the saved EWMA), and the §5.4 staleness rule
 *   still applies because the saved net_epoch/last_beacon_seq pair ages
 *   across the outage. A node that was down longer than
 *   MESH_STALE_EPOCHS superframes finds its restored route poisoned by
 *   the first fresh beacon and rejoins from scratch, which is exactly
 *   what §5.4 wants.
 *
 *   Root: the beacon/flood/command counters, restored with a margin of
 *   MESH_PERSIST_SEQ_MARGIN so they are strictly ahead of anything the
 *   fleet has seen. Without this a rebooted base restarts its beacon
 *   seq at 0, which wrap-safe comparison reads as *older* than the
 *   fleet's held epoch — beacons get ignored, staleness never advances,
 *   and the network wedges. With the margin the fleet sees a forward
 *   epoch jump, poisons its stale state, and re-forms within a few
 *   superframes; commands keep executing because flood seqs and command
 *   IDs never revisit dedup'd values.
 *
 * The blob is versioned, CRC-guarded, and bound to (node_id, net_id) so
 * a re-provisioned device silently discards a stale cache. Load must
 * happen immediately after mesh_init, before any received traffic.
 */
#include "mesh_internal.h"

static const uint8_t route_magic[4] = { 'M', 'R', 'T', '1' };

#define ROLE_ROOT 0u
#define ROLE_NODE 1u

/* Common header: magic(4) role(1) node_id(1) net_id(2). */
#define HDR 8u
#define NBR_REC 8u /* id(1) hops(1) adv_cost(2) etx_q8(2) last_seq(2) */

static void put16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

size_t mesh_route_save(const mesh_t *m, uint8_t *buf, size_t max)
{
    if (!m->initialized || max < MESH_ROUTE_BLOB_MAX)
        return 0;

    memcpy(buf, route_magic, 4);
    buf[4] = m->is_root ? ROLE_ROOT : ROLE_NODE;
    buf[5] = m->node_id;
    put16(&buf[6], m->network_id);

    size_t off = HDR;
    if (m->is_root) {
        put16(&buf[off], m->beacon_seq);
        put16(&buf[off + 2], m->flood_seq);
        put16(&buf[off + 4], m->next_cmd_id);
        off += 6;
    } else {
        put16(&buf[off], m->net_epoch);
        buf[off + 2] = m->have_epoch ? 1 : 0;
        buf[off + 3] = m->parent;
        put16(&buf[off + 4], m->path_cost);
        buf[off + 6] = m->hops;
        off += 7;

        size_t cnt_at = off++;
        uint8_t cnt = 0;
        for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
            const mesh_neighbor_t *n = &m->nbrs[i];
            if (!n->in_use || !n->ever_beacon || n->etx_q8 == 0)
                continue; /* nothing worth reseeding an estimator with */
            buf[off] = n->id;
            buf[off + 1] = n->hops;
            put16(&buf[off + 2], n->adv_cost);
            put16(&buf[off + 4], n->etx_q8);
            put16(&buf[off + 6], n->last_beacon_seq);
            off += NBR_REC;
            cnt++;
        }
        buf[cnt_at] = cnt;
    }

    put16(&buf[off], mesh_crc16(buf, off));
    return off + 2;
}

bool mesh_route_load(mesh_t *m, const uint8_t *buf, size_t len)
{
    if (!m->initialized || len < HDR + 2)
        return false;
    if (memcmp(buf, route_magic, 4) != 0)
        return false;

    /* Locate the CRC from the declared shape, then verify. */
    size_t body;
    if (buf[4] == ROLE_ROOT) {
        body = HDR + 6;
    } else if (buf[4] == ROLE_NODE) {
        if (len < HDR + 8)
            return false;
        body = HDR + 8 + (size_t)buf[HDR + 7] * NBR_REC;
    } else {
        return false;
    }
    if (body + 2 > len || mesh_crc16(buf, body) != get16(&buf[body]))
        return false;

    /* A blob from another identity (re-provisioned device, different
     * deployment) is silently stale. */
    if (buf[5] != m->node_id || get16(&buf[6]) != m->network_id)
        return false;
    if ((buf[4] == ROLE_ROOT) != m->is_root)
        return false;

    if (m->is_root) {
        m->beacon_seq = (uint16_t)(get16(&buf[HDR]) +
                                   MESH_PERSIST_SEQ_MARGIN);
        m->flood_seq = (uint16_t)(get16(&buf[HDR + 2]) +
                                  MESH_PERSIST_SEQ_MARGIN);
        m->next_cmd_id = (uint16_t)(get16(&buf[HDR + 4]) +
                                    MESH_PERSIST_SEQ_MARGIN);
        return true;
    }

    /* Field node: only into a factory-fresh mesh (right after init). */
    if (m->parent != MESH_ID_NONE || m->have_epoch)
        return false;

    uint8_t cnt = buf[HDR + 7];
    if (cnt > MESH_MAX_NEIGHBORS)
        return false;

    m->net_epoch = get16(&buf[HDR]);
    m->have_epoch = buf[HDR + 2] != 0;

    const uint8_t *rec = &buf[HDR + 8];
    for (uint8_t i = 0; i < cnt; i++, rec += NBR_REC) {
        mesh_neighbor_t *n = &m->nbrs[i];
        memset(n, 0, sizeof(*n));
        n->in_use = true;
        n->ever_beacon = true;
        n->id = rec[0];
        n->hops = rec[1];
        n->adv_cost = get16(&rec[2]);
        n->etx_q8 = get16(&rec[4]);
        n->last_beacon_seq = get16(&rec[6]);
        n->last_heard_seq = n->last_beacon_seq;
        /* Estimator restarts (§5.2 refinement 3) seeded with the saved
         * EWMA: one "received" epoch in the window, so etx_now returns
         * max(saved, 1.0) until fresh beacons refine it. */
        n->beacon_bitmap = 1;
        n->epochs_known = 1;
        n->obs_at = m->obs_epochs;
    }

    uint8_t parent = buf[HDR + 3];
    if (parent != MESH_ID_NONE && mesh_nbr_find(m, parent) != NULL) {
        m->parent = parent;
        m->path_cost = get16(&buf[HDR + 4]);
        m->hops = buf[HDR + 6];
        m->stats.parent_changes++;
        if (m->cb.on_parent_change)
            m->cb.on_parent_change(m->cb.user, MESH_ID_NONE, parent);
    }
    return true;
}
