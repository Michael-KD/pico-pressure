/*
 * persist.c — route/state snapshot for flash persistence (§5.3).
 *
 * Fixed deployments should not have to re-form the tree after a watchdog
 * or brown-out reboot. The application (which owns the flash) saves a
 * small self-checking blob and hands it back after the next boot:
 *
 *   Both roles: the scheduler's current mode and alignment origin, so a
 *   reboot resumes the fleet's *commanded* schedule rather than the
 *   boot-default mode — without this, a node rebooting after any fleet
 *   mode switch would transmit into the fleet's sleep segments and
 *   listen at the wrong times until the next switch.
 *
 *   Field node: parent + path cost + the neighbor table's smoothed ETX
 *   estimates + the beacon-epoch position. On load the node resumes its
 *   last-known route immediately — it can tree-unicast telemetry before
 *   hearing a single beacon — and repairs lazily: estimator windows
 *   restart (seeded with the saved EWMA), and the §5.4 staleness rule
 *   still applies because the saved net_epoch/last_beacon_seq pair ages
 *   across the outage. A node down longer than MESH_STALE_EPOCHS
 *   superframes finds its restored route poisoned by the first fresh
 *   beacon and rejoins from scratch; one down past half the sequence
 *   space is caught by the §5.3 epoch-resync guard in beacon.c.
 *
 *   Root: the beacon/flood/command counters, restored with a margin of
 *   MESH_PERSIST_SEQ_MARGIN so they are strictly ahead of anything the
 *   fleet has seen. Without this a rebooted base restarts its beacon
 *   seq at 0, which wrap-safe comparison reads as *older* than the
 *   fleet's held state and gets ignored (the resync guard bounds the
 *   damage, but the margin avoids it entirely); commands keep executing
 *   because flood seqs and command IDs never revisit dedup'd values.
 *
 * The blob is versioned, CRC-guarded, and bound to (node_id, net_id) so
 * a re-provisioned device silently discards a stale cache. Load must
 * happen immediately after mesh_init, before any received traffic.
 */
#include "mesh_internal.h"

static const uint8_t route_magic[4] = { 'M', 'R', 'T', '2' };

#define ROLE_ROOT 0u
#define ROLE_NODE 1u

/* Common header: magic(4) role(1) node_id(1) net_id(2), then the
 * scheduler block: mode_start(8) period(8) five windows(4 each). */
#define HDR 8u
#define SCHED_BLK 36u
#define BODY0 (HDR + SCHED_BLK)
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

static void put32(uint8_t *p, uint32_t v)
{
    put16(p, (uint16_t)(v & 0xFFFF));
    put16(p + 2, (uint16_t)(v >> 16));
}

static uint32_t get32(const uint8_t *p)
{
    return (uint32_t)get16(p) | ((uint32_t)get16(p + 2) << 16);
}

static void put64(uint8_t *p, uint64_t v)
{
    put32(p, (uint32_t)(v & 0xFFFFFFFFu));
    put32(p + 4, (uint32_t)(v >> 32));
}

static uint64_t get64(const uint8_t *p)
{
    return (uint64_t)get32(p) | ((uint64_t)get32(p + 4) << 32);
}

size_t mesh_route_save(const mesh_t *m, uint8_t *buf, size_t max)
{
    if (!m->initialized || max < MESH_ROUTE_BLOB_MAX)
        return 0;

    memcpy(buf, route_magic, 4);
    buf[4] = m->is_root ? ROLE_ROOT : ROLE_NODE;
    buf[5] = m->node_id;
    put16(&buf[6], m->network_id);

    /* Scheduler block: the mode in force and its alignment origin.
     * All zero when the scheduler is off. */
    memset(&buf[HDR], 0, SCHED_BLK);
    if (m->sched.enabled) {
        put64(&buf[HDR], m->mode_start_us);
        put64(&buf[HDR + 8], m->sched.mode.period_us);
        put32(&buf[HDR + 16], m->sched.mode.beacon_win_us);
        put32(&buf[HDR + 20], m->sched.mode.cmd_win_us);
        put32(&buf[HDR + 24], m->sched.mode.telem_win_us);
        put32(&buf[HDR + 28], m->sched.mode.bulk_win_us);
        put32(&buf[HDR + 32], m->sched.mode.telem_slot_us);
    }

    size_t off = BODY0;
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
    if (!m->initialized || len < BODY0 + 2)
        return false;
    if (memcmp(buf, route_magic, 4) != 0)
        return false;

    /* Locate the CRC from the declared shape, then verify. */
    size_t body;
    if (buf[4] == ROLE_ROOT) {
        body = BODY0 + 6;
    } else if (buf[4] == ROLE_NODE) {
        if (len < BODY0 + 8)
            return false;
        body = BODY0 + 8 + (size_t)buf[BODY0 + 7] * NBR_REC;
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

    /* Field node restores only into a factory-fresh mesh. */
    if (!m->is_root && (m->parent != MESH_ID_NONE || m->have_epoch))
        return false;

    /* Resume the commanded schedule: mode + alignment origin. Applied
     * only when both the blob and this boot run the scheduler (a bench
     * boot ignores a scheduled blob and vice versa). */
    if (m->sched.enabled) {
        mesh_sched_mode_t md;
        md.period_us = get64(&buf[HDR + 8]);
        md.beacon_win_us = get32(&buf[HDR + 16]);
        md.cmd_win_us = get32(&buf[HDR + 20]);
        md.telem_win_us = get32(&buf[HDR + 24]);
        md.bulk_win_us = get32(&buf[HDR + 28]);
        md.telem_slot_us = get32(&buf[HDR + 32]);
        if (md.period_us != 0 && mesh_sched_mode_valid(&md)) {
            m->sched.mode = md;
            m->mode_start_us = get64(&buf[HDR]);
            m->pending_mode_at = MESH_NO_EVENT;
        }
    }

    if (m->is_root) {
        m->beacon_seq = (uint16_t)(get16(&buf[BODY0]) +
                                   MESH_PERSIST_SEQ_MARGIN);
        m->flood_seq = (uint16_t)(get16(&buf[BODY0 + 2]) +
                                  MESH_PERSIST_SEQ_MARGIN);
        m->next_cmd_id = (uint16_t)(get16(&buf[BODY0 + 4]) +
                                    MESH_PERSIST_SEQ_MARGIN);
        if (m->next_cmd_id == 0) /* 0 is the auto/error sentinel */
            m->next_cmd_id = 1;
        return true;
    }

    uint8_t cnt = buf[BODY0 + 7];
    if (cnt > MESH_MAX_NEIGHBORS)
        return false;

    m->net_epoch = get16(&buf[BODY0]);
    m->have_epoch = buf[BODY0 + 2] != 0;

    const uint8_t *rec = &buf[BODY0 + 8];
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

    uint8_t parent = buf[BODY0 + 3];
    if (parent != MESH_ID_NONE && mesh_nbr_find(m, parent) != NULL) {
        m->parent = parent;
        m->path_cost = get16(&buf[BODY0 + 4]);
        m->hops = buf[BODY0 + 6];
        m->stats.parent_changes++;
        if (m->cb.on_parent_change)
            m->cb.on_parent_change(m->cb.user, MESH_ID_NONE, parent);
    }
    return true;
}
