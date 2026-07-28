/*
 * flood.c — controlled flooding for commands (§6) and FETCH requests
 * (SESSION frames, §7.3), which share the relay/dedup machinery and
 * the root's flood_seq space.
 *
 * Frame-level dedup is on the frame sequence number, which the root
 * increments on every flood transmission (including re-floods of the
 * same command). Execution-level dedup is on the command ID carried in
 * the payload, handled in command.c — so a re-flood propagates through
 * the whole network again but is executed at most once per node. FETCH
 * needs no execution dedup: re-executing one is the §7.3 resumption
 * mechanism.
 */
#include "mesh_internal.h"

bool mesh_flood_seen(mesh_t *m, uint16_t seq)
{
    for (unsigned i = 0; i < m->flood_seen_cnt; i++) {
        if (m->flood_seen[i] == seq)
            return true;
    }
    m->flood_seen[m->flood_seen_idx] = seq;
    m->flood_seen_idx = (uint8_t)((m->flood_seen_idx + 1) %
                                  MESH_FLOOD_DEDUP);
    if (m->flood_seen_cnt < MESH_FLOOD_DEDUP)
        m->flood_seen_cnt++;
    return false;
}

void mesh_flood_rx(mesh_t *m, const mesh_frame_t *f, uint64_t now)
{
    uint8_t min_plen = (f->type == MESH_FT_SESSION) ? MESH_FETCH_PLEN
                                                    : MESH_CMD_HDR;
    if (f->plen < min_plen)
        return;
    if (mesh_flood_seen(m, f->seq))
        return; /* duplicate (or our own echo at the root) */
    if (f->type == MESH_FT_CMD)
        m->stats.cmds_seen++;

    /* Relay: targeted floods are still relayed by every node (§6.3).
     * The root never relays — it originated the flood. */
    if (!m->is_root && f->ttl > 1) {
        uint8_t buf[MESH_MAX_FRAME];
        size_t len = mesh_frame_build(m->network_id, &m->mic, buf,
                                      f->type, m->node_id, f->dst,
                                      f->seq, (uint8_t)(f->ttl - 1),
                                      f->payload, f->plen);
        uint64_t due = now + mesh_rand_below(m, MESH_FLOOD_BACKOFF_US);
        if (len && mesh_txq_raw(m, buf, (uint8_t)len, due))
            m->stats.floods_relayed++;
    }

    if (m->is_root)
        return;
    if (f->type == MESH_FT_CMD) {
        if (f->dst == MESH_ID_BROADCAST || f->dst == m->node_id)
            mesh_cmd_execute(m, f, now);
    } else if (f->dst == m->node_id) {
        /* FETCH is always targeted — a broadcast FETCH would set every
         * node streaming at once and is ignored. */
        mesh_bulk_fetch_execute(m, f, now);
    }
}

uint16_t mesh_root_command(mesh_t *m, uint8_t target,
                           const uint8_t *payload, uint8_t len,
                           uint16_t cmd_id, uint64_t now_us)
{
    if (!m || !m->initialized || !m->is_root)
        return 0;
    if (len > MESH_MAX_CMD_PAYLOAD || (len > 0 && payload == NULL))
        return 0;

    if (cmd_id == 0) {
        cmd_id = m->next_cmd_id++;
        if (m->next_cmd_id == 0)
            m->next_cmd_id = 1;
    }

    uint16_t seq = ++m->flood_seq;
    (void)mesh_flood_seen(m, seq); /* pre-seed: ignore our own echoes */

    uint8_t body[MESH_MAX_PAYLOAD];
    body[0] = (uint8_t)(cmd_id & 0xFF);
    body[1] = (uint8_t)(cmd_id >> 8);
    if (len > 0)
        memcpy(&body[MESH_CMD_HDR], payload, len);

    uint8_t buf[MESH_MAX_FRAME];
    size_t flen = mesh_frame_build(m->network_id, &m->mic, buf,
                                   MESH_FT_CMD, m->node_id, target, seq,
                                   MESH_FLOOD_TTL, body,
                                   (uint8_t)(MESH_CMD_HDR + len));
    if (!flen || !mesh_txq_raw(m, buf, (uint8_t)flen, now_us))
        return 0;
    return cmd_id;
}
