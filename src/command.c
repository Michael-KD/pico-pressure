/*
 * command.c — command execution with ACKs, and the tree-unicast engine
 * shared by CMD_ACK and TELEM traffic (§6 step 4, design principle 4).
 *
 * Upward frames are unicast hop by hop: dst is the next hop (the
 * sender's parent), src is the immediate transmitter, and the seq field
 * is re-stamped at every hop from that node's counter. End-to-end
 * identity lives in the payload (origin node ID + command ID), so the
 * root's bookkeeping survives re-stamping. Each hop is protected by an
 * explicit HOP_ACK + retransmit; repeated exhaustion toward the parent
 * feeds dead-link detection in tree.c.
 *
 * CMD_ACK payload:  cmd_id(2 LE) origin(1) status(1)
 * HOP_ACK payload:  acked_type(1) acked_seq(2 LE)
 */
#include "mesh_internal.h"

/* ---- executed-command dedup ring ---- */

static bool exec_seen(const mesh_t *m, uint16_t cmd_id)
{
    for (unsigned i = 0; i < m->exec_seen_cnt; i++) {
        if (m->exec_seen[i] == cmd_id)
            return true;
    }
    return false;
}

static void exec_insert(mesh_t *m, uint16_t cmd_id)
{
    m->exec_seen[m->exec_seen_idx] = cmd_id;
    m->exec_seen_idx = (uint8_t)((m->exec_seen_idx + 1) % MESH_EXEC_DEDUP);
    if (m->exec_seen_cnt < MESH_EXEC_DEDUP)
        m->exec_seen_cnt++;
}

/* ---- per-hop dedup ring for upward unicast ---- */

static bool up_seen(mesh_t *m, uint8_t src, uint8_t type, uint16_t seq)
{
    for (unsigned i = 0; i < m->up_seen_cnt; i++) {
        mesh_up_dedup_t *e = &m->up_seen[i];
        if (e->src == src && e->type == type && e->seq == seq)
            return true;
    }
    mesh_up_dedup_t *e = &m->up_seen[m->up_seen_idx];
    e->src = src;
    e->type = type;
    e->seq = seq;
    m->up_seen_idx = (uint8_t)((m->up_seen_idx + 1) % MESH_UP_DEDUP);
    if (m->up_seen_cnt < MESH_UP_DEDUP)
        m->up_seen_cnt++;
    return false;
}

/* ---- sending ---- */

bool mesh_up_send_at(mesh_t *m, uint8_t type, const uint8_t *payload,
                     uint8_t plen, uint8_t ttl, uint64_t due)
{
    if (m->is_root)
        return false;
    if (m->parent == MESH_ID_NONE) {
        m->stats.up_dropped_no_parent++;
        return false;
    }

    uint16_t seq = ++m->up_seq;
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(m->network_id, &m->mic, buf, type,
                                  m->node_id, m->parent, seq, ttl,
                                  payload, plen);
    if (!len)
        return false;
    return mesh_txq_unicast(m, buf, (uint8_t)len, due, m->parent, type,
                            seq);
}

bool mesh_up_send(mesh_t *m, uint8_t type, const uint8_t *payload,
                  uint8_t plen, uint8_t ttl, uint64_t now)
{
    return mesh_up_send_at(m, type, payload, plen, ttl,
                           now + mesh_rand_below(m, MESH_UP_JITTER_US));
}

static void send_cmd_ack(mesh_t *m, uint16_t cmd_id, uint8_t status,
                         uint64_t now)
{
    uint8_t pl[4] = {
        (uint8_t)(cmd_id & 0xFF),
        (uint8_t)(cmd_id >> 8),
        m->node_id,
        status,
    };
    if (mesh_up_send(m, MESH_FT_CMD_ACK, pl, sizeof(pl), MESH_FLOOD_TTL,
                     now))
        m->stats.acks_sent++;
}

static void send_hop_ack(mesh_t *m, uint8_t to, uint8_t type,
                         uint16_t seq, uint64_t now)
{
    uint8_t pl[3] = {
        type,
        (uint8_t)(seq & 0xFF),
        (uint8_t)(seq >> 8),
    };
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(m->network_id, &m->mic, buf,
                                  MESH_FT_HOP_ACK, m->node_id, to,
                                  ++m->up_seq, 0, pl, sizeof(pl));
    if (len && mesh_txq_raw(m, buf, (uint8_t)len, now))
        m->stats.hop_acks_sent++;
}

/* ---- receive paths ---- */

void mesh_cmd_execute(mesh_t *m, const mesh_frame_t *f, uint64_t now)
{
    uint16_t cmd_id = (uint16_t)(f->payload[0] |
                                 ((uint16_t)f->payload[1] << 8));

    if (exec_seen(m, cmd_id)) {
        /* Re-flood of a command we already ran: the base must have
         * missed our ACK — re-ACK without re-executing (§6 step 4). */
        send_cmd_ack(m, cmd_id, 0, now);
        return;
    }

    bool ok = false;
    if (m->cb.on_command)
        ok = m->cb.on_command(m->cb.user, cmd_id,
                              f->payload + MESH_CMD_HDR,
                              (uint8_t)(f->plen - MESH_CMD_HDR));
    if (!ok)
        return; /* rejected: no ACK; the base will re-flood */

    exec_insert(m, cmd_id);
    m->stats.cmds_executed++;
    send_cmd_ack(m, cmd_id, 0, now);
}

void mesh_up_rx(mesh_t *m, const mesh_frame_t *f, uint64_t now)
{
    if (f->dst != m->node_id)
        return; /* overheard traffic between other nodes */

    /* ACK the hop even for duplicates: our previous HOP_ACK was lost. */
    send_hop_ack(m, f->src, f->type, f->seq, now);

    if (up_seen(m, f->src, f->type, f->seq)) {
        m->stats.up_dedup_drops++;
        return;
    }

    if (m->is_root) {
        if (f->type == MESH_FT_CMD_ACK) {
            if (f->plen < 4)
                return;
            uint16_t cmd_id = (uint16_t)(f->payload[0] |
                                         ((uint16_t)f->payload[1] << 8));
            if (m->cb.on_command_ack)
                m->cb.on_command_ack(m->cb.user, f->payload[2], cmd_id,
                                     f->payload[3]);
        } else if (f->type == MESH_FT_TELEM) {
            mesh_telem_deliver_root(m, f);
        }
        return;
    }

    /* Relay one hop closer to the root. */
    if (f->ttl == 0)
        return; /* hop budget exhausted */
    if (m->parent == MESH_ID_NONE) {
        m->stats.up_dropped_no_parent++;
        return;
    }
    uint16_t seq = ++m->up_seq;
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(m->network_id, &m->mic, buf, f->type,
                                  m->node_id, m->parent, seq,
                                  (uint8_t)(f->ttl - 1), f->payload,
                                  f->plen);
    if (!len)
        return;
    uint64_t due = now + mesh_rand_below(m, MESH_UP_JITTER_US);
    if (mesh_txq_unicast(m, buf, (uint8_t)len, due, m->parent, f->type,
                         seq))
        m->stats.up_relayed++;
}

void mesh_hop_ack_rx(mesh_t *m, const mesh_frame_t *f)
{
    if (f->dst != m->node_id || f->plen < 3)
        return;
    uint8_t type = f->payload[0];
    uint16_t seq = (uint16_t)(f->payload[1] |
                              ((uint16_t)f->payload[2] << 8));
    if (mesh_txq_on_hop_ack(m, f->src, type, seq)) {
        if (f->src == m->parent)
            m->parent_fail = 0; /* link demonstrably alive */
    }
}
