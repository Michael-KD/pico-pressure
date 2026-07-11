/*
 * txqueue.c — pending-transmission queue.
 *
 * Every frame the library sends flows through here and fires inside
 * mesh_poll(), which keeps the radio driver called from exactly one
 * place and makes randomized backoffs, per-hop retransmits, and
 * exhaustion detection uniform.
 *
 * Entry lifecycles:
 *   RAW      one transmission at due_us, then freed.
 *   BEACON   like RAW, but the frame is built at fire time so it carries
 *            the node's *current* path cost; skipped if unattached.
 *   UNICAST  transmitted, then retransmitted every retry_interval until a
 *            matching HOP_ACK frees it; after the last send it lingers
 *            one more interval so the final ACK can still arrive, then
 *            counts as exhausted (feeding parent-liveness detection).
 */
#include "mesh_internal.h"

mesh_txq_entry_t *mesh_txq_alloc(mesh_t *m)
{
    for (unsigned i = 0; i < MESH_TXQ_DEPTH; i++) {
        if (!m->txq[i].in_use) {
            mesh_txq_entry_t *e = &m->txq[i];
            memset(e, 0, sizeof(*e));
            e->in_use = true;
            return e;
        }
    }
    m->stats.txq_full++;
    return NULL;
}

bool mesh_txq_raw(mesh_t *m, const uint8_t *buf, uint8_t len, uint64_t due)
{
    mesh_txq_entry_t *e = mesh_txq_alloc(m);
    if (!e)
        return false;
    e->kind = MESH_TXK_RAW;
    memcpy(e->buf, buf, len);
    e->len = len;
    e->attempts_left = 1;
    e->due_us = due;
    return true;
}

bool mesh_txq_unicast(mesh_t *m, const uint8_t *buf, uint8_t len,
                      uint64_t due, uint8_t ack_from, uint8_t ack_type,
                      uint16_t ack_seq)
{
    mesh_txq_entry_t *e = mesh_txq_alloc(m);
    if (!e)
        return false;
    e->kind = MESH_TXK_UNICAST;
    memcpy(e->buf, buf, len);
    e->len = len;
    e->attempts_left = 1 + MESH_HOP_RETRIES;
    e->await_hop_ack = true;
    e->ack_from = ack_from;
    e->ack_type = ack_type;
    e->ack_seq = ack_seq;
    e->retry_interval_us = MESH_HOP_RETRY_US;
    e->due_us = due;
    return true;
}

bool mesh_txq_beacon(mesh_t *m, uint16_t beacon_seq, uint64_t due)
{
    mesh_txq_entry_t *e = mesh_txq_alloc(m);
    if (!e)
        return false;
    e->kind = MESH_TXK_BEACON;
    e->beacon_seq = beacon_seq;
    e->attempts_left = 1;
    e->due_us = due;
    return true;
}

static void txq_exhausted(mesh_t *m, mesh_txq_entry_t *e)
{
    e->in_use = false;
    m->stats.hop_exhausted++;
    if (m->parent != MESH_ID_NONE && e->ack_from == m->parent) {
        m->parent_fail++;
        if (m->parent_fail >= MESH_PARENT_FAIL_LIMIT)
            mesh_tree_parent_dead(m);
    }
}

/* Window class of a queued entry, derived from what it carries (§4). */
static uint8_t txq_win(const mesh_txq_entry_t *e)
{
    if (e->kind == MESH_TXK_BEACON)
        return MESH_WIN_BEACON;
    switch (e->buf[MESH_OFF_TYPE]) {
    case MESH_FT_BEACON:
        return MESH_WIN_BEACON;
    case MESH_FT_CMD:
    case MESH_FT_CMD_ACK:
        return MESH_WIN_CMD; /* "commands and command ACK collection" */
    case MESH_FT_TELEM:
        return MESH_WIN_TELEM;
    case MESH_FT_DATA:
    case MESH_FT_DATA_ACK:
    case MESH_FT_SESSION:
        return MESH_WIN_BULK;
    default:
        return MESH_WIN_ANY; /* HOP_ACK: immediate link response */
    }
}

/* Re-jitter for an entry deferred to a window start, so every deferred
 * sender does not fire at the same instant. */
static uint32_t txq_defer_jitter(mesh_t *m, const mesh_txq_entry_t *e)
{
    if (e->await_hop_ack)
        return mesh_rand_below(m, MESH_UP_JITTER_US);
    if (e->kind == MESH_TXK_BEACON)
        return mesh_rand_below(m, MESH_BEACON_BACKOFF_US);
    if (e->buf[MESH_OFF_TYPE] == MESH_FT_CMD && !m->is_root)
        return mesh_rand_below(m, MESH_FLOOD_BACKOFF_US); /* flood relay */
    return 0; /* root's own beacon/flood: fire at window start */
}

static void txq_fire(mesh_t *m, mesh_txq_entry_t *e, uint64_t now)
{
    if (e->kind == MESH_TXK_BEACON) {
        e->len = mesh_beacon_build(m, e->beacon_seq, e->buf);
        if (e->len == 0) { /* unattached: relaying suppressed (§5.4) */
            e->in_use = false;
            return;
        }
    }

    if (e->await_hop_ack && e->attempts_left == 0) {
        /* Final post-send wait elapsed without a HOP_ACK. */
        txq_exhausted(m, e);
        return;
    }

    if (e->await_hop_ack && e->attempts_left <= MESH_HOP_RETRIES)
        m->stats.hop_retries++; /* any send after the first */

    if (m->radio.transmit(m->radio.ctx, e->buf, e->len, MESH_PROFILE_CONTROL))
        m->stats.tx_frames++;
    else
        m->stats.tx_radio_fail++;

    if (e->await_hop_ack) {
        e->attempts_left--;
        e->due_us = now + e->retry_interval_us +
                    mesh_rand_below(m, MESH_HOP_RETRY_JITTER_US);
    } else {
        e->in_use = false;
    }
}

uint64_t mesh_txq_poll(mesh_t *m, uint64_t now)
{
    uint64_t next = MESH_NO_EVENT;
    for (unsigned i = 0; i < MESH_TXQ_DEPTH; i++) {
        mesh_txq_entry_t *e = &m->txq[i];
        if (!e->in_use)
            continue;
        if (e->due_us <= now) {
            /* Superframe gating (§4): an entry due outside its window
             * is rescheduled to the window's next opening. */
            uint64_t gate = mesh_sched_gate(m, txq_win(e), now);
            if (gate > now) {
                e->due_us = gate + txq_defer_jitter(m, e);
                m->stats.tx_deferred++;
            } else {
                txq_fire(m, e, now);
            }
        }
        if (e->in_use && e->due_us < next)
            next = e->due_us;
    }
    return next;
}

bool mesh_txq_on_hop_ack(mesh_t *m, uint8_t from, uint8_t type,
                         uint16_t seq)
{
    for (unsigned i = 0; i < MESH_TXQ_DEPTH; i++) {
        mesh_txq_entry_t *e = &m->txq[i];
        if (e->in_use && e->await_hop_ack && e->ack_from == from &&
            e->ack_type == type && e->ack_seq == seq) {
            e->in_use = false;
            return true;
        }
    }
    return false;
}
