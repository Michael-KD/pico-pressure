/*
 * bulk.c — bulk data retrieval, first cut (§7.3): FETCH sessions with
 * single-chunk tree unicast.
 *
 * The base floods a FETCH (SESSION frame) at a target node; the target
 * streams the requested byte range up the tree as DATA chunks riding
 * the same per-hop-reliable unicast engine as ACKs and telemetry, one
 * chunk in flight at a time. The base tracks received chunk indices
 * and re-FETCHes missing sub-ranges under the same session id —
 * resumption is base-side state (design principle 6); the node keeps
 * only a cursor. Store-and-forward burst pipelining (§7.3 in full) is
 * a later milestone; DATA_ACK stays reserved for it.
 *
 * FETCH payload: session(1) base_off(4 LE) req_off(4 LE) req_len(4 LE)
 *                chunk_size(1)
 * DATA payload:  session(1) chunk_index(2 LE) crc16(2 LE) data(...)
 *
 * chunk_index is relative to base_off, so a re-FETCH of a sub-range
 * produces chunks at the same indices as the original request. The
 * per-chunk CRC16 guards against *source-side* corruption (an SD read
 * gone wrong, §10.1); transit integrity is already the MIC's job.
 */
#include "mesh_internal.h"

/* CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF). Bitwise: inputs are
 * at most one chunk, and it saves a 512-byte table. */
uint16_t mesh_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)((uint16_t)data[i] << 8);
        for (unsigned b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
    }
    return crc;
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static uint32_t get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* True while the chunk we last sent still sits in the TX queue awaiting
 * its per-hop resolution (HOP_ACK or retry exhaustion frees it). */
static bool chunk_in_flight(const mesh_t *m, uint16_t seq)
{
    for (unsigned i = 0; i < MESH_TXQ_DEPTH; i++) {
        const mesh_txq_entry_t *e = &m->txq[i];
        if (e->in_use && e->await_hop_ack &&
            e->ack_type == MESH_FT_DATA && e->ack_seq == seq)
            return true;
    }
    return false;
}

bool mesh_root_fetch(mesh_t *m, uint8_t target, uint8_t session,
                     uint32_t base_offset, uint32_t req_offset,
                     uint32_t req_length, uint8_t chunk_size,
                     uint64_t now_us)
{
    if (!m || !m->initialized || !m->is_root)
        return false;
    if (target == MESH_ID_ROOT || target == MESH_ID_BROADCAST)
        return false;
    if (chunk_size == 0 || chunk_size > MESH_MAX_BULK_DATA)
        return false;
    if (req_length == 0 || req_offset < base_offset)
        return false;
    if ((req_offset - base_offset) % chunk_size != 0)
        return false; /* chunk indices must stay aligned (§7.3) */
    if (req_length > UINT32_MAX - req_offset)
        return false;
    if (m->sched.enabled && m->sched.mode.bulk_win_us == 0)
        return false; /* this mode never opens a bulk window */

    uint16_t seq = ++m->flood_seq;
    (void)mesh_flood_seen(m, seq); /* pre-seed: ignore our own echoes */

    uint8_t body[MESH_FETCH_PLEN];
    body[0] = session;
    put_u32(&body[1], base_offset);
    put_u32(&body[5], req_offset);
    put_u32(&body[9], req_length);
    body[13] = chunk_size;

    uint8_t buf[MESH_MAX_FRAME];
    size_t flen = mesh_frame_build(m->network_id, &m->mic, buf,
                                   MESH_FT_SESSION, m->node_id, target,
                                   seq, MESH_FLOOD_TTL, body,
                                   MESH_FETCH_PLEN);
    return flen != 0 && mesh_txq_raw(m, buf, (uint8_t)flen, now_us);
}

void mesh_bulk_fetch_execute(mesh_t *m, const mesh_frame_t *f,
                             uint64_t now)
{
    /* plen was checked against MESH_FETCH_PLEN in mesh_flood_rx. */
    if (!m->cb.on_bulk_read)
        return; /* this application serves no bulk data */

    const uint8_t *p = f->payload;
    uint8_t session = p[0];
    uint32_t base = get_u32(&p[1]);
    uint32_t off = get_u32(&p[5]);
    uint32_t rlen = get_u32(&p[9]);
    uint8_t chunk = p[13];
    if (chunk == 0 || chunk > MESH_MAX_BULK_DATA || rlen == 0)
        return;
    if (off < base || (off - base) % chunk != 0)
        return;
    if (rlen > UINT32_MAX - off)
        return;

    /* A FETCH replaces any session in progress — the base drives, and
     * re-FETCHing a missing range under the same session id is the
     * §7.3 resumption mechanism. A chunk already in the queue from the
     * old range delivers normally; the base's bitmap absorbs it. */
    m->bulk.active = true;
    m->bulk.pending = false;
    m->bulk.session = session;
    m->bulk.chunk_size = chunk;
    m->bulk.base_off = base;
    m->bulk.cursor = off;
    m->bulk.end_off = off + rlen;
    m->stats.bulk_fetches_rx++;

    mesh_bulk_pump(m, now); /* first chunk without waiting for a poll */
}

void mesh_bulk_pump(mesh_t *m, uint64_t now)
{
    mesh_bulk_t *b = &m->bulk;
    if (!b->active)
        return;
    if (b->pending) {
        if (chunk_in_flight(m, b->pending_seq))
            return; /* strictly one chunk outstanding */
        b->pending = false;
    }
    if (b->cursor >= b->end_off) {
        b->active = false; /* requested range complete */
        return;
    }
    if (m->parent == MESH_ID_NONE)
        return; /* stalled; resumes when re-attached */
    if (m->sched.enabled && m->sched.mode.bulk_win_us == 0)
        return; /* dormant until a bulk-capable mode arrives */

    uint32_t remaining = b->end_off - b->cursor;
    uint8_t want = b->chunk_size;
    if (remaining < want)
        want = (uint8_t)remaining;

    uint8_t body[MESH_MAX_PAYLOAD];
    uint8_t got = m->cb.on_bulk_read(m->cb.user, b->session, b->cursor,
                                     &body[MESH_BULK_DATA_HDR], want);
    if (got > want)
        got = want;
    if (got == 0) {
        b->active = false; /* end of data before end of range */
        return;
    }

    uint16_t idx = (uint16_t)((b->cursor - b->base_off) / b->chunk_size);
    uint16_t crc = mesh_crc16(&body[MESH_BULK_DATA_HDR], got);
    body[0] = b->session;
    body[1] = (uint8_t)(idx & 0xFF);
    body[2] = (uint8_t)(idx >> 8);
    body[3] = (uint8_t)(crc & 0xFF);
    body[4] = (uint8_t)(crc >> 8);

    if (!mesh_up_send_at(m, MESH_FT_DATA, body,
                         (uint8_t)(MESH_BULK_DATA_HDR + got),
                         MESH_FLOOD_TTL, now))
        return; /* queue full — retried on the next poll */

    b->pending = true;
    b->pending_seq = m->up_seq; /* the seq mesh_up_send_at just stamped */
    m->stats.bulk_chunks_sent++;

    if (got < want)
        b->active = false; /* short read: that was the final chunk */
    else
        b->cursor += got;
}

void mesh_bulk_deliver_root(mesh_t *m, const mesh_frame_t *f)
{
    if (f->plen < MESH_BULK_DATA_HDR)
        return;
    const uint8_t *p = f->payload;
    uint8_t dlen = (uint8_t)(f->plen - MESH_BULK_DATA_HDR);
    uint16_t idx = (uint16_t)(p[1] | ((uint16_t)p[2] << 8));
    uint16_t crc = (uint16_t)(p[3] | ((uint16_t)p[4] << 8));
    if (mesh_crc16(&p[MESH_BULK_DATA_HDR], dlen) != crc) {
        /* Source-side corruption (§10.1): drop, so the base's bitmap
         * keeps this index missing and a re-FETCH re-reads the source. */
        m->stats.bulk_crc_drops++;
        return;
    }
    m->stats.bulk_chunks_root++;
    if (m->cb.on_bulk_data)
        m->cb.on_bulk_data(m->cb.user, p[0], idx,
                           &p[MESH_BULK_DATA_HDR], dlen);
}
