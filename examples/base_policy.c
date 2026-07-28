/*
 * base_policy.c — see base_policy.h.
 *
 * Everything here is base-side policy over library primitives: the
 * library never learns what an "index" or a "campaign" is (§7.1 /
 * principle 6). The LIST convention parsed here is the example
 * fleet's: a LIST session returns a text line
 * "node N: evt 1, <bytes> B, crc16 <hex4>". The campaign uses a fresh
 * session id from the reserved LIST range per node, so a late chunk
 * from the previous node's index can never contaminate the current
 * one, and the text is parsed only once its EOF (a short chunk) has
 * arrived — a partial line would silently truncate the CRC field.
 */
#include "base_policy.h"

#include <stdio.h>
#include <string.h>

/* No-progress window before a re-FETCH: two superframes when
 * scheduled (a chunk may legally wait out a whole sleep segment),
 * a few seconds on a bench fleet where chunks stream immediately. */
static uint64_t bp_timeout(const base_policy_t *p)
{
    if (p->m->sched.enabled)
        return 2 * p->m->sched.mode.period_us;
    return 8000000ull;
}

static uint8_t bp_alloc_session(base_policy_t *p)
{
    uint8_t s = p->next_session++;
    if (p->next_session < 128 || p->next_session >= BP_LIST_SESSION0)
        p->next_session = 128; /* stay inside the campaign FETCH range */
    return s;
}

static uint8_t bp_alloc_list_session(base_policy_t *p)
{
    uint8_t s = (uint8_t)(BP_LIST_SESSION0 + p->list_counter);
    p->list_counter = (uint8_t)((p->list_counter + 1) %
                                (255 - BP_LIST_SESSION0));
    return s;
}

void bp_init(base_policy_t *p, mesh_t *m, uint8_t *buf, uint32_t buf_max)
{
    memset(p, 0, sizeof(*p));
    p->m = m;
    p->buf = buf;
    p->buf_max = buf_max;
    p->next_session = 128;
}

/* ---- fleet mode switching (phase 1 / 4) ---- */

static void bp_flood_mode(base_policy_t *p, uint64_t now)
{
    uint8_t pl[FLEET_MODE_PLEN];
    fleet_mode_cmd(pl, p->mode_id, p->mode_at);
    /* Reusing the same cmd id makes re-floods re-ACK-only (§6). */
    uint16_t id = mesh_root_command(p->m, MESH_ID_BROADCAST, pl,
                                    sizeof(pl), p->mode_cmd_id, now);
    if (id != 0)
        p->mode_cmd_id = id;
    p->next_flood_us = now + (p->m->sched.enabled
                                  ? p->m->sched.mode.period_us
                                  : 3000000ull);
}

/* Arm the root's own switch and launch the fleet flood as one unit.
 * False = not launched (another switch pending, or the tx queue was
 * full — in which case the armed switch is CANCELLED so the base can
 * never change modes alone and desynchronize from its fleet). The
 * caller retries. */
static bool bp_launch_mode(base_policy_t *p, uint8_t mode_id,
                           uint64_t now)
{
    mesh_sched_mode_t md;
    if (!fleet_mode_by_id(mode_id, &md))
        return false;
    /* Two superframes of lead time: the flood needs at least one
     * command window to reach the deep nodes, plus margin for lossy
     * relays (re-floods run each superframe until then). */
    uint64_t at = now + 2 * p->m->sched.mode.period_us;
    if (!mesh_sched_set_mode(p->m, &md, at))
        return false; /* e.g. an operator `mode` still pending */
    p->mode_id = mode_id;
    p->mode_at = at;
    p->mode_cmd_id = 0;
    p->refloods_left = BP_MODE_REFLOODS;
    p->acked_mask = 0;
    bp_flood_mode(p, now);
    if (p->mode_cmd_id == 0) {
        mesh_sched_cancel_mode(p->m);
        return false;
    }
    return true;
}

static void bp_enter_mode_phase(base_policy_t *p, bp_state_t state,
                                uint64_t now)
{
    p->state = state;
    p->mode_at = 0; /* not launched yet; bp_poll launches/retries */
    p->next_flood_us = now;
}

/* ---- per-node transfers (phases 2/3) ---- */

static void bp_issue_fetch(base_policy_t *p, uint64_t now)
{
    /* Resume from the first gap (initial issue: gap = 0). */
    unsigned first = 0;
    while (first < p->nchunks && ((p->got_mask >> first) & 1))
        first++;
    uint32_t off = (uint32_t)first * p->chunk;
    mesh_root_fetch(p->m, p->nodes[p->cur], p->session, 0, off,
                    p->total - off, p->chunk, now);
    p->res[p->cur].attempts++;
    p->last_progress_us = now;
}

static void bp_begin_list(base_policy_t *p, uint64_t now)
{
    bp_result_t *r = &p->res[p->cur];
    memset(r, 0, sizeof(*r));
    r->attempted = true;
    p->state = BP_LIST;
    p->session = bp_alloc_list_session(p);
    /* Chunk 48 with a 96-byte cap: any realistic index line (~35
     * chars) ends in a short chunk, which is the EOF marker the parse
     * waits for. */
    p->chunk = 48;
    p->total = sizeof(p->list_text);
    p->nchunks = (p->total + p->chunk - 1u) / p->chunk;
    p->got_mask = 0;
    p->got = 0;
    p->list_eof = false;
    memset(p->list_text, 0, sizeof(p->list_text));
    mesh_root_fetch(p->m, p->nodes[p->cur], p->session, 0, 0, p->total,
                    p->chunk, now);
    r->attempts++;
    p->last_progress_us = now;
}

static void bp_begin_fetch(base_policy_t *p, uint64_t now)
{
    bp_result_t *r = &p->res[p->cur];
    p->state = BP_FETCH;
    p->session = bp_alloc_session(p);
    p->chunk = MESH_BULK_CHUNK;
    p->total = r->bytes;
    if (p->total > p->buf_max)
        p->total = p->buf_max;
    if (p->total > (uint32_t)BP_MAX_CHUNKS * p->chunk)
        p->total = (uint32_t)BP_MAX_CHUNKS * p->chunk;
    p->nchunks = (p->total + p->chunk - 1u) / p->chunk;
    p->got_mask = 0;
    p->got = 0;
    r->attempts = 0;
    bp_issue_fetch(p, now);
}

static void bp_next_node(base_policy_t *p, uint64_t now)
{
    p->cur++;
    if (p->cur < p->n_nodes) {
        bp_begin_list(p, now);
        return;
    }
    if (p->switch_modes) {
        bp_enter_mode_phase(p, BP_MODE_OUT, now);
        return;
    }
    p->state = BP_IDLE;
}

/* Parse the completed index text; true when usable. */
static bool bp_parse_list(base_policy_t *p)
{
    bp_result_t *r = &p->res[p->cur];
    unsigned bytes;
    unsigned crc;
    const char *at = strstr(p->list_text, "evt");
    if (!at)
        return false;
    if (sscanf(at, "evt %*u, %u B, crc16 %4x", &bytes, &crc) != 2)
        return false;
    if (bytes == 0)
        return false;
    r->listed = true;
    r->bytes = bytes;
    r->crc_want = (uint16_t)crc;
    return true;
}

void bp_on_bulk_data(base_policy_t *p, uint8_t session,
                     uint16_t chunk_index, const uint8_t *data,
                     uint8_t len, uint64_t now)
{
    if (p->state == BP_LIST && session == p->session) {
        uint32_t off = (uint32_t)chunk_index * p->chunk;
        if (off + len <= sizeof(p->list_text) - 1)
            memcpy(&p->list_text[off], data, len);
        p->last_progress_us = now;
        if (len < p->chunk)
            p->list_eof = true;
        /* Parse only a COMPLETE line: chunk boundaries can split the
         * CRC field, and %4x would silently accept 1-3 digits. */
        if (p->list_eof && bp_parse_list(p))
            bp_begin_fetch(p, now);
        return;
    }

    if (p->state != BP_FETCH || session != p->session ||
        chunk_index >= p->nchunks)
        return;

    uint32_t off = (uint32_t)chunk_index * p->chunk;
    if (!((p->got_mask >> chunk_index) & 1)) {
        p->got_mask |= 1ull << chunk_index;
        p->got++;
        if (off + len <= p->buf_max)
            memcpy(&p->buf[off], data, len);
    }
    p->last_progress_us = now;

    if (p->got == p->nchunks) {
        bp_result_t *r = &p->res[p->cur];
        r->crc_got = mesh_crc16(p->buf, p->total);
        r->ok = (r->crc_got == r->crc_want);
        bp_next_node(p, now);
    }
}

void bp_on_command_ack(base_policy_t *p, uint8_t origin, uint16_t cmd_id)
{
    if ((p->state != BP_MODE_IN && p->state != BP_MODE_OUT) ||
        cmd_id == 0 || cmd_id != p->mode_cmd_id)
        return;
    for (unsigned i = 0; i < p->n_nodes; i++) {
        if (p->nodes[i] == origin)
            p->acked_mask |= 1u << i;
    }
}

bool bp_start(base_policy_t *p, const uint8_t *nodes, unsigned n_nodes,
              bool switch_modes, uint8_t restore_mode, uint64_t now)
{
    mesh_sched_mode_t md;
    if (bp_busy(p) || n_nodes == 0 || n_nodes > BP_MAX_NODES || !p->buf)
        return false;
    if (switch_modes &&
        (!p->m->sched.enabled || !fleet_mode_by_id(restore_mode, &md)))
        return false; /* no scheduler = no modes; unknown restore id */

    memcpy(p->nodes, nodes, n_nodes);
    p->n_nodes = n_nodes;
    p->cur = 0;
    memset(p->res, 0, sizeof(p->res));
    p->switch_modes = switch_modes;
    p->restore_mode = restore_mode;

    if (switch_modes)
        bp_enter_mode_phase(p, BP_MODE_IN, now);
    else
        bp_begin_list(p, now);
    return true;
}

void bp_poll(base_policy_t *p, uint64_t now)
{
    switch (p->state) {
    case BP_IDLE:
        return;

    case BP_MODE_IN:
    case BP_MODE_OUT: {
        if (p->mode_at == 0) {
            /* Not launched yet (or a previous launch attempt failed —
             * pending operator switch, full tx queue): keep retrying.
             * The root never switches without a launched flood. */
            if (now >= p->next_flood_us) {
                uint8_t id = (p->state == BP_MODE_IN)
                                 ? FLEET_MODE_RETRIEVAL
                                 : p->restore_mode;
                if (!bp_launch_mode(p, id, now))
                    p->next_flood_us = now + 5000000ull;
            }
            return;
        }
        uint32_t all = (p->n_nodes >= 32)
                           ? 0xFFFFFFFFu
                           : ((1u << p->n_nodes) - 1u);
        bool acked = (p->acked_mask & all) == all;
        if (!acked && p->refloods_left > 0 && now >= p->next_flood_us) {
            p->refloods_left--;
            bp_flood_mode(p, now);
        }
        if (now >= p->mode_at) {
            /* The switch is in effect. Unacked nodes are recorded (see
             * bp_status): with the MIC bound to the absolute epoch a
             * straggler still hears later floods, so a follow-up
             * `mode` command recovers it. */
            if (p->state == BP_MODE_IN)
                bp_begin_list(p, now);
            else
                p->state = BP_IDLE;
        }
        return;
    }

    case BP_LIST:
        if (p->cur >= p->n_nodes) { /* defensive: cannot happen */
            p->state = BP_IDLE;
            return;
        }
        if (now - p->last_progress_us > bp_timeout(p)) {
            bp_result_t *r = &p->res[p->cur];
            if (r->attempts > BP_FETCH_RETRIES) {
                bp_next_node(p, now); /* unreachable/empty: move on */
                return;
            }
            r->attempts++;
            mesh_root_fetch(p->m, p->nodes[p->cur], p->session, 0, 0,
                            p->total, p->chunk, now);
            p->last_progress_us = now;
        }
        return;

    case BP_FETCH:
        if (p->cur >= p->n_nodes) { /* defensive: cannot happen */
            p->state = BP_IDLE;
            return;
        }
        if (now - p->last_progress_us > bp_timeout(p)) {
            if (p->res[p->cur].attempts > BP_FETCH_RETRIES) {
                bp_next_node(p, now); /* give up; result says how far */
                return;
            }
            bp_issue_fetch(p, now); /* re-FETCH from the first gap */
        }
        return;
    }
}

void bp_status(const base_policy_t *p, char *out, unsigned max)
{
    static const char *names[] = { "idle", "mode-in", "list", "fetch",
                                   "mode-out" };
    unsigned n = 0;
    n += (unsigned)snprintf(out + n, max - n, "campaign: %s",
                            names[p->state]);
    if (p->state == BP_MODE_IN || p->state == BP_MODE_OUT) {
        unsigned acks = 0;
        for (unsigned i = 0; i < p->n_nodes; i++)
            acks += (p->acked_mask >> i) & 1u;
        if (p->mode_at == 0)
            n += (unsigned)snprintf(out + n, (n < max) ? max - n : 0,
                                    " (waiting to launch)");
        else
            n += (unsigned)snprintf(out + n, (n < max) ? max - n : 0,
                                    " (mode %u, acks %u/%u)",
                                    p->mode_id, acks, p->n_nodes);
    }
    if ((p->state == BP_LIST || p->state == BP_FETCH) &&
        p->cur < p->n_nodes)
        n += (unsigned)snprintf(out + n, (n < max) ? max - n : 0,
                                " node %u (%u/%u chunks, attempt %u)",
                                p->nodes[p->cur], p->got, p->nchunks,
                                p->res[p->cur].attempts);
    n += (unsigned)snprintf(out + n, (n < max) ? max - n : 0, "\n");
    for (unsigned i = 0; i < p->n_nodes && n < max; i++) {
        const bp_result_t *r = &p->res[i];
        if (!r->attempted)
            continue;
        n += (unsigned)snprintf(out + n, max - n,
                                "  node %u: %s (%lu B, crc %04X/%04X, "
                                "%u attempts)\n", p->nodes[i],
                                r->ok ? "OK"
                                      : (r->listed ? "INCOMPLETE"
                                                   : "NO INDEX"),
                                (unsigned long)r->bytes, r->crc_got,
                                r->crc_want, r->attempts);
    }
}
