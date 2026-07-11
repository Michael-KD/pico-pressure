/*
 * telemetry.c — status reports up the tree (§5.3).
 *
 * The library prepends each report with the sender's routing context —
 * parent and its top-3 neighbors with link ETX — which is what lets the
 * base station assemble the global topology map. The application payload
 * (battery, storage, GPS lock, fault codes…) is opaque to the library.
 *
 * TELEM payload: origin(1) parent(1) [nbr_id(1) etx_q8(2 LE)] x3 app...
 */
#include "mesh_internal.h"

bool mesh_node_telemetry(mesh_t *m, const uint8_t *payload, uint8_t len,
                         uint64_t now_us)
{
    if (!m || !m->initialized || m->is_root)
        return false;
    if (len > MESH_MAX_TELEM_PAYLOAD || (len > 0 && payload == NULL))
        return false;

    uint8_t body[MESH_MAX_PAYLOAD];
    body[0] = m->node_id;
    body[1] = m->parent;

    /* Top-3 fresh neighbors by current ETX, excluding the parent (which
     * is already reported). */
    const mesh_neighbor_t *top[MESH_TELEM_NBRS] = { 0 };
    for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
        const mesh_neighbor_t *n = &m->nbrs[i];
        if (!n->in_use || n->id == m->parent || !mesh_nbr_fresh(m, n))
            continue;
        uint16_t etx = mesh_nbr_etx_now(m, n);
        if (etx == MESH_COST_INF)
            continue;
        for (unsigned s = 0; s < MESH_TELEM_NBRS; s++) {
            if (!top[s] || etx < mesh_nbr_etx_now(m, top[s])) {
                for (unsigned t = MESH_TELEM_NBRS - 1; t > s; t--)
                    top[t] = top[t - 1];
                top[s] = n;
                break;
            }
        }
    }
    for (unsigned s = 0; s < MESH_TELEM_NBRS; s++) {
        uint8_t *slot = &body[2 + s * 3];
        if (top[s]) {
            uint16_t etx = mesh_nbr_etx_now(m, top[s]);
            slot[0] = top[s]->id;
            slot[1] = (uint8_t)(etx & 0xFF);
            slot[2] = (uint8_t)(etx >> 8);
        } else {
            slot[0] = MESH_ID_NONE;
            slot[1] = 0;
            slot[2] = 0;
        }
    }

    if (len > 0)
        memcpy(&body[MESH_TELEM_HDR], payload, len);

    if (!mesh_up_send(m, MESH_FT_TELEM, body,
                      (uint8_t)(MESH_TELEM_HDR + len), MESH_FLOOD_TTL,
                      now_us))
        return false;
    m->stats.telem_sent++;
    return true;
}

void mesh_telem_deliver_root(mesh_t *m, const mesh_frame_t *f)
{
    if (f->plen < MESH_TELEM_HDR || !m->cb.on_telemetry)
        return;

    mesh_telem_info_t info;
    info.origin = f->payload[0];
    info.parent = f->payload[1];
    for (unsigned s = 0; s < MESH_TELEM_NBRS; s++) {
        const uint8_t *slot = &f->payload[2 + s * 3];
        info.nbr_id[s] = slot[0];
        info.nbr_etx_q8[s] = (uint16_t)(slot[1] |
                                        ((uint16_t)slot[2] << 8));
    }
    m->cb.on_telemetry(m->cb.user, &info, f->payload + MESH_TELEM_HDR,
                       (uint8_t)(f->plen - MESH_TELEM_HDR));
}
