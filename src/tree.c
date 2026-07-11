/*
 * tree.c — collection-tree parent selection (§5.2–§5.4).
 *
 * path_cost(me) = min over usable neighbors n of (adv_cost(n) + ETX(me,n))
 *
 * Rules implemented here:
 *   - Loop guard: never select a parent whose advertised cost is >= our
 *     own current cost.
 *   - Hysteresis: a challenger must beat the incumbent by
 *     MESH_HYST_MARGIN_Q8 for MESH_HYST_EPOCHS consecutive beacon epochs.
 *   - Poisoning: a stale or dead parent detaches us; while unattached our
 *     cost is infinity and beacon relaying stops (see mesh_beacon_build).
 *   - Attachment from the unattached state is immediate (no hysteresis).
 */
#include "mesh_internal.h"

static void reset_challenger(mesh_t *m)
{
    m->cand_id = MESH_ID_NONE;
    m->cand_epochs = 0;
}

static void announce_parent_change(mesh_t *m, uint8_t old_parent)
{
    m->stats.parent_changes++;
    if (m->cb.on_parent_change)
        m->cb.on_parent_change(m->cb.user, old_parent, m->parent);
}

static void attach(mesh_t *m, const mesh_neighbor_t *n, uint16_t total)
{
    uint8_t old = m->parent;
    m->parent = n->id;
    m->path_cost = total;
    m->hops = (n->hops == 0xFF) ? 0xFF : (uint8_t)(n->hops + 1);
    m->parent_fail = 0;
    reset_challenger(m);
    if (old != n->id)
        announce_parent_change(m, old);
}

static void detach(mesh_t *m)
{
    uint8_t old = m->parent;
    m->parent = MESH_ID_NONE;
    m->path_cost = MESH_COST_INF;
    m->hops = 0xFF;
    m->parent_fail = 0;
    reset_challenger(m);
    announce_parent_change(m, old);
}

/* Cost through a neighbor if it were our parent; INF if unusable. */
static uint16_t link_total(const mesh_t *m, const mesh_neighbor_t *n)
{
    if (!mesh_nbr_fresh(m, n) || n->adv_cost == MESH_COST_INF)
        return MESH_COST_INF;
    uint16_t etx = mesh_nbr_etx_now(m, n);
    if (etx == MESH_COST_INF)
        return MESH_COST_INF;
    return mesh_cost_add(n->adv_cost, etx);
}

void mesh_tree_evaluate(mesh_t *m)
{
    if (m->is_root) {
        m->parent = MESH_ID_NONE;
        m->path_cost = 0;
        m->hops = 0;
        return;
    }

    /* Refresh the incumbent; a stale/poisoned parent detaches us. */
    if (m->parent != MESH_ID_NONE) {
        mesh_neighbor_t *p = mesh_nbr_find(m, m->parent);
        uint16_t p_total = p ? link_total(m, p) : MESH_COST_INF;
        if (p_total == MESH_COST_INF) {
            detach(m);
        } else {
            m->path_cost = p_total;
            m->hops = (p->hops == 0xFF) ? 0xFF : (uint8_t)(p->hops + 1);
        }
    }

    /* Best candidate under the loop guard (adv_cost < our cost; while
     * unattached our cost is INF, so any finite-cost neighbor passes). */
    mesh_neighbor_t *best = NULL;
    uint16_t best_total = MESH_COST_INF;
    for (unsigned i = 0; i < MESH_MAX_NEIGHBORS; i++) {
        mesh_neighbor_t *n = &m->nbrs[i];
        if (!n->in_use || n->id == m->parent)
            continue;
        if (n->adv_cost >= m->path_cost)
            continue;
        uint16_t t = link_total(m, n);
        if (t < best_total) {
            best = n;
            best_total = t;
        }
    }

    if (m->parent == MESH_ID_NONE) {
        if (best)
            attach(m, best, best_total);
        return;
    }
    if (!best) {
        reset_challenger(m);
        return;
    }

    /* Challenger vs. incumbent, with margin and per-epoch persistence. */
    if (mesh_cost_add(best_total, MESH_HYST_MARGIN_Q8) > m->path_cost) {
        reset_challenger(m);
        return;
    }
    if (!m->have_hyst_epoch || m->last_hyst_epoch != m->net_epoch) {
        /* First evaluation in this epoch: tick the persistence counter. */
        m->have_hyst_epoch = true;
        m->last_hyst_epoch = m->net_epoch;
        if (m->cand_id == best->id)
            m->cand_epochs++;
        else {
            m->cand_id = best->id;
            m->cand_epochs = 1;
        }
    } else if (m->cand_id != best->id) {
        /* Challenger changed within the epoch: restart persistence. */
        m->cand_id = best->id;
        m->cand_epochs = 1;
    }
    if (m->cand_epochs >= MESH_HYST_EPOCHS)
        attach(m, best, best_total);
}

void mesh_tree_parent_dead(mesh_t *m)
{
    if (m->is_root || m->parent == MESH_ID_NONE)
        return;
    /* Demonstrated-dead link (§5.4 "missed ACKs"): erase its reception
     * history so it cannot be re-selected until beacons are heard again. */
    mesh_neighbor_t *p = mesh_nbr_find(m, m->parent);
    if (p) {
        p->beacon_bitmap = 0;
        p->etx_q8 = 0;
    }
    detach(m);
    mesh_tree_evaluate(m); /* may re-attach via a surviving neighbor */
}
