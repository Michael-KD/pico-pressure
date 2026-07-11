/* Unit tests: parent selection — attach, min-cost choice, hysteresis,
 * loop guard, stale-parent poisoning, dead-link failover. */
#include "mesh_test_helpers.h"
#include "test_util.h"

static tx_capture_t cap;

static unsigned n_parent_changes;
static uint8_t last_old_parent, last_new_parent;

static void on_parent_change(void *user, uint8_t old_parent,
                             uint8_t new_parent)
{
    (void)user;
    n_parent_changes++;
    last_old_parent = old_parent;
    last_new_parent = new_parent;
}

static void make_node(mesh_t *m, uint8_t id)
{
    mesh_cfg_t cfg = test_cfg(id, &cap);
    cfg.cb.on_parent_change = on_parent_change;
    T_CHECK(mesh_init(m, &cfg));
    m->have_epoch = true;
    m->net_epoch = 1;
    n_parent_changes = 0;
}

static void test_attach_immediate(void)
{
    mesh_t m;
    make_node(&m, 5);
    T_CHECK_EQ(mesh_parent(&m), MESH_ID_NONE);
    T_CHECK_EQ(mesh_path_cost(&m), MESH_COST_INF);

    fab_nbr(&m, 1, 0, MESH_ETX_ONE, 0); /* the root, perfect link */
    mesh_tree_evaluate(&m);

    T_CHECK_EQ(mesh_parent(&m), 1);
    T_CHECK_EQ(mesh_path_cost(&m), MESH_ETX_ONE);
    T_CHECK_EQ(mesh_hops(&m), 1);
    T_CHECK_EQ(n_parent_changes, 1);
    T_CHECK_EQ(last_new_parent, 1);
}

static void test_picks_min_total_cost(void)
{
    mesh_t m;
    make_node(&m, 5);

    /* Direct root link is lossy (ETX 3.0); a relay at cost 1.0 with a
     * clean link totals 2.0 and must win. */
    fab_nbr(&m, 1, 0, 3 * MESH_ETX_ONE, 0);
    fab_nbr(&m, 2, MESH_ETX_ONE, MESH_ETX_ONE, 1);
    mesh_tree_evaluate(&m);

    T_CHECK_EQ(mesh_parent(&m), 2);
    T_CHECK_EQ(mesh_path_cost(&m), 2 * MESH_ETX_ONE);
    T_CHECK_EQ(mesh_hops(&m), 2);
}

static void test_hysteresis(void)
{
    mesh_t m;
    make_node(&m, 5);

    fab_nbr(&m, 1, 0, 2 * MESH_ETX_ONE, 0); /* incumbent: total 2.0 */
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), 1);

    /* Challenger totals 1.0 — beats 2.0 by more than the 0.5 margin,
     * but must persist for MESH_HYST_EPOCHS epochs. */
    uint8_t both[2] = { 1, 2 };
    fab_nbr(&m, 2, 0, MESH_ETX_ONE, 0);

    mesh_tree_evaluate(&m); /* epoch 1: tick 1 — no switch yet */
    T_CHECK_EQ(mesh_parent(&m), 1);
    mesh_tree_evaluate(&m); /* same epoch: no double-tick */
    T_CHECK_EQ(mesh_parent(&m), 1);

    bump_epoch(&m, both, 2);
    mesh_tree_evaluate(&m); /* epoch 2: tick 2 — switch */
    T_CHECK_EQ(mesh_parent(&m), 2);
    T_CHECK_EQ(n_parent_changes, 2);
}

static void test_marginal_challenger_never_wins(void)
{
    mesh_t m;
    make_node(&m, 5);

    fab_nbr(&m, 1, 0, MESH_ETX_ONE + 100, 0); /* incumbent total ~1.39 */
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), 1);

    /* Challenger better, but within the 0.5-ETX margin: never switch. */
    uint8_t both[2] = { 1, 2 };
    fab_nbr(&m, 2, 0, MESH_ETX_ONE, 0);
    for (int i = 0; i < 6; i++) {
        bump_epoch(&m, both, 2);
        mesh_tree_evaluate(&m);
    }
    T_CHECK_EQ(mesh_parent(&m), 1);
    T_CHECK_EQ(n_parent_changes, 1);
}

static void test_never_selects_infinite_cost(void)
{
    mesh_t m;
    make_node(&m, 5);

    /* Neighbor exists but advertises infinity (poisoned, §5.4). */
    fab_nbr(&m, 3, MESH_COST_INF, MESH_ETX_ONE, 2);
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), MESH_ID_NONE);
    T_CHECK_EQ(n_parent_changes, 0);
}

static void test_stale_parent_poisons_then_reattaches(void)
{
    mesh_t m;
    make_node(&m, 5);

    fab_nbr(&m, 1, 0, MESH_ETX_ONE, 0);
    fab_nbr(&m, 2, MESH_ETX_ONE, MESH_ETX_ONE, 1);
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), 1);

    /* Parent goes silent for MESH_STALE_EPOCHS while node 2 keeps
     * beaconing: detach from 1, reattach through 2. */
    uint8_t only2[1] = { 2 };
    for (int i = 0; i < MESH_STALE_EPOCHS; i++)
        bump_epoch(&m, only2, 1);
    mesh_tree_evaluate(&m);

    T_CHECK_EQ(mesh_parent(&m), 2);
    T_CHECK_EQ(mesh_path_cost(&m), 2 * MESH_ETX_ONE);
    /* Two changes: 1 -> NONE (poison), NONE -> 2 (reattach). */
    T_CHECK_EQ(n_parent_changes, 3);
}

static void test_parent_dead_failover(void)
{
    mesh_t m;
    make_node(&m, 5);

    fab_nbr(&m, 1, 0, MESH_ETX_ONE, 0);
    fab_nbr(&m, 2, MESH_ETX_ONE, MESH_ETX_ONE, 1);
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), 1);

    /* Upward traffic exhausted its retries repeatedly: link declared
     * dead. Node must fail over without waiting for staleness. */
    mesh_tree_parent_dead(&m);
    T_CHECK_EQ(mesh_parent(&m), 2);

    /* The dead link's history is erased so it cannot bounce back. */
    mesh_neighbor_t *n1 = mesh_nbr_find(&m, 1);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, n1), MESH_COST_INF);
}

static void test_root_is_fixed(void)
{
    mesh_t m;
    make_node(&m, MESH_ID_ROOT);
    fab_nbr(&m, 4, MESH_ETX_ONE, MESH_ETX_ONE, 1);
    mesh_tree_evaluate(&m);
    T_CHECK_EQ(mesh_parent(&m), MESH_ID_NONE);
    T_CHECK_EQ(mesh_path_cost(&m), 0);
    T_CHECK_EQ(mesh_hops(&m), 0);
}

int main(void)
{
    T_RUN(test_attach_immediate);
    T_RUN(test_picks_min_total_cost);
    T_RUN(test_hysteresis);
    T_RUN(test_marginal_challenger_never_wins);
    T_RUN(test_never_selects_infinite_cost);
    T_RUN(test_stale_parent_poisons_then_reattaches);
    T_RUN(test_parent_dead_failover);
    T_RUN(test_root_is_fixed);
    return t_summary();
}
