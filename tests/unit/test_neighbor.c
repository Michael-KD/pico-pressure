/* Unit tests: neighbor table, observed-epoch ETX windows, staleness,
 * fresh restart after long silence, eviction. */
#include "mesh_test_helpers.h"
#include "test_util.h"

static tx_capture_t cap;

static void make_mesh(mesh_t *m, uint8_t id)
{
    mesh_cfg_t cfg = test_cfg(id, &cap);
    T_CHECK(mesh_init(m, &cfg));
}

/* Emulate what mesh_beacon_rx does with the epoch counters, then feed
 * the beacon to the estimator. */
static void hear(mesh_t *m, mesh_neighbor_t *n, uint16_t seq,
                 uint16_t adv_cost)
{
    if (!m->have_epoch || mesh_seq_gt(seq, m->net_epoch)) {
        m->net_epoch = seq;
        m->have_epoch = true;
        m->obs_epochs++;
    }
    mesh_nbr_beacon(m, n, seq, adv_cost, 0);
}

static void test_heard_creates_and_smooths(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    mesh_neighbor_t *n = mesh_nbr_heard(&m, 7, -80, 4);
    T_CHECK(n != NULL);
    T_CHECK_EQ(n->id, 7);
    T_CHECK_EQ(n->rssi_q4, -80 * 16);
    T_CHECK(!n->ever_beacon);

    /* EWMA moves a quarter of the way toward the new sample. */
    mesh_nbr_heard(&m, 7, -60, 4);
    T_CHECK_EQ(n->rssi_q4, -80 * 16 + ((-60 * 16) - (-80 * 16)) / 4);

    /* Own ID and broadcast are never neighbors. */
    T_CHECK(mesh_nbr_heard(&m, 5, -50, 0) == NULL);
    T_CHECK(mesh_nbr_heard(&m, MESH_ID_BROADCAST, -50, 0) == NULL);
}

static void test_etx_perfect_reception(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    mesh_neighbor_t *n = mesh_nbr_heard(&m, 1, -70, 6);
    for (uint16_t seq = 1; seq <= 20; seq++)
        hear(&m, n, seq, 0);
    /* Every observed epoch delivered: ETX is exactly 1.0. */
    T_CHECK_EQ(n->etx_q8, MESH_ETX_ONE);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, n), MESH_ETX_ONE);
    T_CHECK(mesh_nbr_fresh(&m, n));
}

static void test_etx_half_reception(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    /* Neighbor 1 delivers every epoch (it defines what we observe);
     * neighbor 2 delivers only even epochs: its ETX -> ~2.0. */
    mesh_neighbor_t *good = mesh_nbr_heard(&m, 1, -70, 6);
    mesh_neighbor_t *flaky = mesh_nbr_heard(&m, 2, -80, 3);
    for (uint16_t seq = 1; seq <= 40; seq++) {
        hear(&m, good, seq, 0);
        if ((seq & 1) == 0)
            hear(&m, flaky, seq, 0);
    }
    T_CHECK_EQ(good->etx_q8, MESH_ETX_ONE);
    T_CHECK(flaky->etx_q8 >= (3 * MESH_ETX_ONE) / 2);
    T_CHECK(flaky->etx_q8 <= (5 * MESH_ETX_ONE) / 2);
}

static void test_etx_penalizes_observed_silence(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    /* Both perfect for 16 epochs; then 2 goes silent while 1 keeps
     * delivering (so the silence is observed). */
    mesh_neighbor_t *good = mesh_nbr_heard(&m, 1, -70, 6);
    mesh_neighbor_t *dying = mesh_nbr_heard(&m, 2, -80, 3);
    for (uint16_t seq = 1; seq <= 16; seq++) {
        hear(&m, good, seq, 0);
        hear(&m, dying, seq, 0);
    }
    uint16_t before = mesh_nbr_etx_now(&m, dying);
    T_CHECK_EQ(before, MESH_ETX_ONE);

    for (uint16_t seq = 17; seq <= 24; seq++)
        hear(&m, good, seq, 0);
    T_CHECK(mesh_nbr_etx_now(&m, dying) > before);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, good), MESH_ETX_ONE);

    /* One epoch past a full observed window of silence: unusable. (The
     * in-progress epoch never counts, so the threshold is WINDOW + 1.) */
    for (uint16_t seq = 25; seq <= 32; seq++)
        hear(&m, good, seq, 0);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, dying), MESH_ETX_MAX_Q8);
    hear(&m, good, 33, 0);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, dying), MESH_COST_INF);
    T_CHECK(!mesh_nbr_fresh(&m, dying));
}

static void test_unobserved_gap_is_not_loss(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    /* Sole upstream neighbor; the beacon wave only reaches us on some
     * epochs. Gaps we never observed must not count against the link. */
    mesh_neighbor_t *n = mesh_nbr_heard(&m, 1, -70, 6);
    hear(&m, n, 1, 0);
    hear(&m, n, 5, 0);  /* seqs 2-4 never reached us at all */
    hear(&m, n, 9, 0);
    T_CHECK_EQ(n->etx_q8, MESH_ETX_ONE);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, n), MESH_ETX_ONE);
}

static void test_fresh_restart_after_long_silence(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    mesh_neighbor_t *good = mesh_nbr_heard(&m, 1, -70, 6);
    mesh_neighbor_t *revived = mesh_nbr_heard(&m, 2, -80, 3);
    for (uint16_t seq = 1; seq <= 4; seq++) {
        hear(&m, good, seq, 0);
        hear(&m, revived, seq, 0);
    }
    /* Node 2 dies for 20 observed epochs... */
    for (uint16_t seq = 5; seq <= 24; seq++)
        hear(&m, good, seq, 0);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, revived), MESH_COST_INF);

    /* ...then revives: the estimator restarts clean instead of carrying
     * the ETX of its own death. */
    hear(&m, revived, 25, 0);
    T_CHECK_EQ(revived->etx_q8, MESH_ETX_ONE);
    T_CHECK_EQ(mesh_nbr_etx_now(&m, revived), MESH_ETX_ONE);
    T_CHECK(mesh_nbr_fresh(&m, revived));
}

static void test_reordered_beacon_ignored(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    mesh_neighbor_t *n = mesh_nbr_heard(&m, 1, -70, 6);
    hear(&m, n, 10, 100);
    hear(&m, n, 12, 100);
    uint16_t bitmap = n->beacon_bitmap;
    /* A stale copy of seq 11 arrives late: estimator unchanged, but the
     * advertised cost still refreshes. */
    hear(&m, n, 11, 300);
    T_CHECK_EQ(n->beacon_bitmap, bitmap);
    T_CHECK_EQ(n->last_beacon_seq, 12);
    T_CHECK_EQ(n->adv_cost, 300);
}

static void test_staleness(void)
{
    mesh_t m;
    make_mesh(&m, 5);

    m.have_epoch = true;
    m.net_epoch = 1;
    mesh_neighbor_t *n = fab_nbr(&m, 1, 0, MESH_ETX_ONE, 0);
    T_CHECK(mesh_nbr_fresh(&m, n));
    /* Staleness counts real superframes (beacon seq), independent of
     * how many of them this node observed. */
    m.net_epoch = (uint16_t)(1 + MESH_STALE_EPOCHS - 1);
    T_CHECK(mesh_nbr_fresh(&m, n));
    m.net_epoch = (uint16_t)(1 + MESH_STALE_EPOCHS);
    T_CHECK(!mesh_nbr_fresh(&m, n));
}

static void test_eviction_protects_parent(void)
{
    mesh_t m;
    make_mesh(&m, 100); /* own ID outside the fabricated 1..16 range */
    m.have_epoch = true;
    m.net_epoch = 1;

    /* Fill the table; make node 1 the parent. */
    for (uint8_t id = 1; id <= MESH_MAX_NEIGHBORS; id++)
        fab_nbr(&m, id, 0, MESH_ETX_ONE, 0);
    m.parent = 1;

    /* Make node 2 clearly the worst link. */
    mesh_neighbor_t *worst = mesh_nbr_find(&m, 2);
    worst->etx_q8 = MESH_ETX_MAX_Q8;

    /* A new neighbor must evict node 2, never the parent. */
    mesh_neighbor_t *fresh = mesh_nbr_heard(&m, 99, -50, 10);
    T_CHECK(fresh != NULL);
    T_CHECK_EQ(fresh->id, 99);
    T_CHECK(mesh_nbr_find(&m, 1) != NULL);
    T_CHECK(mesh_nbr_find(&m, 2) == NULL);
    T_CHECK_EQ(mesh_stats_get(&m)->neighbors_evicted, 1);
}

int main(void)
{
    T_RUN(test_heard_creates_and_smooths);
    T_RUN(test_etx_perfect_reception);
    T_RUN(test_etx_half_reception);
    T_RUN(test_etx_penalizes_observed_silence);
    T_RUN(test_unobserved_gap_is_not_loss);
    T_RUN(test_fresh_restart_after_long_silence);
    T_RUN(test_reordered_beacon_ignored);
    T_RUN(test_staleness);
    T_RUN(test_eviction_protects_parent);
    return t_summary();
}
