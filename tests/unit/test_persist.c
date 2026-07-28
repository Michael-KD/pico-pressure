/*
 * test_persist.c — route/state snapshot codec and restore semantics
 * (src/persist.c, design doc §5.3).
 */
#include <string.h>

#include "mesh_internal.h"
#include "test_util.h"

static bool tx_ok(void *ctx, const uint8_t *data, size_t len,
                  mesh_profile_t p)
{
    (void)ctx;
    (void)data;
    (void)len;
    (void)p;
    return true;
}

static unsigned parent_cb_calls;
static uint8_t parent_cb_new;

static void on_parent(void *user, uint8_t oldp, uint8_t newp)
{
    (void)user;
    (void)oldp;
    parent_cb_calls++;
    parent_cb_new = newp;
}

static void init_node(mesh_t *m, uint8_t id)
{
    mesh_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.node_id = id;
    cfg.network_id = 0xD5E7;
    cfg.radio.transmit = tx_ok;
    cfg.rng_seed = 42;
    cfg.cb.on_parent_change = on_parent;
    mesh_init(m, &cfg);
}

/* Hand-build an attached node: parent 2 at 1.5 ETX, plus neighbor 5. */
static void make_attached(mesh_t *m)
{
    init_node(m, 7);
    mesh_neighbor_t *n = &m->nbrs[0];
    n->in_use = true;
    n->ever_beacon = true;
    n->id = 2;
    n->hops = 1;
    n->adv_cost = 300;
    n->etx_q8 = 384; /* 1.5 */
    n->last_beacon_seq = 120;
    n->beacon_bitmap = 0xFFFF;
    n->epochs_known = 16;

    n = &m->nbrs[1];
    n->in_use = true;
    n->ever_beacon = true;
    n->id = 5;
    n->hops = 2;
    n->adv_cost = 700;
    n->etx_q8 = 300;
    n->last_beacon_seq = 119;

    m->parent = 2;
    m->path_cost = 684;
    m->hops = 2;
    m->net_epoch = 120;
    m->have_epoch = true;
}

static void test_node_roundtrip(void)
{
    static mesh_t a, b;
    make_attached(&a);

    uint8_t blob[MESH_ROUTE_BLOB_MAX];
    size_t len = mesh_route_save(&a, blob, sizeof(blob));
    T_CHECK(len > 0 && len <= sizeof(blob));

    init_node(&b, 7);
    parent_cb_calls = 0;
    T_CHECK(mesh_route_load(&b, blob, len));

    /* Route resumes instantly: parent, cost, epoch position. */
    T_CHECK_EQ(b.parent, 2);
    T_CHECK_EQ(b.path_cost, 684);
    T_CHECK_EQ(b.hops, 2);
    T_CHECK_EQ(b.net_epoch, 120);
    T_CHECK(b.have_epoch);
    T_CHECK_EQ(parent_cb_calls, 1);
    T_CHECK_EQ(parent_cb_new, 2);

    /* Restored neighbors are routing candidates with the saved ETX. */
    mesh_neighbor_t *n = mesh_nbr_find(&b, 2);
    T_CHECK(n != NULL);
    T_CHECK_EQ(n->etx_q8, 384);
    T_CHECK_EQ(n->adv_cost, 300);
    T_CHECK(mesh_nbr_fresh(&b, n));
    T_CHECK_EQ(mesh_nbr_etx_now(&b, n), 384);
    T_CHECK(mesh_nbr_find(&b, 5) != NULL);

    /* ...which means telemetry can go up before any beacon. */
    uint8_t pl[2] = { 1, 2 };
    T_CHECK(mesh_node_telemetry(&b, pl, sizeof(pl), 1000));
}

static void test_node_rejections(void)
{
    static mesh_t a, b;
    make_attached(&a);
    uint8_t blob[MESH_ROUTE_BLOB_MAX];
    size_t len = mesh_route_save(&a, blob, sizeof(blob));
    T_CHECK(len > 0);

    /* CRC: every byte of the body is covered. */
    for (size_t i = 0; i < len - 2; i++) {
        uint8_t bad[MESH_ROUTE_BLOB_MAX];
        memcpy(bad, blob, len);
        bad[i] ^= 0x20;
        init_node(&b, 7);
        T_CHECK(!mesh_route_load(&b, bad, len));
    }

    /* Identity binding: another node's blob is stale, not adopted. */
    init_node(&b, 8);
    T_CHECK(!mesh_route_load(&b, blob, len));

    /* Wrong network. */
    init_node(&b, 7);
    b.network_id = 0xBEEF;
    T_CHECK(!mesh_route_load(&b, blob, len));

    /* Already-running mesh refuses a load. */
    init_node(&b, 7);
    b.have_epoch = true;
    T_CHECK(!mesh_route_load(&b, blob, len));

    /* Truncated blob. */
    init_node(&b, 7);
    T_CHECK(!mesh_route_load(&b, blob, len - 3));

    /* A node blob cannot restore a root and vice versa. */
    static mesh_t root;
    init_node(&root, 0);
    T_CHECK(!mesh_route_load(&root, blob, len));
}

static void test_root_counters(void)
{
    static mesh_t r, r2;
    init_node(&r, 0);
    r.beacon_seq = 500;
    r.flood_seq = 200;
    r.next_cmd_id = 90;

    uint8_t blob[MESH_ROUTE_BLOB_MAX];
    size_t len = mesh_route_save(&r, blob, sizeof(blob));
    T_CHECK(len > 0);

    init_node(&r2, 0);
    T_CHECK(mesh_route_load(&r2, blob, len));

    /* Counters come back strictly ahead: even beacons the old instance
     * sent after the save (up to the margin) compare as older. */
    T_CHECK_EQ(r2.beacon_seq, 500 + MESH_PERSIST_SEQ_MARGIN);
    T_CHECK_EQ(r2.flood_seq, 200 + MESH_PERSIST_SEQ_MARGIN);
    T_CHECK_EQ(r2.next_cmd_id, 90 + MESH_PERSIST_SEQ_MARGIN);
    T_CHECK(mesh_seq_gt(r2.beacon_seq, 510));
}

static void test_unattached_save(void)
{
    /* A never-attached node's blob restores neighbors but no parent. */
    static mesh_t a, b;
    make_attached(&a);
    a.parent = MESH_ID_NONE;

    uint8_t blob[MESH_ROUTE_BLOB_MAX];
    size_t len = mesh_route_save(&a, blob, sizeof(blob));
    T_CHECK(len > 0);

    init_node(&b, 7);
    parent_cb_calls = 0;
    T_CHECK(mesh_route_load(&b, blob, len));
    T_CHECK_EQ(b.parent, MESH_ID_NONE);
    T_CHECK_EQ(parent_cb_calls, 0);
    T_CHECK(mesh_nbr_find(&b, 2) != NULL);
}

int main(void)
{
    T_RUN(test_node_roundtrip);
    T_RUN(test_node_rejections);
    T_RUN(test_root_counters);
    T_RUN(test_unattached_save);
    return t_summary();
}
