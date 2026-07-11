/* Unit tests: command flooding — dedup, TTL, targeted execution,
 * re-flood re-ACK, hop-ACK satisfaction, root-side origination. */
#include "mesh_test_helpers.h"
#include "test_util.h"

static tx_capture_t cap;

static struct {
    unsigned count;
    uint16_t last_id;
    uint8_t last_len;
    uint8_t last_byte;
    bool accept;
} g_cmd;

static bool on_command(void *user, uint16_t cmd_id,
                       const uint8_t *payload, uint8_t len)
{
    (void)user;
    g_cmd.count++;
    g_cmd.last_id = cmd_id;
    g_cmd.last_len = len;
    g_cmd.last_byte = len ? payload[0] : 0;
    return g_cmd.accept;
}

static struct {
    unsigned count;
    uint8_t last_origin;
    uint16_t last_id;
} g_ack;

static void on_command_ack(void *user, uint8_t origin, uint16_t cmd_id,
                           uint8_t status)
{
    (void)user;
    (void)status;
    g_ack.count++;
    g_ack.last_origin = origin;
    g_ack.last_id = cmd_id;
}

/* Node 5, attached directly to the root via one heard beacon. */
static void make_attached_node(mesh_t *m)
{
    mesh_cfg_t cfg = test_cfg(5, &cap);
    cfg.cb.on_command = on_command;
    T_CHECK(mesh_init(m, &cfg));
    memset(&g_cmd, 0, sizeof(g_cmd));
    g_cmd.accept = true;

    uint8_t buf[MESH_MAX_FRAME];
    uint8_t cost0[2] = { 0, 0 };
    size_t len = mesh_frame_build(TEST_NET, &mesh_mic_null, buf,
                                  MESH_FT_BEACON, MESH_ID_ROOT,
                                  MESH_ID_BROADCAST, 1, 0, cost0, 2);
    mesh_rx(m, buf, len, -60, 8, 1000000);
    T_CHECK_EQ(mesh_parent(m), MESH_ID_ROOT);
    drain(m, &cap);
}

static size_t make_cmd(uint8_t *buf, uint8_t dst, uint16_t seq,
                       uint8_t ttl, uint16_t cmd_id, uint8_t arg)
{
    uint8_t body[3] = { (uint8_t)(cmd_id & 0xFF), (uint8_t)(cmd_id >> 8),
                        arg };
    return mesh_frame_build(TEST_NET, &mesh_mic_null, buf, MESH_FT_CMD,
                            MESH_ID_ROOT, dst, seq, ttl, body,
                            sizeof(body));
}

static void test_broadcast_execute_relay_ack(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_cmd(buf, MESH_ID_BROADCAST, 100, 8, 1, 'A');
    mesh_rx(&m, buf, len, -60, 8, t);

    T_CHECK_EQ(g_cmd.count, 1);
    T_CHECK_EQ(g_cmd.last_id, 1);
    T_CHECK_EQ(g_cmd.last_byte, 'A');
    T_CHECK_EQ(g_cmd.last_len, 1);

    mesh_poll(&m, t + 3000000);

    mesh_frame_t relay;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, &relay), 1);
    T_CHECK_EQ(relay.src, 5);              /* re-stamped transmitter */
    T_CHECK_EQ(relay.dst, MESH_ID_BROADCAST);
    T_CHECK_EQ(relay.seq, 100);            /* flood seq preserved */
    T_CHECK_EQ(relay.ttl, 7);              /* decremented */

    mesh_frame_t ack;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, &ack), 1);
    T_CHECK_EQ(ack.dst, MESH_ID_ROOT);
    T_CHECK_EQ(ack.plen, 4);
    T_CHECK_EQ(ack.payload[0], 1);         /* cmd_id lo */
    T_CHECK_EQ(ack.payload[2], 5);         /* origin */
    drain(&m, &cap);

    /* Exact duplicate: fully ignored. */
    mesh_rx(&m, buf, len, -60, 8, t + 4000000);
    mesh_poll(&m, t + 8000000);
    T_CHECK_EQ(g_cmd.count, 1);
    T_CHECK_EQ(cap.count, 0);

    /* Re-flood (new seq, same cmd_id): relayed and re-ACKed, but not
     * re-executed. */
    len = make_cmd(buf, MESH_ID_BROADCAST, 101, 8, 1, 'A');
    mesh_rx(&m, buf, len, -60, 8, t + 9000000);
    mesh_poll(&m, t + 12000000);
    T_CHECK_EQ(g_cmd.count, 1);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 1);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 1);
    T_CHECK_EQ(mesh_stats_get(&m)->cmds_executed, 1);
}

static void test_ttl_exhausted_not_relayed(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_cmd(buf, MESH_ID_BROADCAST, 100, 1, 2, 'B');
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t + 3000000);

    T_CHECK_EQ(g_cmd.count, 1); /* still executed */
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 0); /* no relay */
    T_CHECK_EQ(mesh_stats_get(&m)->floods_relayed, 0);
}

static void test_targeted_commands(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    /* Addressed to node 9: relayed by us, not executed. */
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_cmd(buf, 9, 100, 8, 3, 'C');
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t + 3000000);
    T_CHECK_EQ(g_cmd.count, 0);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 1);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 0);
    drain(&m, &cap);

    /* Addressed to us: relayed and executed. */
    len = make_cmd(buf, 5, 101, 8, 4, 'D');
    mesh_rx(&m, buf, len, -60, 8, t + 4000000);
    mesh_poll(&m, t + 7000000);
    T_CHECK_EQ(g_cmd.count, 1);
    T_CHECK_EQ(g_cmd.last_id, 4);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD, NULL), 1);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 1);
}

static void test_rejected_command_not_acked(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;
    g_cmd.accept = false;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_cmd(buf, 5, 100, 8, 7, 'R');
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t + 3000000);
    T_CHECK_EQ(g_cmd.count, 1);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 0);
    T_CHECK_EQ(mesh_stats_get(&m)->cmds_executed, 0);
    drain(&m, &cap);

    /* The base re-floods; now the app accepts: executed and ACKed. */
    g_cmd.accept = true;
    len = make_cmd(buf, 5, 101, 8, 7, 'R');
    mesh_rx(&m, buf, len, -60, 8, t + 4000000);
    mesh_poll(&m, t + 7000000);
    T_CHECK_EQ(g_cmd.count, 2);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 1);
}

static void test_hop_ack_stops_retries(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    uint8_t buf[MESH_MAX_FRAME];
    size_t len = make_cmd(buf, 5, 100, 8, 8, 'E');
    mesh_rx(&m, buf, len, -60, 8, t);
    mesh_poll(&m, t + 3000000);

    mesh_frame_t ack;
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, &ack), 1);

    /* Parent hop-ACKs our CMD_ACK: all retries must stop. */
    uint8_t hpl[3] = { MESH_FT_CMD_ACK, (uint8_t)(ack.seq & 0xFF),
                       (uint8_t)(ack.seq >> 8) };
    uint8_t hbuf[MESH_MAX_FRAME];
    size_t hlen = mesh_frame_build(TEST_NET, &mesh_mic_null, hbuf,
                                   MESH_FT_HOP_ACK, MESH_ID_ROOT, 5, 1,
                                   0, hpl, sizeof(hpl));
    mesh_rx(&m, hbuf, hlen, -60, 8, t + 3100000);

    mesh_poll(&m, t + 60000000);
    T_CHECK_EQ(cap_find(&m, &cap, MESH_FT_CMD_ACK, NULL), 1); /* no more */
    T_CHECK_EQ(mesh_stats_get(&m)->hop_exhausted, 0);
    T_CHECK_EQ(mesh_parent(&m), MESH_ID_ROOT);
}

static void test_retry_exhaustion_kills_parent_link(void)
{
    mesh_t m;
    make_attached_node(&m);
    uint64_t t = 2000000;

    /* Three commands, three un-ACKed upward sends. Walk time forward in
     * small steps so every retry fires and exhausts. */
    for (uint16_t i = 0; i < MESH_PARENT_FAIL_LIMIT; i++) {
        uint8_t buf[MESH_MAX_FRAME];
        size_t len = make_cmd(buf, 5, (uint16_t)(100 + i), 8,
                              (uint16_t)(10 + i), 'X');
        mesh_rx(&m, buf, len, -60, 8, t);
        for (int step = 0; step < 100; step++)
            mesh_poll(&m, t + (uint64_t)step * 100000);
        t += 20000000;
    }
    /* No HOP_ACK ever came back: the parent link must be declared dead
     * and the node detached (no alternative neighbor exists). */
    T_CHECK(mesh_stats_get(&m)->hop_exhausted >=
            MESH_PARENT_FAIL_LIMIT);
    T_CHECK_EQ(mesh_parent(&m), MESH_ID_NONE);
    T_CHECK_EQ(mesh_path_cost(&m), MESH_COST_INF);
}

static void test_root_originates_and_collects(void)
{
    mesh_t r;
    tx_capture_t rcap;
    memset(&rcap, 0, sizeof(rcap));
    mesh_cfg_t cfg = test_cfg(MESH_ID_ROOT, &rcap);
    cfg.cb.on_command_ack = on_command_ack;
    T_CHECK(mesh_init(&r, &cfg));
    memset(&g_ack, 0, sizeof(g_ack));
    uint64_t t = 1000000;

    uint8_t body[2] = { 0xAA, 0xBB };
    uint16_t id1 = mesh_root_command(&r, MESH_ID_BROADCAST, body, 2, 0, t);
    uint16_t id2 = mesh_root_command(&r, 9, body, 2, 0, t);
    T_CHECK_EQ(id1, 1);
    T_CHECK_EQ(id2, 2);
    mesh_poll(&r, t + 1);

    mesh_frame_t f;
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_CMD, &f), 2);
    T_CHECK_EQ(f.src, MESH_ID_ROOT);
    T_CHECK_EQ(f.ttl, MESH_FLOOD_TTL);

    /* Our own flood echoed back by a relay: must not re-enter. */
    unsigned seen_before = mesh_stats_get(&r)->cmds_seen;
    mesh_rx(&r, rcap.frames[0], rcap.lens[0], -60, 8, t + 500000);
    T_CHECK_EQ(mesh_stats_get(&r)->cmds_seen, seen_before);

    /* A CMD_ACK arrives from node 5: callback plus a hop-ACK back. */
    uint8_t apl[4] = { (uint8_t)(id1 & 0xFF), (uint8_t)(id1 >> 8), 5, 0 };
    uint8_t abuf[MESH_MAX_FRAME];
    size_t alen = mesh_frame_build(TEST_NET, &mesh_mic_null, abuf,
                                   MESH_FT_CMD_ACK, 5, MESH_ID_ROOT, 77,
                                   7, apl, sizeof(apl));
    mesh_rx(&r, abuf, alen, -70, 5, t + 600000);
    mesh_poll(&r, t + 700000);
    T_CHECK_EQ(g_ack.count, 1);
    T_CHECK_EQ(g_ack.last_origin, 5);
    T_CHECK_EQ(g_ack.last_id, id1);
    mesh_frame_t hop;
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_HOP_ACK, &hop), 1);
    T_CHECK_EQ(hop.dst, 5);

    /* Duplicate delivery (our hop-ACK was lost): re-hop-ACK, but the
     * application callback must not fire again. */
    mesh_rx(&r, abuf, alen, -70, 5, t + 800000);
    mesh_poll(&r, t + 900000);
    T_CHECK_EQ(g_ack.count, 1);
    T_CHECK_EQ(cap_find(&r, &rcap, MESH_FT_HOP_ACK, NULL), 2);
}

int main(void)
{
    T_RUN(test_broadcast_execute_relay_ack);
    T_RUN(test_ttl_exhausted_not_relayed);
    T_RUN(test_targeted_commands);
    T_RUN(test_rejected_command_not_acked);
    T_RUN(test_hop_ack_stops_retries);
    T_RUN(test_retry_exhaustion_kills_parent_link);
    T_RUN(test_root_originates_and_collects);
    return t_summary();
}
