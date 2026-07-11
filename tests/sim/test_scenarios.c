/*
 * test_scenarios.c — multi-node integration tests on the simulator.
 *
 * Superframes are shortened to 10 s of virtual time (the real deployment
 * uses 60 s active / 15 min standby); the protocol only cares about
 * epochs, not wall time, so this just makes the tests fast.
 */
#include "sim_net.h"
#include "test_util.h"

#define SF 10000000ull /* 10 s superframe */

static sim_net_t net; /* large; reused across tests */

/* 10-node mixed topology: two branches off the root, forks below, and
 * increasing loss toward the leaves.
 *
 *        0
 *      /   \
 *     1     2
 *    / \     \
 *   3   4     5
 *   |   |     |
 *   6   7     8
 *   |
 *   9
 */
static void build_ten(uint32_t seed, uint8_t extra_loss)
{
    sim_init(&net, 10, seed);
    sim_link(&net, 0, 1, (uint8_t)(5 + extra_loss));
    sim_link(&net, 0, 2, (uint8_t)(5 + extra_loss));
    sim_link(&net, 1, 3, (uint8_t)(10 + extra_loss));
    sim_link(&net, 1, 4, (uint8_t)(10 + extra_loss));
    sim_link(&net, 2, 5, (uint8_t)(10 + extra_loss));
    sim_link(&net, 3, 6, (uint8_t)(15 + extra_loss));
    sim_link(&net, 4, 7, (uint8_t)(15 + extra_loss));
    sim_link(&net, 5, 8, (uint8_t)(15 + extra_loss));
    sim_link(&net, 6, 9, (uint8_t)(20 + extra_loss));
}

static void test_line3_forms_two_hop_tree(void)
{
    sim_init(&net, 3, 0xC0FFEE);
    sim_link(&net, 0, 1, 0);
    sim_link(&net, 1, 2, 0); /* no 0-2 link: terrain blocks it */

    for (int e = 0; e < 3; e++)
        sim_epoch(&net, SF);

    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 1)), 0);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 1);
    T_CHECK_EQ(mesh_hops(sim_mesh(&net, 1)), 1);
    T_CHECK_EQ(mesh_hops(sim_mesh(&net, 2)), 2);
    /* Perfect links: costs are exactly 1.0 and 2.0 ETX. */
    T_CHECK_EQ(mesh_path_cost(sim_mesh(&net, 1)), MESH_ETX_ONE);
    T_CHECK_EQ(mesh_path_cost(sim_mesh(&net, 2)), 2 * MESH_ETX_ONE);
    T_CHECK(sim_tree_valid(&net));
}

static void test_ten_node_tree_formation(void)
{
    build_ten(0xACE1, 0);
    for (int e = 0; e < 6; e++)
        sim_epoch(&net, SF);

    T_CHECK(sim_tree_valid(&net));
    for (unsigned i = 1; i < net.n_nodes; i++)
        T_CHECK(mesh_parent(sim_mesh(&net, (uint8_t)i)) != MESH_ID_NONE);
    /* The deep chain must have found its only viable path. */
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 9)), 6);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 6)), 3);
    T_CHECK_EQ(mesh_hops(sim_mesh(&net, 9)), 4);
}

static void test_self_healing_relay_death(void)
{
    /*   0 ---- 1 ---- 2 ---- 3        primary path: 2 hops to node 2
     *    \             |
     *     4 ---- 5 ----                backup path: 3 hops to node 2
     *
     * All links perfect, so the scenario is deterministic and tests the
     * mechanisms (staleness poisoning, failover, hysteresis-gated
     * re-optimization) rather than luck on lossy links. */
    sim_init(&net, 6, 0xBEE5);
    sim_link(&net, 0, 1, 0);
    sim_link(&net, 1, 2, 0);
    sim_link(&net, 2, 3, 0);
    sim_link(&net, 0, 4, 0);
    sim_link(&net, 4, 5, 0);
    sim_link(&net, 5, 2, 0);

    for (int e = 0; e < 6; e++)
        sim_epoch(&net, SF);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 1); /* 2.0 beats 3.0 ETX */
    T_CHECK(sim_tree_valid(&net));

    /* Relay 1 dies. Node 2 misses MESH_STALE_EPOCHS beacons from it,
     * poisons the route, and fails over to the longer path via 5;
     * node 3 stays under node 2 throughout. */
    sim_kill(&net, 1);
    for (int e = 0; e < 7; e++)
        sim_epoch(&net, SF);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 5);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 3)), 2);
    T_CHECK(sim_tree_valid(&net));

    /* Relay 1 revives. Its link is re-probed fresh, but hysteresis
     * still demands the challenger win for two consecutive epochs, so
     * the first epoch must not flap. */
    sim_revive(&net, 1);
    sim_epoch(&net, SF);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 5); /* no instant flap */
    for (int e = 0; e < 11; e++)
        sim_epoch(&net, SF);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 1); /* re-optimized */
    T_CHECK(sim_tree_valid(&net));
}

static void test_lossy_flood_reaches_everyone_once(void)
{
    /* 6-node line, every link 20% lossy: the worst case for both the
     * flood (5 hops out) and the ACK path (5 hops back). */
    sim_init(&net, 6, 0xFADE);
    for (uint8_t i = 0; i + 1 < 6; i++)
        sim_link(&net, i, (uint8_t)(i + 1), 20);

    /* Formation on a deep lossy line is limited by beacon-wave reach
     * (0.8^5 per epoch to the last node), so give it time. */
    for (int e = 0; e < 12; e++)
        sim_epoch(&net, SF);
    T_CHECK(sim_tree_valid(&net));

    /* Base policy per the design doc: flood, collect ACKs, re-flood the
     * same command ID for stragglers, up to R times. */
    uint8_t arm[3] = { 'A', 'R', 'M' };
    uint16_t cmd = 0;
    unsigned attempts = 0;
    bool all_acked = false;
    while (attempts < 8 && !all_acked) {
        cmd = mesh_root_command(sim_mesh(&net, 0), MESH_ID_BROADCAST,
                                arm, sizeof(arm), cmd, net.now);
        T_CHECK(cmd != 0);
        attempts++;
        sim_run(&net, 30000000); /* 30 s for flood + ACKs */
        sim_epoch(&net, SF);     /* keep the tree fresh */
        all_acked = true;
        for (unsigned i = 1; i < net.n_nodes; i++) {
            if (net.last_acked_cmd[i] != cmd || net.ack_count[i] == 0)
                all_acked = false;
        }
    }
    printf("  flood attempts needed: %u\n", attempts);
    T_CHECK(all_acked);

    /* Exactly-once execution regardless of how many re-floods ran. */
    for (unsigned i = 1; i < net.n_nodes; i++) {
        T_CHECK_EQ(net.nodes[i].cmds_executed, 1);
        T_CHECK_EQ(net.nodes[i].last_cmd_id, cmd);
        T_CHECK_EQ(net.nodes[i].last_cmd_byte, 'A');
    }
}

static void test_targeted_command(void)
{
    sim_init(&net, 4, 0xD1CE);
    for (uint8_t i = 0; i + 1 < 4; i++)
        sim_link(&net, i, (uint8_t)(i + 1), 10);

    for (int e = 0; e < 4; e++)
        sim_epoch(&net, SF);
    T_CHECK(sim_tree_valid(&net));

    uint8_t fetch[2] = { 'F', 7 };
    uint16_t cmd = mesh_root_command(sim_mesh(&net, 0), 3, fetch,
                                     sizeof(fetch), 0, net.now);
    sim_run(&net, 30000000);

    /* Only the target executed; the middle nodes relayed. */
    T_CHECK_EQ(net.nodes[3].cmds_executed, 1);
    T_CHECK_EQ(net.nodes[3].last_cmd_id, cmd);
    T_CHECK_EQ(net.nodes[1].cmds_executed, 0);
    T_CHECK_EQ(net.nodes[2].cmds_executed, 0);
    T_CHECK(mesh_stats_get(sim_mesh(&net, 1))->floods_relayed >= 1);
    T_CHECK_EQ(net.ack_count[3], 1);
    T_CHECK_EQ(net.last_acked_cmd[3], cmd);
}

static void test_telemetry_builds_topology_map(void)
{
    build_ten(0xF00D, 5); /* 10–25% loss across the tree */
    for (int e = 0; e < 6; e++)
        sim_epoch(&net, SF);
    T_CHECK(sim_tree_valid(&net));

    /* Every field node reports once; per-hop retries carry the reports
     * over the lossy links. */
    uint8_t status[2] = { 0x64, 0x01 }; /* battery %, GPS lock */
    for (unsigned i = 1; i < net.n_nodes; i++)
        T_CHECK(mesh_node_telemetry(sim_mesh(&net, (uint8_t)i), status,
                                    sizeof(status), net.now));
    sim_run(&net, 60000000);

    /* The base can now reconstruct the exact tree. */
    for (unsigned i = 1; i < net.n_nodes; i++) {
        T_CHECK(net.telem_count[i] >= 1);
        T_CHECK_EQ(net.reported_parent[i],
                   mesh_parent(sim_mesh(&net, (uint8_t)i)));
    }
}

static void test_authenticated_fleet(void)
{
    /* Whole fleet runs the real AES-128-CMAC MIC; a fifth node keyed
     * with the WRONG key is within radio range of everyone but must
     * remain a bystander: it can neither join the tree nor execute
     * commands, and its presence must not disturb the fleet. */
    static const uint8_t fleet_key[16] = {
        0xde, 0x5e, 0x27, 0x01, 0x02, 0x03, 0x04, 0x05,
        0x06, 0x07, 0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d,
    };
    static const uint8_t wrong_key[16] = { 0xff, 0xee, 0xdd };
    static mesh_mic_cmac_t fleet_cmac, wrong_cmac;
    mesh_mic_cmac_init(&fleet_cmac, fleet_key);
    mesh_mic_cmac_init(&wrong_cmac, wrong_key);
    mesh_mic_ops_t fleet_ops = mesh_mic_cmac_ops(&fleet_cmac);
    mesh_mic_ops_t wrong_ops = mesh_mic_cmac_ops(&wrong_cmac);

    sim_init_mic(&net, 5, 0x5EC0DE, &fleet_ops);
    sim_set_node_mic(&net, 4, &wrong_ops);
    sim_link(&net, 0, 1, 0);
    sim_link(&net, 1, 2, 0);
    sim_link(&net, 2, 3, 0);
    sim_link(&net, 0, 4, 0); /* outsider hears the root directly */
    sim_link(&net, 1, 4, 0);

    for (int e = 0; e < 6; e++)
        sim_epoch(&net, SF);

    /* The keyed fleet forms normally; the outsider stays unattached and
     * has been dropping everything it hears at the MIC gate. (No
     * sim_tree_valid here: it demands every alive node be attached, and
     * the outsider being unattached is the point.) */
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 1)), 0);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 1);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 3)), 2);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 4)), MESH_ID_NONE);
    T_CHECK(mesh_stats_get(sim_mesh(&net, 4))->rx_drop_mic > 0);
    T_CHECK_EQ(mesh_stats_get(sim_mesh(&net, 4))->rx_frames, 0);

    /* An authenticated broadcast command reaches (only) the fleet. */
    uint8_t arm[3] = { 'A', 'R', 'M' };
    uint16_t cmd = mesh_root_command(sim_mesh(&net, 0), MESH_ID_BROADCAST,
                                     arm, sizeof(arm), 0, net.now);
    sim_run(&net, 30000000);
    for (unsigned i = 1; i <= 3; i++) {
        T_CHECK_EQ(net.nodes[i].cmds_executed, 1);
        T_CHECK_EQ(net.last_acked_cmd[i], cmd);
    }
    T_CHECK_EQ(net.nodes[4].cmds_executed, 0);

    /* No MIC drops inside the correctly keyed fleet itself. */
    for (unsigned i = 0; i <= 3; i++)
        T_CHECK_EQ(mesh_stats_get(sim_mesh(&net, (uint8_t)i))->rx_drop_mic,
                   0);
}

static void test_scheduled_fleet(void)
{
    /* §4 end-to-end on a 4-node line with the superframe scheduler on:
     * the root beacons itself, traffic is confined to its windows,
     * radios sleep between superframes, and a fleet-wide mode switch
     * keeps everyone aligned.
     *
     * | beacon 6s | cmd 5s | telem 6s | sleep 3s |  period 20 s */
    static const mesh_sched_cfg_t sc = {
        .enabled = true,
        .guard_us = 100000,
        .mode = {
            .period_us = 20000000,
            .beacon_win_us = 6000000,
            .cmd_win_us = 5000000,
            .telem_win_us = 6000000,
            .bulk_win_us = 0,
            .telem_slot_us = 1000000,
        },
    };
    const uint64_t sf = sc.mode.period_us;

    sim_init_opt(&net, 4, 0x5CED01, NULL, &sc);
    for (uint8_t i = 0; i + 1 < 4; i++)
        sim_link(&net, i, (uint8_t)(i + 1), 0);

    /* No sim_epoch() here: beacons are the scheduler's job now. */
    sim_run(&net, 6 * sf);
    T_CHECK(sim_tree_valid(&net));
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 1)), 0);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 2)), 1);
    T_CHECK_EQ(mesh_parent(sim_mesh(&net, 3)), 2);

    /* Radios sleep in the sleep segment (phase 18 s)... */
    sim_run(&net, 18000000);
    for (unsigned i = 0; i < net.n_nodes; i++)
        T_CHECK(!net.nodes[i].listening);
    /* ...and are awake in the next beacon window (phase 1 s). */
    sim_run(&net, 3000000);
    for (unsigned i = 0; i < net.n_nodes; i++)
        T_CHECK(net.nodes[i].listening);

    /* A command issued mid-sleep is deferred to the next command
     * window and still executes exactly once everywhere. */
    sim_run(&net, 17000000); /* phase 18 s again */
    uint8_t arm[3] = { 'A', 'R', 'M' };
    uint16_t cmd = mesh_root_command(sim_mesh(&net, 0), MESH_ID_BROADCAST,
                                     arm, sizeof(arm), 0, net.now);
    T_CHECK(cmd != 0);
    sim_run(&net, 2 * sf);
    T_CHECK(mesh_stats_get(sim_mesh(&net, 0))->tx_deferred >= 1);
    for (unsigned i = 1; i < net.n_nodes; i++) {
        T_CHECK_EQ(net.nodes[i].cmds_executed, 1);
        T_CHECK_EQ(net.last_acked_cmd[i], cmd);
        T_CHECK(net.ack_count[i] >= 1);
    }

    /* Fleet-wide STATUS-style telemetry lands via the micro-slots. */
    uint8_t status[2] = { 0x64, 0x01 };
    for (unsigned i = 1; i < net.n_nodes; i++)
        T_CHECK(mesh_node_telemetry(sim_mesh(&net, (uint8_t)i), status,
                                    sizeof(status), net.now));
    sim_run(&net, 2 * sf);
    for (unsigned i = 1; i < net.n_nodes; i++) {
        T_CHECK(net.telem_count[i] >= 1);
        T_CHECK_EQ(net.reported_parent[i],
                   mesh_parent(sim_mesh(&net, (uint8_t)i)));
    }

    /* Fleet-synchronized switch to a faster (active) mode: every node
     * re-aligns at the same absolute instant and traffic keeps
     * flowing. */
    mesh_sched_mode_t fast = {
        .period_us = 12000000,
        .beacon_win_us = 4000000,
        .cmd_win_us = 3000000,
        .telem_win_us = 3000000,
        .bulk_win_us = 0,
        .telem_slot_us = 500000,
    };
    uint64_t at = net.now + 5000000;
    for (unsigned i = 0; i < net.n_nodes; i++)
        T_CHECK(mesh_sched_set_mode(sim_mesh(&net, (uint8_t)i), &fast,
                                    at));
    sim_run(&net, 5000000 + 4 * fast.period_us);
    T_CHECK(sim_tree_valid(&net));
    for (unsigned i = 1; i < net.n_nodes; i++)
        T_CHECK_EQ(mesh_sched_superframe(sim_mesh(&net, (uint8_t)i),
                                         net.now),
                   mesh_sched_superframe(sim_mesh(&net, 0), net.now));

    cmd = mesh_root_command(sim_mesh(&net, 0), MESH_ID_BROADCAST, arm,
                            sizeof(arm), 0, net.now);
    sim_run(&net, 3 * fast.period_us);
    for (unsigned i = 1; i < net.n_nodes; i++) {
        T_CHECK_EQ(net.nodes[i].cmds_executed, 2);
        T_CHECK_EQ(net.last_acked_cmd[i], cmd);
    }
}

int main(void)
{
    T_RUN(test_line3_forms_two_hop_tree);
    T_RUN(test_ten_node_tree_formation);
    T_RUN(test_self_healing_relay_death);
    T_RUN(test_lossy_flood_reaches_everyone_once);
    T_RUN(test_targeted_command);
    T_RUN(test_telemetry_builds_topology_map);
    T_RUN(test_authenticated_fleet);
    T_RUN(test_scheduled_fleet);
    return t_summary();
}
