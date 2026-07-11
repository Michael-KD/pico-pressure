/*
 * demo.c — watch the mesh work.
 *
 * Runs a 10-node fleet with redundant links on the simulator, prints the
 * collection tree as it forms, floods an ARM command and shows the ACK
 * checklist, then kills and revives a relay to show self-healing and
 * hysteresis-gated re-optimization.
 *
 *   ./build/tests/mesh_demo
 */
#include <stdio.h>

#include "sim_net.h"

#define SF 10000000ull /* 10 s superframe */

static sim_net_t net;

static void print_subtree(uint8_t id, int depth)
{
    mesh_t *m = sim_mesh(&net, id);
    uint16_t c = mesh_path_cost(m);
    printf("  %*s", depth * 4, "");
    if (id == MESH_ID_ROOT)
        printf("0 (base station)\n");
    else if (!net.nodes[id].alive)
        printf("%u  [DEAD]\n", id);
    else
        printf("%u  cost %u.%02u ETX, hop %u\n", id, c >> 8,
               ((c & 0xFF) * 100) >> 8, mesh_hops(m));
    for (unsigned j = 1; j < net.n_nodes; j++) {
        if (net.nodes[j].alive &&
            mesh_parent(sim_mesh(&net, (uint8_t)j)) == id)
            print_subtree((uint8_t)j, depth + 1);
    }
}

static void print_tree(const char *title)
{
    printf("\n%s\n", title);
    print_subtree(MESH_ID_ROOT, 0);
    unsigned orphans = 0;
    for (unsigned j = 1; j < net.n_nodes; j++) {
        if (net.nodes[j].alive &&
            mesh_parent(sim_mesh(&net, (uint8_t)j)) == MESH_ID_NONE) {
            printf("      %u  [unattached]\n", j);
            orphans++;
        }
    }
    if (!orphans && sim_tree_valid(&net))
        printf("  (tree valid: loop-free, costs decrease toward root)\n");
}

int main(void)
{
    /* Terrain-ish topology: two main branches, some cross links for
     * redundancy, loss growing toward the periphery.
     *
     *        0
     *       / \
     *      1   2
     *     /|\ / \
     *    3 | 4   5
     *    |\|/ \   \
     *    6 (3-4)7  8
     *    |
     *    9
     */
    /* The whole demo fleet runs the real AES-128-CMAC MIC. */
    static const uint8_t key[16] = "desert-demo-key";
    static mesh_mic_cmac_t cmac;
    mesh_mic_cmac_init(&cmac, key);
    mesh_mic_ops_t mic = mesh_mic_cmac_ops(&cmac);

    sim_init_mic(&net, 10, 0xDE5E27, &mic);
    sim_link(&net, 0, 1, 5);
    sim_link(&net, 0, 2, 5);
    sim_link(&net, 1, 3, 10);
    sim_link(&net, 1, 4, 10);
    sim_link(&net, 2, 4, 15);
    sim_link(&net, 2, 5, 10);
    sim_link(&net, 3, 4, 15);
    sim_link(&net, 3, 6, 15);
    sim_link(&net, 4, 7, 15);
    sim_link(&net, 5, 8, 15);
    sim_link(&net, 6, 9, 20);

    printf("=== Desert blast-sensor mesh demo: 1 base + 9 field nodes ===\n");
    printf("    (all frames authenticated with AES-128-CMAC)\n");

    sim_epoch(&net, SF);
    print_tree("After 1 beacon epoch:");
    for (int e = 0; e < 5; e++)
        sim_epoch(&net, SF);
    print_tree("After 6 beacon epochs:");

    /* Flood an ARM command to the whole fleet, re-flooding for
     * stragglers exactly like the base-station policy would. */
    printf("\nFlooding ARM to all nodes...\n");
    uint8_t arm[3] = { 'A', 'R', 'M' };
    uint16_t cmd = 0;
    for (int attempt = 1; attempt <= 5; attempt++) {
        cmd = mesh_root_command(sim_mesh(&net, 0), MESH_ID_BROADCAST,
                                arm, sizeof(arm), cmd, net.now);
        sim_run(&net, 30000000);
        unsigned acked = 0;
        for (unsigned i = 1; i < net.n_nodes; i++) {
            if (net.ack_count[i] && net.last_acked_cmd[i] == cmd)
                acked++;
        }
        printf("  attempt %d: %u/9 nodes ACKed\n", attempt, acked);
        if (acked == 9)
            break;
        sim_epoch(&net, SF);
    }
    for (unsigned i = 1; i < net.n_nodes; i++)
        printf("  node %u: executed=%u acked=%s\n", i,
               net.nodes[i].cmds_executed,
               net.ack_count[i] ? "yes" : "NO");

    /* Kill relay 1 (it is carrying nodes 3, 4 and the chain 6-9). */
    printf("\nRelay node 1 dies (was parent of a whole branch)...\n");
    sim_kill(&net, 1);
    for (int e = 0; e < 8; e++)
        sim_epoch(&net, SF);
    print_tree("8 epochs later (routed around the dead relay):");

    printf("\nRelay node 1 comes back (solar recharge)...\n");
    sim_revive(&net, 1);
    for (int e = 0; e < 10; e++)
        sim_epoch(&net, SF);
    print_tree("10 epochs later (tree re-optimized through node 1):");

    printf("\nSimulator totals: %u frames lost to link loss.\n",
           net.frames_dropped);
    return 0;
}
