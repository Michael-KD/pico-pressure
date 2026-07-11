/*
 * sim_net.h — host-side multi-node network simulator.
 *
 * Because the library is radio-agnostic and externally clocked, whole
 * fleets can run in-process: each virtual node owns a real mesh_t, the
 * simulated radio implements mesh_radio_ops_t, and a virtual clock
 * drives mesh_poll(). Links are directional pairs with a configurable
 * loss percentage; delivery latency models LoRa air time. Collisions
 * and half-duplex effects are NOT modeled in this milestone — the
 * protocol's randomized backoffs make that acceptable for logic tests.
 */
#ifndef SIM_NET_H
#define SIM_NET_H

#include "mesh_internal.h"

#define SIM_MAX_NODES 32
#define SIM_MAX_INFLIGHT 512
#define SIM_STEP_US 10000        /* 10 ms simulation step */
#define SIM_NO_LINK 255

typedef struct sim_net sim_net_t;

typedef struct {
    mesh_t mesh;
    sim_net_t *net;
    uint8_t id;
    bool alive;
    bool listening; /* radio RX state, driven by set_listen */
    /* node-side application state */
    unsigned cmds_executed;
    uint16_t last_cmd_id;
    uint8_t last_cmd_byte;
} sim_node_t;

typedef struct {
    bool used;
    uint8_t to;
    uint8_t len;
    int16_t rssi;
    uint64_t deliver_at;
    uint8_t buf[MESH_MAX_FRAME];
} sim_inflight_t;

struct sim_net {
    unsigned n_nodes;
    uint32_t seed0;               /* seed given to sim_init (for re-init) */
    const mesh_mic_ops_t *mic;    /* fleet MIC ops; NULL = mesh_mic_null */
    const mesh_sched_cfg_t *sched; /* fleet scheduler cfg; NULL = off */
    sim_node_t nodes[SIM_MAX_NODES];
    /* loss_pct[a][b]: percent of frames from a lost on the way to b;
     * SIM_NO_LINK = not in radio range. */
    uint8_t loss_pct[SIM_MAX_NODES][SIM_MAX_NODES];
    sim_inflight_t fly[SIM_MAX_INFLIGHT];
    uint64_t now;
    uint32_t rng;
    /* root-side application state (what the base station would track) */
    unsigned ack_count[SIM_MAX_NODES];
    uint16_t last_acked_cmd[SIM_MAX_NODES];
    unsigned telem_count[SIM_MAX_NODES];
    uint8_t reported_parent[SIM_MAX_NODES];
    unsigned frames_dropped;       /* lost to link loss (diagnostics) */
    unsigned frames_missed_asleep; /* arrived while receiver slept */
};

/* Create n nodes with IDs 0..n-1 (0 is the root); no links yet. */
void sim_init(sim_net_t *s, unsigned n, uint32_t seed);
/* Same, with every node using the given MIC ops (e.g., AES-CMAC).
 * The ops (and their ctx) must outlive the sim. */
void sim_init_mic(sim_net_t *s, unsigned n, uint32_t seed,
                  const mesh_mic_ops_t *mic);
/* Full variant: optional MIC ops and optional superframe scheduler
 * config applied fleet-wide (§4). With sched non-NULL the root beacons
 * itself — drive time with sim_run(), not sim_epoch(). */
void sim_init_opt(sim_net_t *s, unsigned n, uint32_t seed,
                  const mesh_mic_ops_t *mic,
                  const mesh_sched_cfg_t *sched);
/* Re-initialize one node's mesh with different MIC ops — a node keyed
 * differently from the fleet (authentication tests). Resets that node's
 * mesh and app state. */
void sim_set_node_mic(sim_net_t *s, uint8_t id, const mesh_mic_ops_t *mic);
/* Symmetric link with the given loss percentage (0 = perfect). */
void sim_link(sim_net_t *s, uint8_t a, uint8_t b, uint8_t loss_pct);
void sim_kill(sim_net_t *s, uint8_t id);
void sim_revive(sim_net_t *s, uint8_t id);

/* Advance virtual time, delivering frames and polling every node. */
void sim_run(sim_net_t *s, uint64_t dur_us);
/* One beacon epoch: root beacons, then the superframe elapses. */
void sim_epoch(sim_net_t *s, uint64_t superframe_us);

mesh_t *sim_mesh(sim_net_t *s, uint8_t id);

/* True when every alive node is attached and parent chains reach the
 * root without loops, with strictly decreasing path cost toward it. */
bool sim_tree_valid(sim_net_t *s);

#endif /* SIM_NET_H */
