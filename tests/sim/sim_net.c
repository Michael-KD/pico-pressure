/* sim_net.c — see sim_net.h. */
#include <stdio.h>

#include "fleet_modes.h" /* examples' shared mode table ('M' command) */
#include "sim_net.h"

/* ---- deterministic RNG (independent of the library's) ---- */

static uint32_t sim_rand(sim_net_t *s)
{
    uint32_t x = s->rng;
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    s->rng = x;
    return x;
}

static bool sim_lost(sim_net_t *s, uint8_t pct)
{
    return (sim_rand(s) % 100) < pct;
}

/* ---- simulated radio ---- */

static bool sim_transmit(void *ctx, const uint8_t *frame, size_t len,
                         mesh_profile_t profile)
{
    (void)profile;
    sim_node_t *tx = (sim_node_t *)ctx;
    sim_net_t *s = tx->net;

    if (!tx->alive || len > MESH_MAX_FRAME)
        return true; /* radio "sent it"; nobody will hear it */

    for (unsigned j = 0; j < s->n_nodes; j++) {
        if (j == tx->id || !s->nodes[j].alive)
            continue;
        uint8_t loss = s->loss_pct[tx->id][j];
        if (loss == SIM_NO_LINK)
            continue;
        if (sim_lost(s, loss)) {
            s->frames_dropped++;
            continue;
        }
        for (unsigned k = 0; k < SIM_MAX_INFLIGHT; k++) {
            if (!s->fly[k].used) {
                sim_inflight_t *f = &s->fly[k];
                f->used = true;
                f->to = (uint8_t)j;
                f->len = (uint8_t)len;
                memcpy(f->buf, frame, len);
                /* LoRa-ish air time: 60–100 ms. */
                f->deliver_at = s->now + 60000 +
                                (sim_rand(s) % 40000);
                /* Loss percentage doubles as a crude distance proxy
                 * for the reported RSSI. */
                f->rssi = (int16_t)(-60 - loss);
                break;
            }
        }
    }
    return true;
}

static void sim_set_listen(void *ctx, bool enable, mesh_profile_t profile)
{
    (void)profile;
    ((sim_node_t *)ctx)->listening = enable;
}

/* ---- application callbacks ---- */

static bool sim_on_command(void *user, uint16_t cmd_id,
                           const uint8_t *payload, uint8_t len)
{
    sim_node_t *n = (sim_node_t *)user;
    n->cmds_executed++;
    n->last_cmd_id = cmd_id;
    n->last_cmd_byte = len ? payload[0] : 0;
    /* Fleet mode switch ('M', M6): mirror the field firmware — apply
     * the commanded mode at its absolute effective time. */
    if (len == FLEET_MODE_PLEN && payload[0] == FLEET_CMD_MODE) {
        uint8_t mode_id;
        uint64_t at;
        mesh_sched_mode_t md;
        if (!fleet_mode_parse(payload, len, &mode_id, &at) ||
            !fleet_mode_by_id(mode_id, &md))
            return false;
        return mesh_sched_set_mode(&n->mesh, &md, at);
    }
    return true;
}

static void sim_on_command_ack(void *user, uint8_t origin,
                               uint16_t cmd_id, uint8_t status)
{
    (void)status;
    sim_node_t *root = (sim_node_t *)user;
    sim_net_t *s = root->net;
    if (origin < SIM_MAX_NODES) {
        s->ack_count[origin]++;
        s->last_acked_cmd[origin] = cmd_id;
    }
    if (s->ack_hook)
        s->ack_hook(s, origin, cmd_id);
}

static void sim_on_telemetry(void *user, const mesh_telem_info_t *info,
                             const uint8_t *payload, uint8_t len)
{
    (void)payload;
    (void)len;
    sim_node_t *root = (sim_node_t *)user;
    sim_net_t *s = root->net;
    if (info->origin < SIM_MAX_NODES) {
        s->telem_count[info->origin]++;
        s->reported_parent[info->origin] = info->parent;
    }
}

uint8_t sim_bulk_byte(uint8_t id, uint32_t off)
{
    return (uint8_t)(0x3C ^ (id * 37u) ^ off ^ (off >> 6));
}

static uint8_t sim_on_bulk_read(void *user, uint8_t session,
                                uint32_t offset, uint8_t *buf,
                                uint8_t max)
{
    sim_node_t *n = (sim_node_t *)user;
    sim_net_t *s = n->net;

    if (session == 0 || session >= 240) {
        /* LIST convention (0 = manual, 240+ = campaign engine): an
         * index line in the same format as the example field firmware,
         * so the base-policy engine can parse it (M6). */
        char idx[64];
        uint16_t crc = 0xFFFF;
        for (uint32_t o = 0; o < s->bulk_src_len; o++) {
            uint8_t b = sim_bulk_byte(n->id, o);
            crc = mesh_crc16_step(crc, b);
        }
        int len = snprintf(idx, sizeof(idx),
                           "node %u: evt 1, %u B, crc16 %04X", n->id,
                           (unsigned)s->bulk_src_len, crc);
        uint8_t i = 0;
        while (i < max && offset + i < (uint32_t)len) {
            buf[i] = (uint8_t)idx[offset + i];
            i++;
        }
        return i;
    }

    uint8_t i = 0;
    while (i < max && offset + i < s->bulk_src_len) {
        buf[i] = sim_bulk_byte(n->id, offset + i);
        i++;
    }
    return i;
}

static void sim_on_bulk_data(void *user, uint8_t session,
                             uint16_t chunk_index, const uint8_t *data,
                             uint8_t len)
{
    sim_node_t *root = (sim_node_t *)user;
    sim_net_t *s = root->net;
    if (s->bulk_hook)
        s->bulk_hook(s, session, chunk_index, data, len);
    if (session != s->bulk_session)
        return;
    if (chunk_index < SIM_BULK_MAX_CHUNKS &&
        !(s->bulk_mask & (1ull << chunk_index))) {
        s->bulk_mask |= 1ull << chunk_index;
        s->bulk_rx_chunks++;
    }
    uint32_t off = s->bulk_base + (uint32_t)chunk_index * s->bulk_chunk;
    for (uint8_t i = 0; i < len; i++) {
        if (off + i < SIM_BULK_BUF)
            s->bulk_buf[off + i] = data[i];
    }
}

bool sim_fetch(sim_net_t *s, uint8_t target, uint8_t session,
               uint32_t base, uint32_t off, uint32_t rlen, uint8_t chunk)
{
    if (session != s->bulk_session) {
        s->bulk_session = session;
        s->bulk_mask = 0;
        s->bulk_rx_chunks = 0;
        memset(s->bulk_buf, 0, sizeof(s->bulk_buf));
    }
    s->bulk_base = base;
    s->bulk_chunk = chunk;
    return mesh_root_fetch(&s->nodes[0].mesh, target, session, base,
                           off, rlen, chunk, s->now);
}

/* ---- lifecycle ---- */

static void sim_node_setup(sim_net_t *s, unsigned i,
                           const mesh_mic_ops_t *mic)
{
    sim_node_t *node = &s->nodes[i];
    node->net = s;
    node->id = (uint8_t)i;
    node->alive = true;
    node->cmds_executed = 0;
    node->last_cmd_id = 0;
    node->last_cmd_byte = 0;

    mesh_cfg_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.node_id = (uint8_t)i;
    cfg.network_id = 0xD5E7; /* "desert" */
    cfg.radio.ctx = node;
    cfg.radio.transmit = sim_transmit;
    cfg.radio.set_listen = sim_set_listen;
    if (mic)
        cfg.mic = *mic;
    if (s->sched)
        cfg.sched = *s->sched;
    cfg.rng_seed = s->seed0 * 2654435761u + (uint32_t)i + 1;
    cfg.cb.user = node;
    if (i == 0) {
        cfg.cb.on_command_ack = sim_on_command_ack;
        cfg.cb.on_telemetry = sim_on_telemetry;
        cfg.cb.on_bulk_data = sim_on_bulk_data;
    } else {
        cfg.cb.on_command = sim_on_command;
        cfg.cb.on_bulk_read = sim_on_bulk_read;
    }
    mesh_init(&node->mesh, &cfg);
}

void sim_init_opt(sim_net_t *s, unsigned n, uint32_t seed,
                  const mesh_mic_ops_t *mic,
                  const mesh_sched_cfg_t *sched)
{
    memset(s, 0, sizeof(*s));
    s->n_nodes = n;
    s->seed0 = seed;
    s->mic = mic;
    s->sched = sched;
    s->rng = seed ? seed : 1;
    memset(s->loss_pct, SIM_NO_LINK, sizeof(s->loss_pct));
    memset(s->reported_parent, MESH_ID_NONE, sizeof(s->reported_parent));
    s->bulk_src_len = SIM_BULK_BUF;

    for (unsigned i = 0; i < n; i++)
        sim_node_setup(s, i, mic);
}

void sim_init_mic(sim_net_t *s, unsigned n, uint32_t seed,
                  const mesh_mic_ops_t *mic)
{
    sim_init_opt(s, n, seed, mic, NULL);
}

void sim_init(sim_net_t *s, unsigned n, uint32_t seed)
{
    sim_init_opt(s, n, seed, NULL, NULL);
}

void sim_set_node_mic(sim_net_t *s, uint8_t id, const mesh_mic_ops_t *mic)
{
    sim_node_setup(s, id, mic);
}

void sim_link(sim_net_t *s, uint8_t a, uint8_t b, uint8_t loss_pct)
{
    s->loss_pct[a][b] = loss_pct;
    s->loss_pct[b][a] = loss_pct;
}

void sim_kill(sim_net_t *s, uint8_t id)
{
    s->nodes[id].alive = false;
}

void sim_revive(sim_net_t *s, uint8_t id)
{
    s->nodes[id].alive = true;
}

void sim_reboot(sim_net_t *s, uint8_t id)
{
    sim_node_setup(s, id, s->mic);
}

mesh_t *sim_mesh(sim_net_t *s, uint8_t id)
{
    return &s->nodes[id].mesh;
}

/* ---- time ---- */

static void sim_step(sim_net_t *s)
{
    s->now += SIM_STEP_US;

    for (unsigned k = 0; k < SIM_MAX_INFLIGHT; k++) {
        sim_inflight_t *f = &s->fly[k];
        if (!f->used || f->deliver_at > s->now)
            continue;
        f->used = false;
        sim_node_t *rx = &s->nodes[f->to];
        if (rx->alive && !rx->listening) {
            s->frames_missed_asleep++;
        } else if (rx->alive) {
            /* Tap: keep the raw wire bytes of the newest delivered
             * frame for replay experiments (§10.2). */
            s->tap_to = f->to;
            s->tap_len = f->len;
            memcpy(s->tap_buf, f->buf, f->len);
            mesh_rx(&rx->mesh, f->buf, f->len, f->rssi, 8, s->now);
        }
    }

    for (unsigned i = 0; i < s->n_nodes; i++) {
        if (s->nodes[i].alive)
            mesh_poll(&s->nodes[i].mesh, s->now);
    }
}

void sim_run(sim_net_t *s, uint64_t dur_us)
{
    uint64_t end = s->now + dur_us;
    while (s->now < end)
        sim_step(s);
}

void sim_epoch(sim_net_t *s, uint64_t superframe_us)
{
    if (s->nodes[0].alive)
        mesh_root_beacon(&s->nodes[0].mesh, s->now);
    sim_run(s, superframe_us);
}

/* ---- invariants ---- */

bool sim_tree_valid(sim_net_t *s)
{
    for (unsigned i = 1; i < s->n_nodes; i++) {
        if (!s->nodes[i].alive)
            continue;
        uint8_t at = (uint8_t)i;
        unsigned steps = 0;
        while (at != MESH_ID_ROOT) {
            mesh_t *m = &s->nodes[at].mesh;
            uint8_t p = mesh_parent(m);
            if (p == MESH_ID_NONE || p >= s->n_nodes ||
                !s->nodes[p].alive) {
                printf("  tree invalid: node %u parent %u (from %u)\n",
                       at, p, i);
                return false;
            }
            /* Cost must strictly decrease toward the root. */
            uint16_t my_cost = mesh_path_cost(m);
            uint16_t p_cost = mesh_path_cost(&s->nodes[p].mesh);
            if (p != MESH_ID_ROOT && p_cost >= my_cost) {
                printf("  tree invalid: node %u cost %u <= parent %u "
                       "cost %u\n", at, my_cost, p, p_cost);
                return false;
            }
            if (++steps > s->n_nodes) {
                printf("  tree invalid: loop through node %u\n", i);
                return false;
            }
            at = p;
        }
    }
    return true;
}
