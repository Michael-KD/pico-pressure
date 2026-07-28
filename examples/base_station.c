/*
 * base_station.c — base-station gateway example (Pico + RYLR998).
 *
 * Runs the mesh root (node 0) with a human CLI on USB serial: beacons
 * every superframe, floods commands, tracks the ACK checklist, and
 * builds the fleet topology map from telemetry (§5.3). Pair it with one
 * or more field nodes (main.c) for a working bench mesh.
 *
 * In the full system this role is a LoRa gateway attached to the base
 * server, and policy (retry counts, alerting, retrieval scheduling)
 * lives there (design principle 6: "the base is the only smart place").
 * This example keeps just enough policy to be useful from a terminal:
 *
 *   arm [min]   flood ARM to the fleet (default 60 minutes)
 *   disarm      flood DISARM
 *   status      flood STATUS (nodes answer with telemetry)
 *   resend      re-flood the last command (same cmd id: nodes that ran
 *               it re-ACK without re-executing — §6 step 4)
 *   top         print the topology map / ACK checklist
 *   stats       mesh + radio counters
 *   list <n>    fetch node n's recording index (bulk session 0)
 *   fetch <n> [bytes]   retrieve node n's synthetic event window (§7.3)
 *   refetch     re-FETCH from the first missing chunk (resumption)
 *   fstat       fetch progress: chunk bitmap, missing list, final CRC
 *   retrieve [n]  M6 campaign: switch the fleet to retrieval mode,
 *               LIST+FETCH+verify every known node (or just n), switch
 *               back — the §7.4 retrieval scheduler (base_policy.[ch])
 *   rstat       campaign progress / per-node results
 *   mode <m>    flood a fleet mode switch (active|standby|retrieval)
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "base_policy.h"
#include "flash_store.h"
#include "gps_pico.h"
#include "mesh/mesh.h"
#include "node_config.h"
#include "provision_cli.h"
#include "rylr998_pico.h"

static mesh_t mesh;
static rylr998_t radio;
static mesh_mic_cmac_t cmac;
static gps_pico_t gpsp;

/* GPS locked at boot: scheduler on, absolute time, auto-beacons. */
static bool use_gps;
static uint64_t mono_last;

static uint64_t app_now(void)
{
    uint64_t local = time_us_64();
    if (!use_gps)
        return local;
    uint64_t t = gps_now(&gpsp.gps, local);
    if (t <= mono_last)
        t = mono_last + 1;
    mono_last = t;
    return t;
}

/* Persist the root's sequence counters so a power-cycled base rejoins
 * its own fleet instead of beaconing "old" sequence numbers at it. */
static void save_root_counters(void)
{
    uint8_t blob[MESH_ROUTE_BLOB_MAX];
    size_t bl = mesh_route_save(&mesh, blob, sizeof(blob));
    if (bl > 0)
        flash_store_write(FLASH_SLOT_ROUTE, blob, bl);
}

/* ---- fleet bookkeeping (what the base server would persist) ---- */

#define FLEET_MAX 255

typedef struct {
    bool     seen;             /* any ACK or telemetry ever */
    uint16_t acked_cmd;        /* last command this node ACKed */
    bool     acked_valid;
    uint8_t  parent;
    uint8_t  nbr_id[MESH_TELEM_NBRS];
    uint16_t nbr_etx[MESH_TELEM_NBRS];
    uint8_t  state;            /* from telemetry payload */
    uint8_t  batt_dv;
    int8_t   temp_c;
    uint64_t last_heard_us;
} fleet_entry_t;

static fleet_entry_t fleet[FLEET_MAX];

/* Last flooded command, for `resend` and the ACK checklist. */
static uint16_t last_cmd_id;
static uint8_t last_cmd_pl[24]; /* fits the 17 B absolute-window ARM */
static uint8_t last_cmd_len;

/* ---- bulk fetch bookkeeping (§7.3: the base holds the bitmap) ---- */

#define FETCH_MAX_CHUNKS 64 /* 64 x 96 B chunks = 6 kB per fetch */

static struct {
    bool     active;
    uint8_t  target;
    uint8_t  session;
    uint8_t  chunk;
    uint32_t total;
    unsigned nchunks;
    uint64_t mask;    /* received chunk indices */
    unsigned got;
    uint8_t  buf[FETCH_MAX_CHUNKS * MESH_BULK_CHUNK];
} fx;

static uint8_t next_session = 1; /* 1..127 manual; 128+ belongs to the
                                  * retrieval campaign engine */

/* ---- M6 retrieval campaign (base_policy.[ch]) ---- */

static base_policy_t bp;
static uint8_t bp_buf[6144];

/* ---- mesh callbacks ---- */

static void on_command_ack(void *user, uint8_t origin, uint16_t cmd_id,
                           uint8_t status)
{
    (void)user;
    (void)status; /* reserved, always 0 in M1 */
    if (origin >= FLEET_MAX)
        return;
    fleet[origin].seen = true;
    fleet[origin].acked_cmd = cmd_id;
    fleet[origin].acked_valid = true;
    fleet[origin].last_heard_us = time_us_64();
    bp_on_command_ack(&bp, origin, cmd_id);
    printf("ACK: node %u executed cmd %u\n", origin, cmd_id);
}

static void on_telemetry(void *user, const mesh_telem_info_t *info,
                         const uint8_t *payload, uint8_t len)
{
    (void)user;
    if (info->origin >= FLEET_MAX)
        return;
    fleet_entry_t *e = &fleet[info->origin];
    e->seen = true;
    e->parent = info->parent;
    memcpy(e->nbr_id, info->nbr_id, sizeof(e->nbr_id));
    memcpy(e->nbr_etx, info->nbr_etx_q8, sizeof(e->nbr_etx));
    if (len >= 3) {
        e->state = payload[0];
        e->batt_dv = payload[1];
        e->temp_c = (int8_t)payload[2];
    }
    e->last_heard_us = time_us_64();
    printf("TELEM: node %u (parent %u, %s, %s, %u.%uV, %dC)\n",
           info->origin, info->parent,
           (e->state & 1) ? "ARMED" : "standby",
           (e->state & 2) ? "gps" : "NO-GPS", e->batt_dv / 10,
           e->batt_dv % 10, e->temp_c);
}

static void on_bulk_data(void *user, uint8_t session, uint16_t idx,
                         const uint8_t *data, uint8_t len)
{
    (void)user;
    if (bp_busy(&bp)) {
        /* Campaign in flight: sessions 0 and 128+ are its traffic. */
        bp_on_bulk_data(&bp, session, idx, data, len, app_now());
        if (session == 0 || session >= 128)
            return;
    }
    if (session == 0) {
        /* LIST: the node's recording index streams back as text. */
        printf("INDEX: %.*s\n", (int)len, (const char *)data);
        return;
    }
    if (!fx.active || session != fx.session || idx >= FETCH_MAX_CHUNKS)
        return;
    uint32_t off = (uint32_t)idx * fx.chunk;
    if (!(fx.mask & (1ull << idx))) {
        fx.mask |= 1ull << idx;
        fx.got++;
        if (off + len <= sizeof(fx.buf))
            memcpy(&fx.buf[off], data, len);
    }
    printf("DATA: chunk %u (%u B) — %u/%u\n", idx, len, fx.got,
           fx.nchunks);
    if (fx.got == fx.nchunks)
        printf("fetch complete: %lu bytes, crc16 %04X (compare with the "
               "node's index line)\n",
               (unsigned long)fx.total, mesh_crc16(fx.buf, fx.total));
}

static void on_radio_frame(void *ctx, const uint8_t *frame, size_t len,
                           int16_t rssi_dbm, int8_t snr_db)
{
    (void)ctx;
    mesh_rx(&mesh, frame, len, rssi_dbm, snr_db, app_now());
}

/* ---- commands ---- */

static void flood(const uint8_t *pl, uint8_t len, uint16_t reuse_id)
{
    uint64_t now = app_now();
    uint16_t id = mesh_root_command(&mesh, MESH_ID_BROADCAST, pl, len,
                                    reuse_id, now);
    if (id == 0) {
        printf("command failed (queue full?)\n");
        return;
    }
    if (id != last_cmd_id) {
        /* Fresh command: reset the checklist. */
        for (unsigned i = 0; i < FLEET_MAX; i++)
            fleet[i].acked_valid = false;
        last_cmd_id = id;
        memcpy(last_cmd_pl, pl, len);
        last_cmd_len = len;
    }
    mesh_poll(&mesh, now);
    save_root_counters(); /* command IDs must survive a base reboot */
    printf("flooded cmd %u (%u bytes); watching for ACKs\n", id, len);
}

static void print_top(void)
{
    printf("node | parent | state   | batt | temp | acked cmd %u\n",
           last_cmd_id);
    printf("-----+--------+---------+------+------+----------\n");
    for (unsigned i = 1; i < FLEET_MAX; i++) {
        if (!fleet[i].seen)
            continue;
        const fleet_entry_t *e = &fleet[i];
        printf(" %3u |   %3u  | %-7s | %u.%uV | %3dC | %s%s\n", i,
               e->parent, (e->state & 1) ? "ARMED" : "standby",
               e->batt_dv / 10, e->batt_dv % 10, e->temp_c,
               (e->acked_valid && e->acked_cmd == last_cmd_id) ? "yes"
                                                               : "NO",
               (e->state & 2) ? "" : "  [no gps]");
        for (unsigned k = 0; k < MESH_TELEM_NBRS; k++) {
            if (e->nbr_id[k] != MESH_ID_NONE)
                printf("     |        |   sees %u at %u.%02u ETX\n",
                       e->nbr_id[k], e->nbr_etx[k] >> 8,
                       ((e->nbr_etx[k] & 0xFF) * 100) >> 8);
        }
    }
}

/* ---- bulk fetch commands (§7.3, base-driven) ---- */

static void start_fetch(uint8_t node, uint32_t bytes)
{
    uint64_t now = app_now();
    if (bytes == 0)
        bytes = 1;
    if (bytes > sizeof(fx.buf))
        bytes = sizeof(fx.buf);
    memset(&fx, 0, sizeof(fx));
    fx.target = node;
    fx.session = next_session++;
    if (next_session >= 128) /* 128+ belongs to the campaign engine */
        next_session = 1;
    fx.chunk = MESH_BULK_CHUNK;
    fx.total = bytes;
    fx.nchunks = (bytes + fx.chunk - 1) / fx.chunk;
    if (!mesh_root_fetch(&mesh, node, fx.session, 0, 0, bytes, fx.chunk,
                         now)) {
        printf("fetch failed to start (queue full?)\n");
        return;
    }
    fx.active = true;
    mesh_poll(&mesh, now);
    printf("fetch: session %u — %lu bytes in %u x %u B chunks from "
           "node %u\n", fx.session, (unsigned long)bytes, fx.nchunks,
           fx.chunk, node);
}

static unsigned first_missing_chunk(void)
{
    unsigned i = 0;
    while (i < fx.nchunks && ((fx.mask >> i) & 1))
        i++;
    return i;
}

static void refetch(void)
{
    if (!fx.active) {
        printf("no fetch session (use: fetch <node> [bytes])\n");
        return;
    }
    unsigned first = first_missing_chunk();
    if (first >= fx.nchunks) {
        printf("fetch already complete\n");
        return;
    }
    uint32_t off = (uint32_t)first * fx.chunk;
    uint64_t now = app_now();
    if (!mesh_root_fetch(&mesh, fx.target, fx.session, 0, off,
                         fx.total - off, fx.chunk, now)) {
        printf("refetch failed (queue full?)\n");
        return;
    }
    mesh_poll(&mesh, now);
    printf("refetch: session %u from chunk %u (offset %lu)\n",
           fx.session, first, (unsigned long)off);
}

static void print_fstat(void)
{
    if (!fx.active) {
        printf("no fetch session\n");
        return;
    }
    printf("fetch: node %u session %u — %u/%u chunks, %lu bytes "
           "requested\n", fx.target, fx.session, fx.got, fx.nchunks,
           (unsigned long)fx.total);
    if (fx.got >= fx.nchunks) {
        printf("  complete; crc16 %04X\n", mesh_crc16(fx.buf, fx.total));
        return;
    }
    printf("  missing:");
    unsigned shown = 0;
    for (unsigned i = 0; i < fx.nchunks && shown < 16; i++) {
        if (!((fx.mask >> i) & 1)) {
            printf(" %u", i);
            shown++;
        }
    }
    printf("%s — `refetch` resumes from the first gap\n",
           (fx.nchunks - fx.got > 16) ? " ..." : "");
}

/* ---- M6: retrieval campaign CLI ---- */

/* Tree depth of a node per the telemetry topology map (bounded walk). */
static unsigned depth_of(uint8_t node)
{
    unsigned d = 0;
    uint8_t at = node;
    while (at != MESH_ID_ROOT && at != MESH_ID_NONE && d < 16) {
        at = fleet[at].parent;
        d++;
    }
    return d;
}

static void start_retrieve(const char *args)
{
    uint8_t targets[BP_MAX_NODES];
    unsigned n = 0;

    unsigned one;
    if (sscanf(args, "%u", &one) == 1 && one > 0 && one < 255) {
        targets[n++] = (uint8_t)one;
    } else {
        /* All nodes ever seen, nearest-first (§7.4: serialized walk,
         * shortest paths first so early failures cost least). */
        for (unsigned d = 1; d <= 16 && n < BP_MAX_NODES; d++) {
            for (unsigned i = 1; i < FLEET_MAX && n < BP_MAX_NODES;
                 i++) {
                if (fleet[i].seen && depth_of((uint8_t)i) == d)
                    targets[n++] = (uint8_t)i;
            }
        }
    }
    if (n == 0) {
        printf("no known nodes (run `status` first, or: retrieve "
               "<node>)\n");
        return;
    }

    /* With GPS/scheduler: full campaign incl. fleet mode switch to
     * retrieval and back to the default active mode. Bench: transfers
     * only. */
    if (!bp_start(&bp, targets, n, use_gps, FLEET_MODE_ACTIVE,
                  app_now())) {
        printf("campaign already running (see `rstat`)\n");
        return;
    }
    printf("retrieval campaign: %u node%s%s\n", n, n == 1 ? "" : "s",
           use_gps ? ", switching fleet to retrieval mode" : "");
}

static void print_rstat(void)
{
    static char out[512];
    bp_status(&bp, out, sizeof(out));
    fputs(out, stdout);
}

static void cmd_mode(const char *args)
{
    uint8_t id;
    if (strncmp(args, "active", 6) == 0)
        id = FLEET_MODE_ACTIVE;
    else if (strncmp(args, "standby", 7) == 0)
        id = FLEET_MODE_STANDBY;
    else if (strncmp(args, "retrieval", 9) == 0)
        id = FLEET_MODE_RETRIEVAL;
    else {
        printf("usage: mode active|standby|retrieval\n");
        return;
    }
    if (!use_gps) {
        printf("modes need the scheduler (GPS lock at boot)\n");
        return;
    }
    mesh_sched_mode_t md;
    fleet_mode_by_id(id, &md);
    uint64_t at = app_now() + 2 * mesh.sched.mode.period_us;
    if (!mesh_sched_set_mode(&mesh, &md, at)) {
        printf("mode switch already pending\n");
        return;
    }
    uint8_t pl[FLEET_MODE_PLEN];
    fleet_mode_cmd(pl, id, at);
    flood(pl, sizeof(pl), 0);
    printf("fleet mode -> %s in %llus (watch the ACK checklist; "
           "`resend` for stragglers)\n", args,
           (unsigned long long)((at - app_now()) / 1000000));
}

static void print_stats(void)
{
    const mesh_stats_t *m = mesh_stats_get(&mesh);
    const rylr998_stats_t *r = rylr998_stats(&radio);
    printf("mesh: rx %lu (drop len/net/mic %lu/%lu/%lu) tx %lu "
           "beacons_rx %lu acks %lu\n",
           (unsigned long)m->rx_frames, (unsigned long)m->rx_drop_len,
           (unsigned long)m->rx_drop_net, (unsigned long)m->rx_drop_mic,
           (unsigned long)m->tx_frames, (unsigned long)m->beacons_rx,
           (unsigned long)m->acks_sent);
    printf("radio: sent %lu busy %lu err %lu | rcv %lu bad %lu | "
           "timeouts %lu resets %lu\n",
           (unsigned long)r->tx_sent, (unsigned long)r->tx_busy,
           (unsigned long)r->tx_err, (unsigned long)r->rx_frames,
           (unsigned long)r->rx_bad, (unsigned long)r->cmd_timeouts,
           (unsigned long)r->unexpected_resets);
}

static void run_cli_line(char *line)
{
    if (provision_cli(line)) {
        /* identity/key management (M4) — handled in provision_cli.c */
    } else if (strncmp(line, "arm", 3) == 0) {
        unsigned minutes = 60;
        (void)sscanf(line + 3, "%u", &minutes);
        if (use_gps) {
            /* §6.2: absolute GPS validity window <start, stop>. Stale
             * or replayed copies are inert once the window passes. */
            uint64_t start = app_now();
            uint64_t stop = start + (uint64_t)minutes * 60000000ull;
            uint8_t pl[17];
            pl[0] = 'A';
            for (int i = 0; i < 8; i++) {
                pl[1 + i] = (uint8_t)(start >> (8 * i));
                pl[9 + i] = (uint8_t)(stop >> (8 * i));
            }
            flood(pl, sizeof(pl), 0);
        } else {
            /* Bench fallback: duration form. */
            uint8_t pl[3] = { 'A', (uint8_t)(minutes & 0xFF),
                              (uint8_t)(minutes >> 8) };
            flood(pl, sizeof(pl), 0);
        }
    } else if (strcmp(line, "disarm") == 0) {
        uint8_t pl[1] = { 'D' };
        flood(pl, sizeof(pl), 0);
    } else if (strcmp(line, "status") == 0) {
        uint8_t pl[1] = { 'S' };
        flood(pl, sizeof(pl), 0);
    } else if (strcmp(line, "resend") == 0) {
        if (last_cmd_id)
            flood(last_cmd_pl, last_cmd_len, last_cmd_id);
        else
            printf("nothing to resend\n");
    } else if (strcmp(line, "top") == 0) {
        print_top();
    } else if (strcmp(line, "stats") == 0) {
        print_stats();
    } else if (strncmp(line, "list", 4) == 0) {
        unsigned node = 0;
        if (sscanf(line + 4, "%u", &node) == 1 && node > 0 && node < 255) {
            /* LIST = bulk session 0 by convention: the node streams its
             * recording index back as text. (This replaces any fetch
             * session in progress on that node — `refetch` resumes.) */
            uint64_t now = app_now();
            if (mesh_root_fetch(&mesh, (uint8_t)node, 0, 0, 0, 128, 32,
                                now))
                printf("list: asked node %u for its index\n", node);
            else
                printf("list failed (queue full?)\n");
            mesh_poll(&mesh, now);
        } else {
            printf("usage: list <node>\n");
        }
    } else if (strncmp(line, "fetch", 5) == 0) {
        unsigned node = 0;
        unsigned long bytes = 4096; /* the example nodes' event size */
        if (sscanf(line + 5, "%u %lu", &node, &bytes) >= 1 && node > 0 &&
            node < 255)
            start_fetch((uint8_t)node, (uint32_t)bytes);
        else
            printf("usage: fetch <node> [bytes]\n");
    } else if (strcmp(line, "refetch") == 0) {
        refetch();
    } else if (strcmp(line, "fstat") == 0) {
        print_fstat();
    } else if (strncmp(line, "retrieve", 8) == 0) {
        start_retrieve(line + 8 + (line[8] == ' ' ? 1 : 0));
    } else if (strcmp(line, "rstat") == 0) {
        print_rstat();
    } else if (strncmp(line, "mode ", 5) == 0) {
        cmd_mode(line + 5);
    } else if (line[0] != '\0') {
        printf("commands: arm [min] | disarm | status | resend | top | "
               "stats | list <n> | fetch <n> [bytes] | refetch | "
               "fstat | retrieve [n] | rstat | mode <m> | provision\n");
    }
}

static void cli_tick(void)
{
    static char line[64];
    static unsigned len;
    for (;;) {
        int c = getchar_timeout_us(0);
        if (c == PICO_ERROR_TIMEOUT)
            return;
        if (c == '\r' || c == '\n') {
            printf("\n");
            line[len] = '\0';
            len = 0;
            run_cli_line(line);
            printf("> ");
        } else if (len < sizeof(line) - 1 && c >= 0x20 && c < 0x7F) {
            line[len++] = (char)c;
            putchar(c); /* echo */
        }
    }
}

int main(void)
{
    stdio_init_all();
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    gpio_put(PICO_DEFAULT_LED_PIN, 1); /* root is always "attached" */

    uint64_t now = time_us_64();

    /* Identity + network key: from the flash provisioning blob when one
     * is stored (M4), else the compiled node_config.h bench defaults.
     * The base always runs as node 0 regardless of the blob's ID. */
    prov_t prov;
    provision_boot(&prov, MESH_ID_ROOT);
    if (prov.node_id != MESH_ID_ROOT)
        printf("note: blob says node %u, but base firmware is always "
               "node 0 (using the blob's net+key)\n", prov.node_id);

    rylr998_cfg_t rcfg = {
        .address = MESH_ID_ROOT,
        .network_id = RADIO_MODULE_NETID,
        .band_hz = RADIO_BAND_HZ,
        .sf = RADIO_SF,
        .bw = RADIO_BW,
        .cr = RADIO_CR,
        .preamble = RADIO_PREAMBLE,
        .power_dbm = RADIO_POWER_DBM,
    };
    rylr998_pico_init(&radio, &rcfg, on_radio_frame, NULL, now);

    mesh_mic_cmac_init(&cmac, prov.key);

    /* GPS (M5): the base anchors the fleet timebase, so it waits for a
     * lock like every field node. Without one it falls back to bench
     * mode (application-timed beacons, scheduler off fleet-wide). */
    gps_pico_init(&gpsp);
    printf("gps: waiting up to %us for lock...\n",
           (unsigned)(GPS_WAIT_US / 1000000));
    uint64_t gps_deadline = time_us_64() + GPS_WAIT_US;
    while (time_us_64() < gps_deadline) {
        uint64_t t = time_us_64();
        gps_pico_pump(&gpsp);
        rylr998_pico_pump(&radio, t);
        rylr998_poll(&radio, t);
        if (gps_locked(&gpsp.gps, t)) {
            use_gps = true;
            break;
        }
        sleep_ms(2);
    }
    if (use_gps)
        printf("gps: locked%s — scheduler on, auto-beacons\n",
               gpsp.gps.pps_ever ? " (PPS)" : " (no PPS)");
    else
        printf("gps: no lock — bench mode, manual beacon timer\n");

    mesh_cfg_t cfg = {
        .node_id = MESH_ID_ROOT,
        .network_id = prov.net_id,
        .radio = rylr998_radio_ops(&radio),
        .mic = mesh_mic_cmac_ops(&cmac),
        .cb = {
            .on_command_ack = on_command_ack,
            .on_telemetry = on_telemetry,
            .on_bulk_data = on_bulk_data,
        },
        .rng_seed = 0xB05EBA5E,
    };
    if (use_gps) {
        cfg.sched.enabled = true;
        fleet_mode_by_id(FLEET_MODE_ACTIVE, &cfg.sched.mode);
    }
    if (!mesh_init(&mesh, &cfg)) {
        printf("mesh_init failed (bad config)\n");
        for (;;)
            tight_loop_contents();
    }

    /* Restore the root's persisted sequence counters (advanced by the
     * safety margin) so this reboot doesn't reset the fleet's notion
     * of the current epoch — see mesh_route_load. */
    if (mesh_route_load(&mesh, flash_store_read(FLASH_SLOT_ROUTE),
                        MESH_ROUTE_BLOB_MAX))
        printf("counters: resumed from flash (+margin)\n");
    save_root_counters();

    bp_init(&bp, &mesh, bp_buf, sizeof(bp_buf));

    printf("\n=== blast-sensor base station (root) ===\n> ");

    uint64_t next_beacon = time_us_64(); /* bench mode only */
    uint64_t next_counter_save = time_us_64() + 600000000ull;

    for (;;) {
        uint64_t local = time_us_64();
        uint64_t mnow = app_now();

        rylr998_pico_pump(&radio, local);
        rylr998_poll(&radio, local);
        gps_pico_pump(&gpsp);
        if (use_gps)
            mesh_sched_clock_uncertainty(
                &mesh, gps_uncertainty_us(&gpsp.gps, local));
        mesh_poll(&mesh, mnow);
        bp_poll(&bp, mnow);
        {
            static bool bp_was_busy;
            if (bp_was_busy && !bp_busy(&bp)) {
                printf("retrieval campaign finished:\n");
                print_rstat();
                printf("> ");
            }
            bp_was_busy = bp_busy(&bp);
        }

        /* Scheduler on: the library beacons at each beacon-window
         * start. Bench mode: application-timed beacons as in M1. */
        if (!use_gps && rylr998_ready(&radio) && local >= next_beacon) {
            next_beacon = local + SUPERFRAME_US;
            mesh_root_beacon(&mesh, mnow);
            mesh_poll(&mesh, mnow);
        }

        /* Periodic counter save (10 min — well inside the restore
         * margin of MESH_PERSIST_SEQ_MARGIN epochs). */
        if (local >= next_counter_save) {
            next_counter_save = local + 600000000ull;
            save_root_counters();
        }

        cli_tick();
        sleep_us(500);
    }
}
