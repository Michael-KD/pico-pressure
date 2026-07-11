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
 */
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"

#include "mesh/mesh.h"
#include "node_config.h"
#include "rylr998_pico.h"

static mesh_t mesh;
static rylr998_t radio;
static mesh_mic_cmac_t cmac;

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
static uint8_t last_cmd_pl[8];
static uint8_t last_cmd_len;

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
    printf("TELEM: node %u (parent %u, %s, %u.%uV, %dC)\n", info->origin,
           info->parent, e->state ? "ARMED" : "standby", e->batt_dv / 10,
           e->batt_dv % 10, e->temp_c);
}

static void on_radio_frame(void *ctx, const uint8_t *frame, size_t len,
                           int16_t rssi_dbm, int8_t snr_db)
{
    (void)ctx;
    mesh_rx(&mesh, frame, len, rssi_dbm, snr_db, time_us_64());
}

/* ---- commands ---- */

static void flood(const uint8_t *pl, uint8_t len, uint16_t reuse_id)
{
    uint64_t now = time_us_64();
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
        printf(" %3u |   %3u  | %-7s | %u.%uV | %3dC | %s\n", i,
               e->parent, e->state ? "ARMED" : "standby", e->batt_dv / 10,
               e->batt_dv % 10, e->temp_c,
               (e->acked_valid && e->acked_cmd == last_cmd_id) ? "yes"
                                                               : "NO");
        for (unsigned k = 0; k < MESH_TELEM_NBRS; k++) {
            if (e->nbr_id[k] != MESH_ID_NONE)
                printf("     |        |   sees %u at %u.%02u ETX\n",
                       e->nbr_id[k], e->nbr_etx[k] >> 8,
                       ((e->nbr_etx[k] & 0xFF) * 100) >> 8);
        }
    }
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
    if (strncmp(line, "arm", 3) == 0) {
        unsigned minutes = 60;
        (void)sscanf(line + 3, "%u", &minutes);
        uint8_t pl[3] = { 'A', (uint8_t)(minutes & 0xFF),
                          (uint8_t)(minutes >> 8) };
        flood(pl, sizeof(pl), 0);
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
    } else if (line[0] != '\0') {
        printf("commands: arm [min] | disarm | status | resend | top | "
               "stats\n");
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

    static const uint8_t key[16] = MESH_NET_KEY;
    mesh_mic_cmac_init(&cmac, key);

    mesh_cfg_t cfg = {
        .node_id = MESH_ID_ROOT,
        .network_id = MESH_NET_ID,
        .radio = rylr998_radio_ops(&radio),
        .mic = mesh_mic_cmac_ops(&cmac),
        .cb = {
            .on_command_ack = on_command_ack,
            .on_telemetry = on_telemetry,
        },
        .rng_seed = 0xB05EBA5E,
    };
    if (!mesh_init(&mesh, &cfg)) {
        printf("mesh_init failed (bad config)\n");
        for (;;)
            tight_loop_contents();
    }

    printf("\n=== blast-sensor base station (root) ===\n> ");

    uint64_t next_beacon = now; /* beacon immediately once radio is up */

    for (;;) {
        now = time_us_64();

        rylr998_pico_pump(&radio, now);
        rylr998_poll(&radio, now);
        mesh_poll(&mesh, now);

        /* Superframe scheduling is application policy until M2: one
         * beacon per SUPERFRAME_US, first one when the radio is ready. */
        if (rylr998_ready(&radio) && now >= next_beacon) {
            next_beacon = now + SUPERFRAME_US;
            mesh_root_beacon(&mesh, now);
            mesh_poll(&mesh, now);
        }

        cli_tick();
        sleep_us(500);
    }
}
