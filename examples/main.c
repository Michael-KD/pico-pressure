/*
 * main.c — field-node example firmware (Raspberry Pi Pico + RYLR998).
 *
 * A real field device built on the mesh library: joins the collection
 * tree, executes flooded commands (simplified ARM/DISARM/STATUS from
 * design doc §6), and reports telemetry up the tree. All frames are
 * authenticated with AES-128-CMAC under the pre-shared network key —
 * from the flash provisioning blob (`provision` on the USB CLI, M4),
 * falling back to the node_config.h bench defaults.
 *
 * Time (M5): at boot the node waits for a GPS lock (NMEA + optional
 * PPS, gps.[ch]); once locked, mesh time is fleet-shared absolute GPS
 * time, the §4 superframe scheduler runs (windows, duty cycling), ARM
 * validity windows are absolute (§6.2), and the §10.2 superframe-bound
 * replay protection is live. Without a lock the node free-runs
 * scheduler-off — bench mode, clearly flagged in the log. The §5.3
 * flash route cache resumes the pre-reboot route instantly.
 *
 * What is deliberately simplified relative to the production firmware:
 *   - Sensors: the ARMED state toggles a flag and the LED; the 1 kHz
 *     pressure pipeline, SD logging, and the full §9 state machine live
 *     in the logger application, not in this example.
 *   - Stored data: bulk FETCH (§7) serves a deterministic synthetic
 *     trace instead of real event windows off the SD card, so
 *     retrieval can be exercised and CRC-verified on the bench.
 *
 * Flash the same UF2 to every field node, then personalize each over
 * USB serial: `provision <id> <netid> <key>` (unique ID per device!).
 * Logs go to USB serial.
 */
#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "flash_store.h"
#include "fleet_modes.h"
#include "gps_pico.h"
#include "mesh/mesh.h"
#include "node_config.h"
#include "provision_cli.h"
#include "rylr998_pico.h"

static mesh_t mesh;
static rylr998_t radio;
static mesh_mic_cmac_t cmac;
static gps_pico_t gpsp;

/* Runtime identity: the flash provisioning blob when present (M4), else
 * the compiled NODE_ID bench default. */
static uint8_t node_id;

/* True when GPS locked at boot: mesh time is absolute GPS time and the
 * superframe scheduler is on (M5). False = bench free-run, M1 style. */
static bool use_gps;
static uint64_t mono_last;

/* Route cache (§5.3): dirty on parent change, written rate-limited. */
static bool route_dirty;
static uint64_t route_saved_at;

/* Mesh time: GPS absolute, monotonically clamped (discipline updates
 * may nudge the offset backward a hair; the mesh requires monotonic
 * now), or the boot-local clock when free-running. */
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

/* ---- application state (simplified §9) ---- */

static bool armed;
static uint64_t armed_until;

/* ---- sensors (battery/temperature for telemetry) ---- */

static uint16_t read_vsys_mv(void)
{
    adc_select_input(3); /* VSYS / 3 */
    uint32_t raw = adc_read();
    return (uint16_t)((raw * 3 * 3300) / 4096);
}

static int8_t read_temp_c(void)
{
    adc_select_input(4); /* internal temperature sensor */
    uint32_t raw = adc_read();
    /* T = 27 - (V - 0.706) / 0.001721, in integer µV. */
    int32_t uv = (int32_t)((raw * 3300000ull) / 4096);
    return (int8_t)(27 - (uv - 706000) / 1721);
}

/* ---- telemetry ---- */

static void send_telemetry(uint64_t now)
{
    uint8_t pl[5];
    uint16_t up_min = (uint16_t)(time_us_64() / 60000000ull);
    pl[0] = (uint8_t)((armed ? 1 : 0) |  /* state: bit0 armed, */
                      (gps_locked(&gpsp.gps, time_us_64()) ? 2
                                                           : 0)); /* bit1 gps */
    pl[1] = (uint8_t)(read_vsys_mv() / 100); /* battery, decivolts */
    pl[2] = (uint8_t)read_temp_c();      /* °C */
    pl[3] = (uint8_t)(up_min & 0xFF);    /* uptime, minutes LE */
    pl[4] = (uint8_t)(up_min >> 8);
    if (mesh_node_telemetry(&mesh, pl, sizeof(pl), now))
        printf("telem: sent (state=%u vsys=%u.%uV temp=%dC)\n",
               pl[0], pl[1] / 10, pl[1] % 10, (int8_t)pl[2]);
    else
        printf("telem: skipped (no parent yet)\n");
}

/* ---- synthetic "recording" served over bulk FETCH (§7.3) ----
 * The production firmware serves real event windows off the SD card
 * (LIST index -> byte ranges). This example serves a deterministic
 * 4 kB synthetic trace so retrieval can be exercised — and verified by
 * CRC — on the bench. Session 0 is the LIST convention: it returns the
 * recording index as text. */

#define EVENT_LEN 4096u

static uint8_t event_byte(uint32_t off)
{
    return (uint8_t)(0x80 ^ (uint8_t)(node_id * 37) ^
                     (uint8_t)(off * 13) ^ (uint8_t)(off >> 5));
}

/* CRC16 of the whole synthetic event, streamed without a 4 kB buffer
 * (the production firmware does the same over SD reads). */
static uint16_t event_crc16(void)
{
    uint16_t crc = 0xFFFF;
    for (uint32_t o = 0; o < EVENT_LEN; o++)
        crc = mesh_crc16_step(crc, event_byte(o));
    return crc;
}

static uint8_t on_bulk_read(void *user, uint8_t session, uint32_t off,
                            uint8_t *buf, uint8_t max)
{
    (void)user;
    static uint8_t noisy_session = 0xFF;
    if (session != noisy_session) {
        noisy_session = session;
        printf("bulk: serving session %u (event crc16 %04X)\n", session,
               event_crc16());
    }

    uint8_t i = 0;
    if (session == 0) {
        /* LIST: one index line; a short read marks its end. */
        char idx[64];
        int n = snprintf(idx, sizeof(idx), "node %u: evt 1, %u B, "
                         "crc16 %04X", node_id, (unsigned)EVENT_LEN,
                         event_crc16());
        while (i < max && off + i < (uint32_t)n) {
            buf[i] = (uint8_t)idx[off + i];
            i++;
        }
        return i;
    }
    while (i < max && off + i < EVENT_LEN) {
        buf[i] = event_byte(off + i);
        i++;
    }
    return i;
}

/* ---- mesh callbacks ---- */

static uint64_t telem_due; /* STATUS pulls the next report forward */

static uint64_t get_u64le(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--)
        v = (v << 8) | p[i];
    return v;
}

static bool on_command(void *user, uint16_t cmd_id,
                       const uint8_t *payload, uint8_t len)
{
    (void)user;
    uint64_t now = app_now();
    if (len < 1)
        return false;

    switch (payload[0]) {
    case 'A':
        /* ARM. Two forms (§6.2):
         *   17 B: absolute GPS validity window <start u64, stop u64> —
         *         the production semantics; stale or replayed ARMs are
         *         inert because the window has passed.
         *    3 B: duration minutes (2B LE) — bench fallback for fleets
         *         without GPS time. */
        if (len == 17) {
            if (!use_gps) {
                printf("cmd %u: ARM window but no GPS time; "
                       "rejected\n", cmd_id);
                return false;
            }
            uint64_t start = get_u64le(&payload[1]);
            uint64_t stop = get_u64le(&payload[9]);
            if (now < start || now >= stop) {
                printf("cmd %u: ARM outside validity window; "
                       "ignored (§6.2)\n", cmd_id);
                return false;
            }
            armed = true;
            armed_until = stop;
            printf("cmd %u: ARM until +%lus (absolute window)\n",
                   cmd_id, (unsigned long)((stop - now) / 1000000));
            return true;
        }
        if (len == 3) {
            uint16_t minutes =
                (uint16_t)(payload[1] | (payload[2] << 8));
            armed = true;
            armed_until = now + (uint64_t)minutes * 60000000ull;
            printf("cmd %u: ARM for %u min (duration form)\n", cmd_id,
                   minutes);
            return true;
        }
        return false;
    case 'D': /* DISARM */
        armed = false;
        printf("cmd %u: DISARM\n", cmd_id);
        return true;
    case FLEET_CMD_MODE: { /* fleet mode switch (M6, §4) */
        uint8_t mode_id;
        uint64_t at;
        mesh_sched_mode_t md;
        if (!use_gps || !fleet_mode_parse(payload, len, &mode_id, &at) ||
            !fleet_mode_by_id(mode_id, &md))
            return false;
        if (!mesh_sched_set_mode(&mesh, &md, at))
            return false; /* another switch pending: no ACK, re-flood
                           * catches us later */
        printf("cmd %u: MODE %u at +%lus\n", cmd_id, mode_id,
               (unsigned long)((at - now) / 1000000));
        return true;
    }
    case 'S': /* STATUS: report in the next second */
        telem_due = now + 1000000;
        printf("cmd %u: STATUS\n", cmd_id);
        return true;
    default:
        printf("cmd %u: unknown opcode 0x%02x (no ACK)\n", cmd_id,
               payload[0]);
        return false; /* not ACKed; base will see the gap */
    }
}

static void on_parent_change(void *user, uint8_t old_parent,
                             uint8_t new_parent)
{
    (void)user;
    /* Production firmware appends this to the SD event log (§9). */
    printf("mesh: parent %u -> %u (cost %u.%02u ETX, hop %u)\n",
           old_parent, new_parent, mesh_path_cost(&mesh) >> 8,
           ((mesh_path_cost(&mesh) & 0xFF) * 100) >> 8, mesh_hops(&mesh));
    route_dirty = true; /* flash route cache (§5.3), written from loop */
}

/* ---- radio RX -> mesh ---- */

static void on_radio_frame(void *ctx, const uint8_t *frame, size_t len,
                           int16_t rssi_dbm, int8_t snr_db)
{
    (void)ctx;
    mesh_rx(&mesh, frame, len, rssi_dbm, snr_db, app_now());
}

/* ---- LED heartbeat: slow blink = attached, fast = searching ---- */

static void led_tick(uint64_t now)
{
    uint64_t period = (mesh_parent(&mesh) != MESH_ID_NONE) ? 2000000
                                                           : 250000;
    gpio_put(PICO_DEFAULT_LED_PIN, (now / period) & 1);
}

/* ---- minimal CLI: provisioning + diagnostics (M4) ----
 * The field node's USB serial is primarily a log, but it accepts enough
 * of a CLI to personalize a freshly flashed generic UF2. */

static void run_cli_line(char *line)
{
    if (provision_cli(line))
        return;
    if (strcmp(line, "stats") == 0) {
        const mesh_stats_t *s = mesh_stats_get(&mesh);
        printf("mesh: rx %lu (drop mic %lu) tx %lu parent %u "
               "cost %u.%02u\n", (unsigned long)s->rx_frames,
               (unsigned long)s->rx_drop_mic,
               (unsigned long)s->tx_frames, mesh_parent(&mesh),
               mesh_path_cost(&mesh) >> 8,
               ((mesh_path_cost(&mesh) & 0xFF) * 100) >> 8);
    } else if (line[0] != '\0') {
        printf("commands: provision [show|clear|<id> <netid> <key>] | "
               "stats\n");
    }
}

static void cli_tick(void)
{
    static char line[80];
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
        } else if (len < sizeof(line) - 1 && c >= 0x20 && c < 0x7F) {
            line[len++] = (char)c;
            putchar(c); /* echo */
        }
    }
}

int main(void)
{
    stdio_init_all();
    adc_init();
    adc_set_temp_sensor_enabled(true);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    uint64_t now = time_us_64();

    /* Identity + network key (M4): flash provisioning blob if present,
     * else the compiled bench defaults. A blob claiming node 0 is a
     * base-station blob on field firmware — ignore its ID. */
    prov_t prov;
    provision_boot(&prov, NODE_ID);
    if (prov.node_id == MESH_ID_ROOT) {
        printf("blob says node 0 (base) but this is field firmware; "
               "using compiled NODE_ID %u\n", NODE_ID);
        prov.node_id = NODE_ID;
    }
    node_id = prov.node_id;

    printf("\n=== blast-sensor field node %u ===\n", node_id);

    /* Radio: bring the RYLR998 up with the deployment RF profile. */
    rylr998_cfg_t rcfg = {
        .address = node_id, /* module addr = node ID (debug nicety) */
        .network_id = RADIO_MODULE_NETID,
        .band_hz = RADIO_BAND_HZ,
        .sf = RADIO_SF,
        .bw = RADIO_BW,
        .cr = RADIO_CR,
        .preamble = RADIO_PREAMBLE,
        .power_dbm = RADIO_POWER_DBM,
    };
    rylr998_pico_init(&radio, &rcfg, on_radio_frame, NULL, now);

    /* MIC: real AES-128-CMAC under the deployment key. */
    mesh_mic_cmac_init(&cmac, prov.key);

    /* GPS (M5): wait for a first lock so mesh time is fleet-shared
     * absolute time and the superframe scheduler (with the §10.2
     * replay-protected MIC) can run. No lock within the window —
     * no module wired, or on a bench indoors — falls back to
     * free-running with the scheduler off. */
    gps_pico_init(&gpsp);
    printf("gps: waiting up to %us for lock (LED double-blinks while "
           "waiting)...\n", (unsigned)(GPS_WAIT_US / 1000000));
    uint64_t gps_deadline = time_us_64() + GPS_WAIT_US;
    while (time_us_64() < gps_deadline) {
        uint64_t t = time_us_64();
        gps_pico_pump(&gpsp);
        rylr998_pico_pump(&radio, t);
        rylr998_poll(&radio, t);
        gpio_put(PICO_DEFAULT_LED_PIN, ((t / 100000) % 10) < 2);
        if (gps_locked(&gpsp.gps, t)) {
            use_gps = true;
            break;
        }
        sleep_ms(2);
    }
    if (use_gps)
        printf("gps: locked%s — scheduler on, replay protection "
               "active\n", gpsp.gps.pps_ever ? " (PPS)" : " (no PPS)");
    else
        printf("gps: no lock — free-running, scheduler off (bench "
               "mode; replay protection dormant)\n");

    /* RNG seed must differ per node: mix the flash unique ID. */
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    uint32_t seed = 0x9E3779B9u * (uint32_t)(node_id + 1);
    for (unsigned i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
        seed = seed * 31 + uid.id[i];

    mesh_cfg_t cfg = {
        .node_id = node_id,
        .network_id = prov.net_id,
        .radio = rylr998_radio_ops(&radio),
        .mic = mesh_mic_cmac_ops(&cmac),
        .cb = {
            .on_command = on_command,
            .on_parent_change = on_parent_change,
            .on_bulk_read = on_bulk_read,
        },
        .rng_seed = seed ? seed : 1,
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

    /* Route cache (§5.3): resume the pre-reboot route immediately if a
     * valid snapshot is stored; repaired lazily from live beacons. */
    if (mesh_route_load(&mesh, flash_store_read(FLASH_SLOT_ROUTE),
                        MESH_ROUTE_BLOB_MAX))
        printf("route: resumed from flash (parent %u)\n",
               mesh_parent(&mesh));
    route_dirty = false; /* what's stored is what we just loaded */

    telem_due = app_now() + TELEM_PERIOD_US;
    bool radio_was_ready = false;
    uint64_t unc_next = 0;

    for (;;) {
        uint64_t local = time_us_64();
        uint64_t mnow = app_now();

        /* The whole stack is pumped from this single loop: UART bytes
         * into the AT driver (which delivers frames into mesh_rx) and
         * the GPS parser, then driver timeouts, then mesh scheduling.
         * The radio driver runs on the local clock; the mesh runs on
         * mesh time (GPS absolute when locked). */
        rylr998_pico_pump(&radio, local);
        rylr998_poll(&radio, local);
        gps_pico_pump(&gpsp);
        if (use_gps && local >= unc_next) {
            unc_next = local + 1000000;
            mesh_sched_clock_uncertainty(
                &mesh, gps_uncertainty_us(&gpsp.gps, local));
        }
        mesh_poll(&mesh, mnow);

        if (!radio_was_ready && rylr998_ready(&radio)) {
            radio_was_ready = true;
            printf("radio: RYLR998 configured (%.0f MHz, SF%u)\n",
                   (double)RADIO_BAND_HZ / 1e6, RADIO_SF);
        }

        if (armed && mnow >= armed_until) {
            armed = false;
            printf("arm window elapsed; disarmed\n");
        }

        if (mnow >= telem_due) {
            telem_due = mnow + TELEM_PERIOD_US;
            send_telemetry(mnow);
            mesh_poll(&mesh, mnow); /* flush the enqueued report */
        }

        /* Route cache write: rare (parent changes only), rate-limited,
         * and — when scheduled — deferred into the sleep segment so
         * the ~50 ms interrupts-off flash write can't eat a frame. */
        if (route_dirty && local - route_saved_at > 60000000ull &&
            (!use_gps || !mesh_sched_listening(&mesh))) {
            uint8_t blob[MESH_ROUTE_BLOB_MAX];
            size_t bl = mesh_route_save(&mesh, blob, sizeof(blob));
            if (bl > 0)
                flash_store_write(FLASH_SLOT_ROUTE, blob, bl);
            route_dirty = false;
            route_saved_at = local;
        }

        cli_tick();
        led_tick(local);
        /* The AT-driver needs sub-ms servicing while frames stream, so
         * this stays a paced busy loop; deep MCU sleep in the schedule
         * gap is logger-application territory (§9). */
        sleep_us(500);
    }
}
