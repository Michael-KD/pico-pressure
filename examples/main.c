/*
 * main.c — field-node example firmware (Raspberry Pi Pico + RYLR998).
 *
 * A real field device built on the mesh library: joins the collection
 * tree, executes flooded commands (simplified ARM/DISARM/STATUS from
 * design doc §6), and reports telemetry up the tree. All frames are
 * authenticated with AES-128-CMAC under the pre-shared network key in
 * node_config.h.
 *
 * What is deliberately simplified relative to the production firmware:
 *   - Time: time_us_64() (µs since boot) instead of GPS-disciplined
 *     absolute time. The mesh only needs a monotonic local clock, but
 *     the library's superframe scheduler (§4, milestone M2) needs a
 *     fleet-shared timebase, so cfg.sched stays disabled here until
 *     the GPS/PPS time source lands (M5). Likewise ARM carries a
 *     duration instead of an absolute validity window.
 *   - Sensors: the ARMED state toggles a flag and the LED; the 1 kHz
 *     pressure pipeline, SD logging, and the full §9 state machine live
 *     in the logger application, not in this example.
 *
 * Set NODE_ID in node_config.h (unique per device!) before flashing.
 * Logs go to USB serial.
 */
#include <stdio.h>
#include <string.h>

#include "hardware/adc.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"

#include "mesh/mesh.h"
#include "node_config.h"
#include "rylr998_pico.h"

static mesh_t mesh;
static rylr998_t radio;
static mesh_mic_cmac_t cmac;

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
    uint16_t up_min = (uint16_t)(now / 60000000ull);
    pl[0] = armed ? 1 : 0;               /* state */
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

/* ---- mesh callbacks ---- */

static uint64_t telem_due; /* STATUS pulls the next report forward */

static bool on_command(void *user, uint16_t cmd_id,
                       const uint8_t *payload, uint8_t len)
{
    (void)user;
    uint64_t now = time_us_64();
    if (len < 1)
        return false;

    switch (payload[0]) {
    case 'A': { /* ARM <duration minutes, 2B LE> */
        if (len < 3)
            return false;
        uint16_t minutes = (uint16_t)(payload[1] | (payload[2] << 8));
        /* Production firmware: validate the absolute GPS validity
         * window here (§6) so stale/replayed ARMs are inert. */
        armed = true;
        armed_until = now + (uint64_t)minutes * 60000000ull;
        printf("cmd %u: ARM for %u min\n", cmd_id, minutes);
        return true;
    }
    case 'D': /* DISARM */
        armed = false;
        printf("cmd %u: DISARM\n", cmd_id);
        return true;
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
}

/* ---- radio RX -> mesh ---- */

static void on_radio_frame(void *ctx, const uint8_t *frame, size_t len,
                           int16_t rssi_dbm, int8_t snr_db)
{
    (void)ctx;
    mesh_rx(&mesh, frame, len, rssi_dbm, snr_db, time_us_64());
}

/* ---- LED heartbeat: slow blink = attached, fast = searching ---- */

static void led_tick(uint64_t now)
{
    uint64_t period = (mesh_parent(&mesh) != MESH_ID_NONE) ? 2000000
                                                           : 250000;
    gpio_put(PICO_DEFAULT_LED_PIN, (now / period) & 1);
}

int main(void)
{
    stdio_init_all();
    adc_init();
    adc_set_temp_sensor_enabled(true);
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

    uint64_t now = time_us_64();
    printf("\n=== blast-sensor field node %u ===\n", NODE_ID);

    /* Radio: bring the RYLR998 up with the deployment RF profile. */
    rylr998_cfg_t rcfg = {
        .address = NODE_ID, /* module addr = node ID (debug nicety) */
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
    static const uint8_t key[16] = MESH_NET_KEY;
    mesh_mic_cmac_init(&cmac, key);

    /* RNG seed must differ per node: mix the flash unique ID. */
    pico_unique_board_id_t uid;
    pico_get_unique_board_id(&uid);
    uint32_t seed = 0x9E3779B9u * (NODE_ID + 1);
    for (unsigned i = 0; i < PICO_UNIQUE_BOARD_ID_SIZE_BYTES; i++)
        seed = seed * 31 + uid.id[i];

    mesh_cfg_t cfg = {
        .node_id = NODE_ID,
        .network_id = MESH_NET_ID,
        .radio = rylr998_radio_ops(&radio),
        .mic = mesh_mic_cmac_ops(&cmac),
        .cb = {
            .on_command = on_command,
            .on_parent_change = on_parent_change,
        },
        .rng_seed = seed ? seed : 1,
    };
    if (!mesh_init(&mesh, &cfg)) {
        printf("mesh_init failed (bad config)\n");
        for (;;)
            tight_loop_contents();
    }

    telem_due = now + TELEM_PERIOD_US;
    bool radio_was_ready = false;

    for (;;) {
        now = time_us_64();

        /* The whole stack is pumped from this single loop: UART bytes
         * into the AT driver (which delivers frames into mesh_rx), then
         * driver timeouts, then mesh scheduling. */
        rylr998_pico_pump(&radio, now);
        rylr998_poll(&radio, now);
        mesh_poll(&mesh, now);

        if (!radio_was_ready && rylr998_ready(&radio)) {
            radio_was_ready = true;
            printf("radio: RYLR998 configured (%.0f MHz, SF%u)\n",
                   (double)RADIO_BAND_HZ / 1e6, RADIO_SF);
        }

        if (armed && now >= armed_until) {
            armed = false;
            printf("arm window elapsed; disarmed\n");
        }

        if (now >= telem_due) {
            telem_due = now + TELEM_PERIOD_US;
            send_telemetry(now);
            mesh_poll(&mesh, now); /* flush the enqueued report */
        }

        led_tick(now);
        sleep_us(500); /* M2 replaces this with scheduled sleep */
    }
}
