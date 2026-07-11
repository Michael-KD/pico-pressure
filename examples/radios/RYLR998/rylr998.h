/*
 * rylr998.h — mesh radio driver for the REYAX RYLR998 LoRa module.
 *
 * The RYLR998 is a UART AT-command transceiver (see the datasheets in
 * this directory): commands go down as "AT+...\r\n" text, the module
 * answers "+OK"/"+ERR=n", and received packets arrive unsolicited as
 * "+RCV=<addr>,<len>,<data>,<RSSI>,<SNR>". This driver speaks that
 * protocol and exposes the library's mesh_radio_ops_t on top of it.
 *
 * Portability: this file is deliberately platform-free — no pico-sdk
 * includes. The application supplies a byte-write function (UART TX)
 * and feeds received UART bytes into rylr998_input(); see
 * ../../rylr998_pico.c for the Pico glue. That split lets the AT state
 * machine be unit-tested on the host (tests/unit/test_rylr998.c).
 *
 * Framing: the AT command guide specifies the <Data> field as ASCII
 * (max 240 bytes), so binary mesh frames are hex-encoded on the air —
 * 2 characters per byte. That caps a mesh frame at 120 bytes, ample for
 * every control frame (beacons/commands/telemetry are 15–40 B); revisit
 * for the bulk-transfer milestone. Because payloads are pure hex, +RCV
 * lines can never contain stray commas or CR/LF, which keeps the line
 * parser trivial and robust.
 *
 * Mesh addressing vs module addressing: the mesh does its own routing
 * with in-frame src/dst IDs and expects a broadcast medium, so every
 * frame is sent to module address 0 (= broadcast to all addresses).
 * The module's own address is set to the mesh node ID purely as a
 * debugging nicety (+RCV then shows the true transmitter).
 *
 * Single-threaded like the mesh library: call rylr998_input() and
 * rylr998_poll() from the main loop only, never from an ISR (the UART
 * ISR should just buffer bytes).
 */
#ifndef RYLR998_H
#define RYLR998_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mesh/mesh_radio.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Hex encoding halves the module's 240-byte AT payload ceiling. */
#define RYLR998_MAX_FRAME 120

/* Longest legitimate line: +RCV=65535,240,<240 hex>,-128,-20 */
#define RYLR998_LINE_MAX 300

/* Command/response timeout before the driver resends (config) or gives
 * up on the ack (send). */
#ifndef RYLR998_CMD_TIMEOUT_US
#define RYLR998_CMD_TIMEOUT_US 2000000u
#endif

/* Module reboot time after AT+RESET before +READY is expected. */
#ifndef RYLR998_RESET_TIMEOUT_US
#define RYLR998_RESET_TIMEOUT_US 3000000u
#endif

/* ---- module configuration (written during the init sequence) ---- */

typedef struct {
    uint16_t address;    /* module address; use the mesh node ID */
    uint8_t  network_id; /* module group, 3..15 or 18 (18 default;
                          * unrelated to the mesh's 2-byte network_id) */
    uint32_t band_hz;    /* center frequency, e.g. 915000000 */
    uint8_t  sf;         /* spreading factor 5..11; SF<=9 at 125 kHz,
                          * <=10 at 250 kHz, <=11 at 500 kHz */
    uint8_t  bw;         /* 7 = 125 kHz, 8 = 250 kHz, 9 = 500 kHz */
    uint8_t  cr;         /* coding rate 1..4 (4/5 .. 4/8) */
    uint8_t  preamble;   /* 12 unless network_id == 18 (then 4..24) */
    uint8_t  power_dbm;  /* 0..22 (<=14 for CE compliance) */

    /* Half-duplex guard: after an accepted AT+SEND the driver refuses
     * further transmits for base + per_byte * (AT payload length) so the
     * frame gets off the air before the next one (the module answers
     * +ERR=17 if you pile on). 0 = defaults sized for SF9/125 kHz from
     * the datasheet throughput table (~190 B/s). */
    uint32_t airtime_base_us;
    uint32_t airtime_per_byte_us;
} rylr998_cfg_t;

/* ---- host hooks ---- */

/* Write bytes to the module's UART (may buffer; must accept all len). */
typedef void (*rylr998_write_fn)(void *ctx, const uint8_t *data,
                                 size_t len);

/* A mesh frame was received (hex already decoded, ready for mesh_rx). */
typedef void (*rylr998_frame_fn)(void *ctx, const uint8_t *frame,
                                 size_t len, int16_t rssi_dbm,
                                 int8_t snr_db);

/* ---- diagnostics ---- */

typedef struct {
    uint32_t tx_sent;        /* AT+SEND accepted by the driver */
    uint32_t tx_busy;        /* transmit() rejected: not ready / in TX */
    uint32_t tx_err;         /* +ERR in response to AT+SEND */
    uint32_t rx_frames;      /* +RCV lines decoded and delivered */
    uint32_t rx_bad;         /* malformed +RCV lines dropped */
    uint32_t cmd_timeouts;   /* responses that never came */
    uint32_t cfg_retries;    /* config-sequence steps resent */
    uint32_t module_errs;    /* all +ERR=n responses */
    uint32_t unexpected_resets; /* +READY outside the init sequence */
} rylr998_stats_t;

/* ---- driver state (allocate statically; treat fields as private) ---- */

typedef enum {
    RYLR998_ST_RESET = 0,   /* AT+RESET sent, waiting for +READY */
    RYLR998_ST_CONFIG,      /* stepping through the config commands */
    RYLR998_ST_READY,       /* configured; normal operation */
} rylr998_state_t;

typedef struct {
    rylr998_cfg_t cfg;
    rylr998_write_fn write;
    void *write_ctx;
    rylr998_frame_fn on_frame;
    void *frame_ctx;

    rylr998_state_t state;
    uint8_t  cfg_step;       /* index into the config sequence */
    bool     awaiting;       /* a command is out, response pending */
    bool     need_send_step; /* current step not yet transmitted */
    uint64_t deadline_us;    /* response timeout for the pending cmd */
    uint64_t retry_at_us;    /* backoff before resending a config step */
    uint64_t tx_quiet_until; /* half-duplex guard after AT+SEND */
    bool     pending_is_send;

    bool     listen_want;    /* desired MODE: true = 0, false = 1 */
    bool     listen_dirty;

    uint64_t now_us;         /* latest time seen (input/poll) */

    char     line[RYLR998_LINE_MAX];
    size_t   line_len;
    bool     line_overflow;

    rylr998_stats_t stats;
} rylr998_t;

/* ---- API ---- */

/* Initialize and begin the module config sequence (soft reset, then
 * ADDRESS / NETWORKID / BAND / PARAMETER / CRFOP / MODE). Pump
 * rylr998_input() + rylr998_poll() until rylr998_ready() before
 * expecting traffic; the mesh can be initialized immediately, its sends
 * are simply refused (and retried) until the module is up. */
void rylr998_init(rylr998_t *r, const rylr998_cfg_t *cfg,
                  rylr998_write_fn write, void *write_ctx,
                  rylr998_frame_fn on_frame, void *frame_ctx,
                  uint64_t now_us);

/* Feed raw bytes read from the module's UART. Received frames are
 * delivered synchronously via on_frame. Main loop only. */
void rylr998_input(rylr998_t *r, const uint8_t *bytes, size_t n,
                   uint64_t now_us);

/* Drive timeouts and the config sequence. Returns the absolute time of
 * the next driver deadline, or UINT64_MAX if idle. Call regularly. */
uint64_t rylr998_poll(rylr998_t *r, uint64_t now_us);

/* True once the config sequence has completed. */
bool rylr998_ready(const rylr998_t *r);

const rylr998_stats_t *rylr998_stats(const rylr998_t *r);

/* ---- mesh_radio_ops_t binding (ctx = rylr998_t *) ---- */

bool rylr998_transmit(void *ctx, const uint8_t *frame, size_t len,
                      mesh_profile_t profile);
void rylr998_set_listen(void *ctx, bool enable, mesh_profile_t profile);
mesh_radio_ops_t rylr998_radio_ops(rylr998_t *r);

#ifdef __cplusplus
}
#endif

#endif /* RYLR998_H */
