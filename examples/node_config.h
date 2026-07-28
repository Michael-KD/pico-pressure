/*
 * node_config.h — compiled configuration for the example firmware
 * (field node and base station).
 *
 * Identity and the network key are normally provisioned into flash over
 * USB serial (`provision` on either firmware's CLI — M4) so one UF2
 * serves every device; the identity values below are bench-only
 * FALLBACKS used when no provisioning blob is stored. The radio/timing
 * settings further down are deployment-wide and stay compiled in.
 */
#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

/* ---- identity fallbacks (used only when unprovisioned) ---- */

/* Field-node ID, 1..254 — must be unique in the deployment. The base
 * station firmware ignores this and always runs as node 0. */
#define NODE_ID 1

/* ---- deployment (EDIT PER DEPLOYMENT, same on every device) ---- */

/* Mesh network ID (in-frame, 2 bytes) — not the radio module's. */
#define MESH_NET_ID 0xD5E7

/* Pre-shared AES-128 network key for the frame MIC (§10.2) — bench
 * fallback only. Real deployments provision a key generated on the PC
 * (`openssl rand -hex 16`) into flash instead; with a known key anyone
 * can inject ARM commands. */
#define MESH_NET_KEY                                                    \
    {                                                                   \
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,                 \
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,                 \
    }

/* Beacon period for the free-running fallback (no GPS lock at boot:
 * scheduler off, application-timed beacons, M1 style). */
#define SUPERFRAME_US (10ull * 1000000)

/* Field node: unsolicited telemetry period. */
#define TELEM_PERIOD_US (60ull * 1000000)

/* ---- GPS time source (M5) ----
 * Any NMEA receiver (u-blox NEO-6M/M8N class) on uart0; 1PPS on a GPIO
 * gives ±1 ms fleet alignment instead of ±400 ms (set -1 if unwired).
 * A device that gets no lock within GPS_WAIT_US free-runs with the
 * scheduler off — fine on the bench, flagged in the logs. */
#define GPS_UART    uart0
#define GPS_BAUD    9600
#define GPS_TX_PIN  0 /* Pico TX -> GPS RX (reserved for rx config) */
#define GPS_RX_PIN  1 /* GPS TX -> Pico RX */
#define GPS_PPS_PIN 2 /* 1PPS output; -1 = not wired */
#define GPS_WAIT_US (90ull * 1000000)

/* The fleet's superframe modes (active/standby/retrieval) live in
 * fleet_modes.h — one table shared by every device and the simulator,
 * because superframe alignment and the replay MIC both bind to it. */

/* ---- RYLR998 module settings (same on every device) ---- */

/* Radio UART wiring (Pico GP4/GP5 = uart1 TX/RX by default). */
#define RADIO_UART      uart1
#define RADIO_UART_IRQ  UART1_IRQ
#define RADIO_BAUD      115200
#define RADIO_TX_PIN    4
#define RADIO_RX_PIN    5
/* Module NRST (active low). Define as -1 if not wired; the driver's
 * soft reset (AT+RESET) is then the only reset path. */
#define RADIO_NRST_PIN  6

/* RF profile — must match on every module that should hear each other.
 * SF9 / 125 kHz / CR 4/5 / preamble 12 is the module default: a good
 * range/airtime balance for the control profile (design doc §3.3).
 * Check your regulatory region before changing band or power
 * (EU 868 MHz: duty-cycle limits and CRFOP <= 14 for CE). */
#define RADIO_BAND_HZ    915000000u
#define RADIO_SF         9   /* 5..11; <=9 at 125 kHz */
#define RADIO_BW         7   /* 7 = 125 kHz, 8 = 250 kHz, 9 = 500 kHz */
#define RADIO_CR         1   /* 4/5 */
#define RADIO_PREAMBLE   12
#define RADIO_POWER_DBM  22  /* 0..22; <=14 for CE */

/* Module-level group ID (3..15 or 18). 18 is the module default and
 * the only value that allows a configurable preamble. All deployment
 * separation is done by MESH_NET_ID + the MIC key, not by this. */
#define RADIO_MODULE_NETID 18

#endif /* NODE_CONFIG_H */
