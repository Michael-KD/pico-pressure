/*
 * node_config.h — per-device and per-deployment provisioning for the
 * example firmware (field node and base station).
 *
 * In a production flow these values come from provisioning flash; for
 * the examples, edit here and rebuild per device.
 */
#ifndef NODE_CONFIG_H
#define NODE_CONFIG_H

/* ---- identity (EDIT PER DEVICE) ---- */

/* Field-node ID, 1..254 — must be unique in the deployment. The base
 * station firmware ignores this and always runs as node 0. */
#define NODE_ID 1

/* ---- deployment (EDIT PER DEPLOYMENT, same on every device) ---- */

/* Mesh network ID (in-frame, 2 bytes) — not the radio module's. */
#define MESH_NET_ID 0xD5E7

/* Pre-shared AES-128 network key for the frame MIC (§10.2).
 * CHANGE THIS for any real deployment and keep it out of version
 * control; with a known key anyone can inject ARM commands. */
#define MESH_NET_KEY                                                    \
    {                                                                   \
        0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,                 \
        0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,                 \
    }

/* Superframe period — how often the base beacons (active mode; the
 * standby/active mode machinery is milestone M2). */
#define SUPERFRAME_US (10ull * 1000000)

/* Field node: unsolicited telemetry period. */
#define TELEM_PERIOD_US (60ull * 1000000)

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
