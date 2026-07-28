/*
 * gps_pico.h — Pico glue for the GPS time source (M5).
 *
 * Wires a standard NMEA GPS receiver (u-blox NEO-6M/M8N class) to the
 * portable gps.[ch] core: UART0 for the NMEA stream and an optional
 * GPIO for the 1PPS pulse (pins in node_config.h). The PPS ISR only
 * latches time_us_64(); everything else runs from the main loop.
 *
 * Wiring: GPS TX -> GPS_RX_PIN, GPS PPS -> GPS_PPS_PIN (optional but
 * strongly recommended — it is the difference between ±1 ms and
 * ±400 ms of fleet clock alignment), 3V3/GND. GPS_TX_PIN is reserved
 * for receiver configuration and currently unused.
 */
#ifndef GPS_PICO_H
#define GPS_PICO_H

#include "gps.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gps_t gps;
} gps_pico_t;

/* Claim the UART and PPS GPIO. One instance per firmware. */
void gps_pico_init(gps_pico_t *gp);

/* Drain UART bytes and any latched PPS edge into the core. Call from
 * the main loop at least every few ms. */
void gps_pico_pump(gps_pico_t *gp);

#ifdef __cplusplus
}
#endif

#endif /* GPS_PICO_H */
