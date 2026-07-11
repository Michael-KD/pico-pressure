/*
 * rylr998_pico.h — pico-sdk glue for the portable RYLR998 driver.
 *
 * Owns the radio UART (pins/IRQ from node_config.h): the UART RX
 * interrupt only pushes bytes into a ring buffer; the main loop drains
 * it into the driver with rylr998_pico_pump(). One radio per board.
 */
#ifndef RYLR998_PICO_H
#define RYLR998_PICO_H

#include "rylr998.h"

/* Set up the UART + NRST pin, hardware-reset the module, and start the
 * driver's config sequence. on_frame delivers received mesh frames
 * (call mesh_rx from it — this runs in main-loop context). */
void rylr998_pico_init(rylr998_t *drv, const rylr998_cfg_t *cfg,
                       rylr998_frame_fn on_frame, void *frame_ctx,
                       uint64_t now_us);

/* Drain UART bytes into the driver. Call every main-loop pass, before
 * rylr998_poll()/mesh_poll(). */
void rylr998_pico_pump(rylr998_t *drv, uint64_t now_us);

#endif /* RYLR998_PICO_H */
