/*
 * gps_pico.c — see gps_pico.h.
 */
#include "gps_pico.h"

#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "node_config.h"

/* Single instance: the PPS ISR needs a target without a context arg. */
static gps_pico_t *inst;
static volatile uint64_t pps_edge; /* 0 = none pending */

#if GPS_PPS_PIN >= 0
static void pps_isr(uint gpio, uint32_t events)
{
    (void)events;
    if (gpio == (uint)GPS_PPS_PIN)
        pps_edge = time_us_64();
}
#endif

void gps_pico_init(gps_pico_t *gp)
{
    inst = gp;
    gps_init(&gp->gps);

    uart_init(GPS_UART, GPS_BAUD);
    gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);
    gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
    uart_set_format(GPS_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(GPS_UART, true);

#if GPS_PPS_PIN >= 0
    gpio_init(GPS_PPS_PIN);
    gpio_set_dir(GPS_PPS_PIN, GPIO_IN);
    gpio_pull_down(GPS_PPS_PIN);
    gpio_set_irq_enabled_with_callback(GPS_PPS_PIN, GPIO_IRQ_EDGE_RISE,
                                       true, pps_isr);
#endif
}

void gps_pico_pump(gps_pico_t *gp)
{
    if (gp != inst)
        return;

    /* Drain queued sentence bytes BEFORE delivering a latched edge: a
     * sentence that completed before the edge in real time must not be
     * processed after it, or it would pair with the wrong second (the
     * core's minimum-gap rule catches the residue). 9600 baud ≈ one
     * char per ms; the 32-deep FIFO tolerates the loop's pacing. */
    while (uart_is_readable(GPS_UART))
        gps_input(&gp->gps, (char)uart_getc(GPS_UART), time_us_64());

    uint64_t edge = pps_edge;
    if (edge != 0) {
        pps_edge = 0;
        gps_on_pps(&gp->gps, edge);
    }
}
