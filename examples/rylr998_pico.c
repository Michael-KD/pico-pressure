/* rylr998_pico.c — see rylr998_pico.h. */
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"

#include "node_config.h"
#include "rylr998_pico.h"

/* ---- RX ring buffer, filled by the UART interrupt ---- */

#define RX_RING_SIZE 512u /* power of two */

static uint8_t rx_ring[RX_RING_SIZE];
static volatile uint32_t rx_head; /* ISR-owned */
static uint32_t rx_tail;          /* main-loop-owned */

static void uart_rx_isr(void)
{
    while (uart_is_readable(RADIO_UART)) {
        uint8_t b = (uint8_t)uart_getc(RADIO_UART);
        uint32_t head = rx_head;
        uint32_t next = (head + 1) & (RX_RING_SIZE - 1);
        if (next != (rx_tail & (RX_RING_SIZE - 1))) {
            rx_ring[head & (RX_RING_SIZE - 1)] = b;
            rx_head = next;
        }
        /* Ring full: drop the byte. The AT line parser resynchronizes
         * on the next newline. */
    }
}

/* ---- driver hooks ---- */

static void radio_uart_write(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    /* Blocks only for UART FIFO space (~23 ms worst case for a full
     * AT+SEND at 115200) — within the radio-op contract of "hand the
     * frame over, don't wait for air time". */
    uart_write_blocking(RADIO_UART, data, len);
}

/* ---- public API ---- */

void rylr998_pico_init(rylr998_t *drv, const rylr998_cfg_t *cfg,
                       rylr998_frame_fn on_frame, void *frame_ctx,
                       uint64_t now_us)
{
    uart_init(RADIO_UART, RADIO_BAUD);
    gpio_set_function(RADIO_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(RADIO_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(RADIO_UART, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(RADIO_UART, true);

    irq_set_exclusive_handler(RADIO_UART_IRQ, uart_rx_isr);
    irq_set_enabled(RADIO_UART_IRQ, true);
    uart_set_irq_enables(RADIO_UART, true /* rx */, false /* tx */);

#if RADIO_NRST_PIN >= 0
    /* Hardware reset: NRST low for >=100 ms (datasheet). */
    gpio_init(RADIO_NRST_PIN);
    gpio_set_dir(RADIO_NRST_PIN, GPIO_OUT);
    gpio_put(RADIO_NRST_PIN, 0);
    sleep_ms(120);
    gpio_put(RADIO_NRST_PIN, 1);
#endif

    /* The driver also issues AT+RESET; whichever +READY arrives first
     * kicks off the config sequence, and timeouts cover the rest. */
    rylr998_init(drv, cfg, radio_uart_write, NULL, on_frame, frame_ctx,
                 now_us);
}

void rylr998_pico_pump(rylr998_t *drv, uint64_t now_us)
{
    uint8_t chunk[64];
    for (;;) {
        uint32_t head = rx_head; /* snapshot; ISR may advance it */
        size_t n = 0;
        while (rx_tail != head && n < sizeof(chunk)) {
            chunk[n++] = rx_ring[rx_tail & (RX_RING_SIZE - 1)];
            rx_tail = (rx_tail + 1) & (RX_RING_SIZE - 1);
        }
        if (n == 0)
            break;
        rylr998_input(drv, chunk, n, now_us);
    }
}
