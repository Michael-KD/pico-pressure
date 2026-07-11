/*
 * test_rylr998.c — host tests for the RYLR998 AT-protocol driver core.
 *
 * The driver is platform-free (the Pico glue only moves UART bytes), so
 * the whole state machine runs here: config sequence, +RCV parsing and
 * hex decode, busy gating, timeouts, and unexpected module resets.
 */
#include <stdlib.h>
#include <string.h>

#include "rylr998.h"
#include "test_util.h"

/* ---- fake UART: capture everything the driver writes ---- */

static char wr_buf[4096];
static size_t wr_len;

static void fake_write(void *ctx, const uint8_t *data, size_t len)
{
    (void)ctx;
    if (wr_len + len < sizeof(wr_buf)) {
        memcpy(wr_buf + wr_len, data, len);
        wr_len += len;
    }
}

static void wr_reset(void)
{
    wr_len = 0;
    memset(wr_buf, 0, sizeof(wr_buf));
}

/* Number of complete "...\r\n" commands captured. */
static unsigned wr_lines(void)
{
    unsigned n = 0;
    for (size_t i = 0; i + 1 < wr_len; i++)
        if (wr_buf[i] == '\r' && wr_buf[i + 1] == '\n')
            n++;
    return n;
}

static bool wr_contains(const char *s)
{
    wr_buf[wr_len] = '\0';
    return strstr(wr_buf, s) != NULL;
}

/* ---- frame capture ---- */

static uint8_t fr_buf[RYLR998_MAX_FRAME];
static size_t fr_len;
static int16_t fr_rssi;
static int8_t fr_snr;
static unsigned fr_count;

static void fake_on_frame(void *ctx, const uint8_t *frame, size_t len,
                          int16_t rssi_dbm, int8_t snr_db)
{
    (void)ctx;
    memcpy(fr_buf, frame, len);
    fr_len = len;
    fr_rssi = rssi_dbm;
    fr_snr = snr_db;
    fr_count++;
}

/* ---- helpers ---- */

static rylr998_t drv;
static uint64_t now;

static void feed(const char *s)
{
    rylr998_input(&drv, (const uint8_t *)s, strlen(s), now);
}

static rylr998_cfg_t default_cfg(void)
{
    rylr998_cfg_t c;
    memset(&c, 0, sizeof(c));
    c.address = 7;
    c.network_id = 18;
    c.band_hz = 915000000;
    c.sf = 9;
    c.bw = 7;
    c.cr = 1;
    c.preamble = 12;
    c.power_dbm = 14;
    return c;
}

/* +OKs needed to walk the whole config sequence. */
#define CFG_OK_COUNT 6

/* Bring a fresh driver through reset + config to READY. */
static void boot(void)
{
    rylr998_cfg_t c = default_cfg();
    now = 1000000;
    wr_reset();
    fr_count = 0;
    rylr998_init(&drv, &c, fake_write, NULL, fake_on_frame, NULL, now);
    feed("+RESET\r\n+READY\r\n");
    for (int i = 0; i < CFG_OK_COUNT; i++)
        feed("+OK\r\n");
}

static void test_config_sequence(void)
{
    rylr998_cfg_t c = default_cfg();
    now = 1000000;
    wr_reset();
    rylr998_init(&drv, &c, fake_write, NULL, fake_on_frame, NULL, now);

    T_CHECK(wr_contains("AT+RESET\r\n"));
    T_CHECK(!rylr998_ready(&drv));

    /* Module reboots and announces itself; the driver walks the config
     * sequence one command per +OK. */
    feed("+RESET\r\n+READY\r\n");
    T_CHECK(wr_contains("AT+MODE=0\r\n"));
    feed("+OK\r\n");
    T_CHECK(wr_contains("AT+ADDRESS=7\r\n"));
    feed("+OK\r\n");
    T_CHECK(wr_contains("AT+NETWORKID=18\r\n"));
    feed("+OK\r\n");
    T_CHECK(wr_contains("AT+BAND=915000000\r\n"));
    feed("+OK\r\n");
    T_CHECK(wr_contains("AT+PARAMETER=9,7,1,12\r\n"));
    feed("+OK\r\n");
    T_CHECK(!rylr998_ready(&drv));
    T_CHECK(wr_contains("AT+CRFOP=14\r\n"));
    feed("+OK\r\n");
    T_CHECK(rylr998_ready(&drv));
}

static void test_transmit_hex_encoding(void)
{
    boot();
    wr_reset();

    uint8_t frame[4] = { 0xDE, 0x5E, 0x27, 0x0A };
    T_CHECK(rylr998_transmit(&drv, frame, sizeof(frame),
                             MESH_PROFILE_CONTROL));
    /* Broadcast to module address 0; length is in AT (hex) characters. */
    T_CHECK(wr_contains("AT+SEND=0,8,DE5E270A\r\n"));
    T_CHECK_EQ(rylr998_stats(&drv)->tx_sent, 1);
}

static void test_busy_gating(void)
{
    boot();
    uint8_t frame[2] = { 0x01, 0x02 };

    /* Awaiting +OK: second transmit refused. */
    T_CHECK(rylr998_transmit(&drv, frame, 2, MESH_PROFILE_CONTROL));
    T_CHECK(!rylr998_transmit(&drv, frame, 2, MESH_PROFILE_CONTROL));
    feed("+OK\r\n");

    /* +OK received but the frame is still on the air: refused until the
     * airtime guard elapses. */
    T_CHECK(!rylr998_transmit(&drv, frame, 2, MESH_PROFILE_CONTROL));
    now += 60ull * 1000000;
    rylr998_poll(&drv, now);
    T_CHECK(rylr998_transmit(&drv, frame, 2, MESH_PROFILE_CONTROL));
    feed("+OK\r\n");

    T_CHECK_EQ(rylr998_stats(&drv)->tx_busy, 2);

    /* Oversized frames (mesh > 120 B can't hex-fit in 240) refused. */
    uint8_t big[RYLR998_MAX_FRAME + 1] = { 0 };
    now += 60ull * 1000000;
    rylr998_poll(&drv, now);
    T_CHECK(!rylr998_transmit(&drv, big, sizeof(big),
                              MESH_PROFILE_CONTROL));
}

static void test_rcv_parsing(void)
{
    boot();

    /* A 12-byte frame from module address 3, RSSI -99 dBm, SNR -7 dB. */
    feed("+RCV=3,24,E7D5010005070008DEADBEEF,-99,-7\r\n");
    T_CHECK_EQ(fr_count, 1);
    T_CHECK_EQ(fr_len, 12);
    T_CHECK_EQ(fr_buf[0], 0xE7);
    T_CHECK_EQ(fr_buf[11], 0xEF);
    T_CHECK_EQ(fr_rssi, -99);
    T_CHECK_EQ(fr_snr, -7);

    /* Datasheet examples show spaces after commas — tolerated. */
    feed("+RCV=50, 4, C0FF, -42, 11\r\n");
    T_CHECK_EQ(fr_count, 2);
    T_CHECK_EQ(fr_len, 2);
    T_CHECK_EQ(fr_buf[0], 0xC0);
    T_CHECK_EQ(fr_rssi, -42);
    T_CHECK_EQ(fr_snr, 11);

    /* Garbage is dropped, not delivered: odd length, non-hex data,
     * missing fields, truncated data. */
    unsigned before = fr_count;
    feed("+RCV=3,3,ABC,-99,-7\r\n");
    feed("+RCV=3,4,WXYZ,-99,-7\r\n");
    feed("+RCV=3,8,AABB\r\n");
    feed("+RCV=nonsense\r\n");
    T_CHECK_EQ(fr_count, before);
    T_CHECK_EQ(rylr998_stats(&drv)->rx_bad, 4);

    /* Interleaved chatter doesn't confuse the line parser. */
    feed("+RCV=9,6,ABCDEF,");
    feed("-60,9\r\n+RC");
    feed("V=9,2,55,-61,8\r\n");
    T_CHECK_EQ(fr_count, before + 2);
    T_CHECK_EQ(fr_buf[0], 0x55);
}

static void test_reset_recovery(void)
{
    boot();
    T_CHECK(rylr998_ready(&drv));

    /* Module brownout: it comes back with +READY at flash defaults.
     * The driver must notice and reconfigure. */
    wr_reset();
    feed("+READY\r\n");
    T_CHECK(!rylr998_ready(&drv));
    T_CHECK_EQ(rylr998_stats(&drv)->unexpected_resets, 1);
    T_CHECK(wr_contains("AT+MODE=0\r\n"));
    for (int i = 0; i < CFG_OK_COUNT; i++)
        feed("+OK\r\n");
    T_CHECK(rylr998_ready(&drv));
    T_CHECK(wr_contains("AT+CRFOP=14\r\n"));
}

static void test_timeouts_and_errors(void)
{
    rylr998_cfg_t c = default_cfg();
    now = 1000000;
    wr_reset();
    rylr998_init(&drv, &c, fake_write, NULL, fake_on_frame, NULL, now);

    /* No +READY ever (module already running, reset lost): after the
     * reset timeout the driver proceeds to configuration anyway. */
    now += RYLR998_RESET_TIMEOUT_US + 1;
    rylr998_poll(&drv, now);
    T_CHECK(wr_contains("AT+MODE=0\r\n"));

    /* A config step that gets +ERR is retried after a backoff. */
    unsigned lines = wr_lines();
    feed("+ERR=4\r\n");
    now += 300000; /* > backoff */
    rylr998_poll(&drv, now);
    T_CHECK_EQ(wr_lines(), lines + 1);
    T_CHECK(rylr998_stats(&drv)->cfg_retries >= 1);

    /* A config step whose +OK never arrives is resent on timeout. */
    lines = wr_lines();
    now += RYLR998_CMD_TIMEOUT_US + 1;
    rylr998_poll(&drv, now);
    T_CHECK_EQ(wr_lines(), lines + 1);
    T_CHECK(rylr998_stats(&drv)->cmd_timeouts >= 1);

    /* Finish configuration, then exercise a SEND +ERR=17. */
    for (int i = 0; i < CFG_OK_COUNT; i++)
        feed("+OK\r\n");
    T_CHECK(rylr998_ready(&drv));
    uint8_t frame[2] = { 0xAA, 0xBB };
    T_CHECK(rylr998_transmit(&drv, frame, 2, MESH_PROFILE_CONTROL));
    feed("+ERR=17\r\n");
    T_CHECK_EQ(rylr998_stats(&drv)->tx_err, 1);
}

static void test_set_listen(void)
{
    boot();
    wr_reset();

    /* Config already left the module in MODE=0, so enabling listen is a
     * no-op; disabling issues the sleep command on the next poll. */
    rylr998_set_listen(&drv, true, MESH_PROFILE_CONTROL);
    rylr998_poll(&drv, now);
    T_CHECK_EQ(wr_lines(), 0);

    rylr998_set_listen(&drv, false, MESH_PROFILE_CONTROL);
    rylr998_poll(&drv, now);
    T_CHECK(wr_contains("AT+MODE=1\r\n"));
    feed("+OK\r\n");

    rylr998_set_listen(&drv, true, MESH_PROFILE_CONTROL);
    rylr998_poll(&drv, now);
    T_CHECK(wr_contains("AT+MODE=0\r\n"));
    feed("+OK\r\n");
}

static void test_roundtrip_two_drivers(void)
{
    /* Frame out of driver A, over a "cable" that turns the AT+SEND into
     * the +RCV the receiving module would emit, into driver B. */
    boot();
    wr_reset();

    uint8_t frame[13];
    for (unsigned i = 0; i < sizeof(frame); i++)
        frame[i] = (uint8_t)(i * 37 + 1);
    T_CHECK(rylr998_transmit(&drv, frame, sizeof(frame),
                             MESH_PROFILE_CONTROL));

    /* Extract the hex payload from the captured AT+SEND=0,<n>,<hex>. */
    wr_buf[wr_len] = '\0';
    const char *hex = strrchr(wr_buf, ',');
    T_CHECK(hex != NULL);
    hex++;
    char rcv[RYLR998_LINE_MAX];
    snprintf(rcv, sizeof(rcv), "+RCV=7,%u,%.*s,-80,10\r\n",
             (unsigned)(sizeof(frame) * 2), (int)(sizeof(frame) * 2),
             hex);

    static rylr998_t drv_b;
    rylr998_cfg_t c = default_cfg();
    c.address = 8;
    rylr998_init(&drv_b, &c, fake_write, NULL, fake_on_frame, NULL, now);
    fr_count = 0;
    rylr998_input(&drv_b, (const uint8_t *)rcv, strlen(rcv), now);

    T_CHECK_EQ(fr_count, 1);
    T_CHECK_EQ(fr_len, sizeof(frame));
    T_CHECK(memcmp(fr_buf, frame, sizeof(frame)) == 0);
    T_CHECK_EQ(fr_rssi, -80);
    T_CHECK_EQ(fr_snr, 10);
}

int main(void)
{
    T_RUN(test_config_sequence);
    T_RUN(test_transmit_hex_encoding);
    T_RUN(test_busy_gating);
    T_RUN(test_rcv_parsing);
    T_RUN(test_reset_recovery);
    T_RUN(test_timeouts_and_errors);
    T_RUN(test_set_listen);
    T_RUN(test_roundtrip_two_drivers);
    return t_summary();
}
