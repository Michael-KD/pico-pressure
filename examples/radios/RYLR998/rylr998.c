/* rylr998.c — see rylr998.h for the design notes. */
#include <stdio.h>
#include <string.h>

#include "rylr998.h"

#define CFG_STEPS 6
#define CFG_RETRY_BACKOFF_US 200000u

/* Defaults for the half-duplex TX guard, sized for SF9 / 125 kHz from
 * the datasheet throughput table (~190 B/s) plus margin for preamble
 * and module processing. */
#define AIRTIME_BASE_DEFAULT_US 150000u
#define AIRTIME_PER_BYTE_DEFAULT_US 5300u

/* The module adds 8 bytes of its own framing to every packet on the
 * air (AT command guide, "THE SEQUENCE OF USING AT COMMAND" note 5). */
#define MODULE_AIR_OVERHEAD 8u

/* ---- tiny text helpers ---- */

static int hexval(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    return -1;
}

static void skip_spaces(const char **pp)
{
    while (**pp == ' ')
        (*pp)++;
}

static bool parse_uint(const char **pp, uint32_t *out)
{
    const char *p = *pp;
    uint32_t v = 0;
    bool any = false;
    skip_spaces(&p);
    while (*p >= '0' && *p <= '9') {
        v = v * 10 + (uint32_t)(*p - '0');
        p++;
        any = true;
    }
    *pp = p;
    *out = v;
    return any;
}

static bool parse_int(const char **pp, int32_t *out)
{
    const char *p = *pp;
    bool neg = false;
    uint32_t mag;
    skip_spaces(&p);
    if (*p == '-') {
        neg = true;
        p++;
    }
    if (!parse_uint(&p, &mag))
        return false;
    *pp = p;
    *out = neg ? -(int32_t)mag : (int32_t)mag;
    return true;
}

static bool expect_comma(const char **pp)
{
    skip_spaces(pp);
    if (**pp != ',')
        return false;
    (*pp)++;
    return true;
}

/* ---- command transmission ---- */

static void send_cmd(rylr998_t *r, const char *cmd, uint32_t timeout_us)
{
    r->write(r->write_ctx, (const uint8_t *)cmd, strlen(cmd));
    r->write(r->write_ctx, (const uint8_t *)"\r\n", 2);
    r->awaiting = true;
    r->deadline_us = r->now_us + timeout_us;
}

static void send_config_step(rylr998_t *r)
{
    char buf[64];
    const rylr998_cfg_t *c = &r->cfg;

    switch (r->cfg_step) {
    case 0:
        snprintf(buf, sizeof(buf), "AT+MODE=0");
        break;
    case 1:
        snprintf(buf, sizeof(buf), "AT+ADDRESS=%u", (unsigned)c->address);
        break;
    case 2:
        snprintf(buf, sizeof(buf), "AT+NETWORKID=%u",
                 (unsigned)c->network_id);
        break;
    case 3:
        snprintf(buf, sizeof(buf), "AT+BAND=%lu",
                 (unsigned long)c->band_hz);
        break;
    case 4:
        snprintf(buf, sizeof(buf), "AT+PARAMETER=%u,%u,%u,%u",
                 (unsigned)c->sf, (unsigned)c->bw, (unsigned)c->cr,
                 (unsigned)c->preamble);
        break;
    default:
        snprintf(buf, sizeof(buf), "AT+CRFOP=%u", (unsigned)c->power_dbm);
        break;
    }
    r->need_send_step = false;
    r->pending_is_send = false;
    send_cmd(r, buf, RYLR998_CMD_TIMEOUT_US);
}

static void begin_config(rylr998_t *r)
{
    r->state = RYLR998_ST_CONFIG;
    r->cfg_step = 0;
    r->need_send_step = true;
    r->awaiting = false;
    send_config_step(r);
}

/* ---- response handling ---- */

static void handle_ok(rylr998_t *r)
{
    r->awaiting = false;
    if (r->state == RYLR998_ST_CONFIG) {
        r->cfg_step++;
        if (r->cfg_step >= CFG_STEPS)
            r->state = RYLR998_ST_READY;
        else
            send_config_step(r);
    }
}

static void handle_err(rylr998_t *r)
{
    r->stats.module_errs++;
    r->awaiting = false;
    if (r->state == RYLR998_ST_CONFIG) {
        /* Resend the failing step after a short backoff. */
        r->stats.cfg_retries++;
        r->need_send_step = true;
        r->retry_at_us = r->now_us + CFG_RETRY_BACKOFF_US;
    } else if (r->pending_is_send) {
        /* e.g. +ERR=17: previous TX still on the air. The mesh's
         * retransmit ladder recovers the frame. */
        r->stats.tx_err++;
    }
}

static void handle_ready(rylr998_t *r)
{
    if (r->state != RYLR998_ST_RESET) {
        /* Spontaneous reboot (brownout, watchdog): the module is back
         * at its flash defaults, so walk the config sequence again. */
        r->stats.unexpected_resets++;
    }
    begin_config(r);
}

static void handle_rcv(rylr998_t *r, const char *p)
{
    uint32_t addr, dlen;
    int32_t rssi, snr;
    uint8_t frame[RYLR998_MAX_FRAME];

    if (!parse_uint(&p, &addr) || !expect_comma(&p) ||
        !parse_uint(&p, &dlen) || !expect_comma(&p))
        goto bad;
    skip_spaces(&p);

    /* <Data>: exactly dlen hex characters (we only ever transmit hex,
     * so anything else is noise or a foreign system). */
    if (dlen == 0 || dlen > 240 || (dlen & 1) != 0 ||
        strlen(p) < dlen)
        goto bad;
    for (uint32_t i = 0; i < dlen / 2; i++) {
        int hi = hexval(p[2 * i]);
        int lo = hexval(p[2 * i + 1]);
        if (hi < 0 || lo < 0)
            goto bad;
        frame[i] = (uint8_t)((hi << 4) | lo);
    }
    p += dlen;

    if (!expect_comma(&p) || !parse_int(&p, &rssi) ||
        !expect_comma(&p) || !parse_int(&p, &snr))
        goto bad;

    if (rssi < INT16_MIN)
        rssi = INT16_MIN;
    if (snr > INT8_MAX)
        snr = INT8_MAX;
    if (snr < INT8_MIN)
        snr = INT8_MIN;

    r->stats.rx_frames++;
    if (r->on_frame)
        r->on_frame(r->frame_ctx, frame, dlen / 2, (int16_t)rssi,
                    (int8_t)snr);
    return;
bad:
    r->stats.rx_bad++;
}

static void handle_line(rylr998_t *r, const char *line)
{
    if (line[0] == '\0')
        return;
    if (strncmp(line, "+RCV=", 5) == 0)
        handle_rcv(r, line + 5);
    else if (strcmp(line, "+OK") == 0)
        handle_ok(r);
    else if (strncmp(line, "+ERR=", 5) == 0)
        handle_err(r);
    else if (strcmp(line, "+READY") == 0)
        handle_ready(r);
    /* +RESET, query echoes (+MODE=0, ...), and anything unrecognized
     * are informational — ignore. */
}

/* ---- public API ---- */

void rylr998_init(rylr998_t *r, const rylr998_cfg_t *cfg,
                  rylr998_write_fn write, void *write_ctx,
                  rylr998_frame_fn on_frame, void *frame_ctx,
                  uint64_t now_us)
{
    memset(r, 0, sizeof(*r));
    r->cfg = *cfg;
    if (r->cfg.airtime_base_us == 0)
        r->cfg.airtime_base_us = AIRTIME_BASE_DEFAULT_US;
    if (r->cfg.airtime_per_byte_us == 0)
        r->cfg.airtime_per_byte_us = AIRTIME_PER_BYTE_DEFAULT_US;
    r->write = write;
    r->write_ctx = write_ctx;
    r->on_frame = on_frame;
    r->frame_ctx = frame_ctx;
    r->now_us = now_us;
    r->listen_want = true;

    /* Soft reset so the config sequence starts from a known state. If
     * the module misses it, the +READY timeout falls through to the
     * config sequence anyway. */
    r->state = RYLR998_ST_RESET;
    send_cmd(r, "AT+RESET", RYLR998_RESET_TIMEOUT_US);
}

void rylr998_input(rylr998_t *r, const uint8_t *bytes, size_t n,
                   uint64_t now_us)
{
    r->now_us = now_us;
    for (size_t i = 0; i < n; i++) {
        char c = (char)bytes[i];
        if (c == '\n') {
            if (r->line_overflow) {
                r->stats.rx_bad++;
            } else {
                r->line[r->line_len] = '\0';
                handle_line(r, r->line);
            }
            r->line_len = 0;
            r->line_overflow = false;
        } else if (c != '\r') {
            if (r->line_len < sizeof(r->line) - 1)
                r->line[r->line_len++] = c;
            else
                r->line_overflow = true;
        }
    }
}

uint64_t rylr998_poll(rylr998_t *r, uint64_t now_us)
{
    r->now_us = now_us;

    if (r->awaiting && now_us >= r->deadline_us) {
        r->awaiting = false;
        r->stats.cmd_timeouts++;
        if (r->state == RYLR998_ST_RESET) {
            /* No +READY — the module may already be running (e.g. warm
             * MCU reboot without a module power cycle). Configure it. */
            begin_config(r);
        } else if (r->state == RYLR998_ST_CONFIG) {
            r->stats.cfg_retries++;
            r->need_send_step = true;
            r->retry_at_us = now_us;
        }
        /* READY + send timeout: nothing more to do; the mesh's per-hop
         * retries own frame-level recovery. */
    }

    if (!r->awaiting) {
        if (r->state == RYLR998_ST_CONFIG && r->need_send_step &&
            now_us >= r->retry_at_us) {
            send_config_step(r);
        } else if (r->state == RYLR998_ST_READY && r->listen_dirty) {
            r->listen_dirty = false;
            r->pending_is_send = false;
            send_cmd(r, r->listen_want ? "AT+MODE=0" : "AT+MODE=1",
                     RYLR998_CMD_TIMEOUT_US);
        }
    }

    if (r->awaiting)
        return r->deadline_us;
    if (r->state == RYLR998_ST_CONFIG && r->need_send_step)
        return r->retry_at_us;
    return UINT64_MAX;
}

bool rylr998_ready(const rylr998_t *r)
{
    return r->state == RYLR998_ST_READY;
}

const rylr998_stats_t *rylr998_stats(const rylr998_t *r)
{
    return &r->stats;
}

/* ---- mesh_radio_ops_t binding ---- */

bool rylr998_transmit(void *ctx, const uint8_t *frame, size_t len,
                      mesh_profile_t profile)
{
    (void)profile; /* single-channel module; CONTROL only until M3 */
    rylr998_t *r = (rylr998_t *)ctx;
    static const char hexdig[] = "0123456789ABCDEF";
    char buf[RYLR998_LINE_MAX];

    if (r->state != RYLR998_ST_READY || r->awaiting ||
        r->now_us < r->tx_quiet_until ||
        len == 0 || len > RYLR998_MAX_FRAME) {
        r->stats.tx_busy++;
        return false;
    }

    unsigned alen = (unsigned)(len * 2);
    int n = snprintf(buf, sizeof(buf), "AT+SEND=0,%u,", alen);
    for (size_t i = 0; i < len; i++) {
        buf[n + 2 * (int)i] = hexdig[frame[i] >> 4];
        buf[n + 2 * (int)i + 1] = hexdig[frame[i] & 0x0F];
    }
    buf[n + (int)alen] = '\0';

    r->pending_is_send = true;
    send_cmd(r, buf, RYLR998_CMD_TIMEOUT_US);

    /* Half-duplex guard: refuse further sends until this frame has had
     * time to clear the air. */
    r->tx_quiet_until = r->now_us + r->cfg.airtime_base_us +
                        (uint64_t)r->cfg.airtime_per_byte_us *
                            (alen + MODULE_AIR_OVERHEAD);
    r->stats.tx_sent++;
    return true;
}

void rylr998_set_listen(void *ctx, bool enable, mesh_profile_t profile)
{
    (void)profile;
    rylr998_t *r = (rylr998_t *)ctx;
    if (r->listen_want != enable) {
        r->listen_want = enable;
        r->listen_dirty = true; /* rylr998_poll() issues the AT+MODE */
    }
}

mesh_radio_ops_t rylr998_radio_ops(rylr998_t *r)
{
    mesh_radio_ops_t ops = {
        .ctx = r,
        .transmit = rylr998_transmit,
        .set_listen = rylr998_set_listen,
    };
    return ops;
}
