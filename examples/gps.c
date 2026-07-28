/*
 * gps.c — NMEA parsing and clock discipline (see gps.h).
 *
 * Only RMC is parsed (any talker: GPRMC, GNRMC, ...): it is the one
 * sentence that carries status + time + date together, which is all a
 * timebase needs. Everything else is checksum-validated and ignored.
 */
#include "gps.h"

#include <string.h>

void gps_init(gps_t *g)
{
    memset(g, 0, sizeof(*g));
}

void gps_on_pps(gps_t *g, uint64_t local_us)
{
    g->pps_local = local_us;
    g->pps_used = false;
    g->pps_ever = true;
}

/* Howard Hinnant's civil-days algorithm: days since 1970-01-01. */
static int64_t days_from_civil(int64_t y, unsigned m, unsigned d)
{
    y -= m <= 2;
    int64_t era = (y >= 0 ? y : y - 399) / 400;
    unsigned yoe = (unsigned)(y - era * 400);
    unsigned doy = (153u * (m + (m > 2 ? (unsigned)-3 : 9u)) + 2u) / 5u +
                   d - 1u;
    unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    return era * 146097 + (int64_t)doe - 719468;
}

uint64_t gps_utc_to_us(unsigned year, unsigned month, unsigned day,
                       unsigned hh, unsigned mm, unsigned ss,
                       uint32_t frac_us)
{
    int64_t days = days_from_civil((int64_t)year, month, day);
    uint64_t secs = (uint64_t)days * 86400u + hh * 3600u + mm * 60u + ss;
    return secs * 1000000u + frac_us;
}

static bool two_digits(const char *p, unsigned *out)
{
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9')
        return false;
    *out = (unsigned)((p[0] - '0') * 10 + (p[1] - '0'));
    return true;
}

/* Split the sentence body (after "$", before "*") into comma fields.
 * Returns the field count; empty fields are valid (NMEA is positional). */
static unsigned split_fields(char *body, const char *fields[],
                             unsigned max)
{
    unsigned n = 0;
    char *p = body;
    while (n < max) {
        fields[n++] = p;
        char *c = strchr(p, ',');
        if (!c)
            break;
        *c = '\0';
        p = c + 1;
    }
    return n;
}

/* Parse "hhmmss[.sss]" and "ddmmyy" into Unix µs. */
static bool parse_rmc_time(const char *tim, const char *date,
                           uint64_t *out_us, uint32_t *out_frac)
{
    unsigned hh, mm, ss, dd, mo, yy;
    if (strlen(tim) < 6 || strlen(date) != 6)
        return false;
    if (!two_digits(tim, &hh) || !two_digits(tim + 2, &mm) ||
        !two_digits(tim + 4, &ss))
        return false;
    if (!two_digits(date, &dd) || !two_digits(date + 2, &mo) ||
        !two_digits(date + 4, &yy))
        return false;
    if (hh > 23 || mm > 59 || ss > 60 || dd < 1 || dd > 31 || mo < 1 ||
        mo > 12)
        return false;

    uint32_t frac = 0;
    if (tim[6] == '.') {
        uint32_t scale = 100000;
        for (const char *p = tim + 7; *p >= '0' && *p <= '9' && scale;
             p++, scale /= 10)
            frac += (uint32_t)(*p - '0') * scale;
    }
    *out_frac = frac;
    *out_us = gps_utc_to_us(2000u + yy, mo, dd, hh, mm, ss, frac);
    return true;
}

static void apply_offset(gps_t *g, int64_t new_offset, uint64_t local_us)
{
    if (g->disciplined) {
        int64_t delta = new_offset - g->offset_us;
        if (delta > 5000 || delta < -5000)
            g->steps++;
    }
    g->offset_us = new_offset;
    g->disciplined = true;
    g->disc_local = local_us;
    g->disciplines++;
}

static void process_rmc(gps_t *g, char *body, uint64_t local_us)
{
    /* body: "GPRMC,hhmmss.ss,A,lat,N,lon,W,spd,cog,ddmmyy,..." */
    const char *f[12];
    unsigned n = split_fields(body, f, 12);
    if (n < 10)
        return;

    bool valid = f[2][0] == 'A';
    if (!valid) {
        /* Status V: the receiver's time may be an RTC guess — track fix
         * loss but never discipline from it. */
        g->have_fix = false;
        return;
    }

    uint64_t rmc_us;
    uint32_t frac;
    if (!parse_rmc_time(f[1], f[9], &rmc_us, &frac))
        return;

    g->have_fix = true;
    g->last_rmc_local = local_us;

    if (g->pps_ever && !g->pps_used && g->pps_local != 0 &&
        local_us - g->pps_local < GPS_PPS_MAX_AGE_US) {
        /* The edge marked the top of the second this sentence labels. */
        g->pps_used = true;
        apply_offset(g, (int64_t)(rmc_us - frac) - (int64_t)g->pps_local,
                     local_us);
    } else if (!g->pps_ever) {
        /* No PPS wired: sentence completion labels the reported time,
         * to within the receiver's output latency (covered by the
         * larger GPS_UNC_NOPPS_US). */
        apply_offset(g, (int64_t)rmc_us - (int64_t)local_us, local_us);
    }
    /* PPS wired but this sentence missed its edge: skip — the next
     * labeled edge disciplines. */
}

static void process_line(gps_t *g, uint64_t local_us)
{
    /* line: sentence between '$' and CR/LF, e.g. "GPRMC,...*68" */
    char *star = strrchr(g->line, '*');
    if (!star || star - g->line < 1 || strlen(star) < 3) {
        g->bad_nmea++;
        return;
    }
    uint8_t want = 0;
    for (int i = 1; i <= 2; i++) {
        char c = star[i];
        uint8_t v;
        if (c >= '0' && c <= '9')
            v = (uint8_t)(c - '0');
        else if (c >= 'A' && c <= 'F')
            v = (uint8_t)(c - 'A' + 10);
        else if (c >= 'a' && c <= 'f')
            v = (uint8_t)(c - 'a' + 10);
        else {
            g->bad_nmea++;
            return;
        }
        want = (uint8_t)((want << 4) | v);
    }
    uint8_t sum = 0;
    for (const char *p = g->line; p < star; p++)
        sum ^= (uint8_t)*p;
    if (sum != want) {
        g->bad_nmea++;
        return;
    }
    g->sentences++;

    *star = '\0'; /* body ends at the checksum */
    if (strlen(g->line) > 6 && memcmp(g->line + 2, "RMC,", 4) == 0)
        process_rmc(g, g->line, local_us);
}

void gps_input(gps_t *g, char c, uint64_t local_us)
{
    if (c == '$') {
        g->in_line = true;
        g->len = 0;
        return;
    }
    if (!g->in_line)
        return;
    if (c == '\r' || c == '\n') {
        g->in_line = false;
        if (g->len > 0 && g->len < GPS_LINE_MAX) {
            g->line[g->len] = '\0';
            process_line(g, local_us);
        }
        return;
    }
    if (g->len >= GPS_LINE_MAX - 1) {
        g->in_line = false; /* oversized garbage */
        g->bad_nmea++;
        return;
    }
    g->line[g->len++] = c;
}

bool gps_locked(const gps_t *g, uint64_t local_us)
{
    return g->disciplined && g->have_fix &&
           local_us - g->last_rmc_local < GPS_FIX_TIMEOUT_US;
}

uint64_t gps_now(const gps_t *g, uint64_t local_us)
{
    if (!g->disciplined)
        return 0;
    return (uint64_t)((int64_t)local_us + g->offset_us);
}

uint32_t gps_uncertainty_us(const gps_t *g, uint64_t local_us)
{
    if (!g->disciplined)
        return UINT32_MAX;
    uint64_t elapsed = local_us - g->disc_local;
    uint64_t drift = elapsed * GPS_DRIFT_PPM / 1000000u;
    uint64_t base = g->pps_ever ? GPS_UNC_PPS_US : GPS_UNC_NOPPS_US;
    uint64_t total = base + drift;
    return (total > UINT32_MAX) ? UINT32_MAX : (uint32_t)total;
}
