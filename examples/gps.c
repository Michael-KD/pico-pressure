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
    g->pps_prev = g->pps_local;
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

static void apply_offset(gps_t *g, int64_t new_offset, uint64_t local_us,
                         bool by_pps)
{
    g->offset_us = new_offset;
    g->disciplined = true;
    g->disc_by_pps = by_pps;
    g->disc_local = local_us;
    g->disciplines++;
}

/* PPS-grade discipline with delayed-edge rejection: an edge whose ISR
 * ran late (interrupts held off by a flash write) produces one offset
 * jump that the next healthy second contradicts, while a genuine clock
 * step (receiver re-fix) repeats identically — so a suspect jump is
 * parked until a second point agrees with it. */
static void pps_offset(gps_t *g, int64_t new_offset, uint64_t local_us)
{
    if (g->disciplined && g->disc_by_pps) {
        int64_t d = new_offset - g->offset_us;
        if (d > GPS_STEP_SUSPECT_US || d < -GPS_STEP_SUSPECT_US) {
            int64_t cd = new_offset - g->cand_offset;
            g->steps++;
            if (g->have_cand &&
                cd <= GPS_STEP_CONFIRM_US && cd >= -GPS_STEP_CONFIRM_US) {
                g->have_cand = false; /* confirmed: a real step */
                apply_offset(g, new_offset, local_us, true);
            } else {
                g->cand_offset = new_offset;
                g->have_cand = true;
            }
            return;
        }
        g->have_cand = false;
    }
    apply_offset(g, new_offset, local_us, true);
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

    uint64_t whole = rmc_us - frac; /* top of the labeled second */
    if (g->pps_ever && g->pps_local != 0) {
        uint64_t gap = local_us - g->pps_local;
        if (!g->pps_used && gap >= GPS_PPS_MIN_GAP_US &&
            gap <= GPS_PPS_LATE_US) {
            /* Normal case: the newest edge marked the top of the second
             * this sentence labels. */
            g->pps_used = true;
            pps_offset(g, (int64_t)whole - (int64_t)g->pps_local,
                       local_us);
        } else if (!g->pps_used && gap < GPS_PPS_MIN_GAP_US &&
                   g->pps_prev != 0 &&
                   local_us - g->pps_prev <=
                       GPS_PPS_LATE_US + 1000000u) {
            /* The sentence squeaked in just after the NEXT second's
             * edge (long NMEA burst at 9600 baud, or pump ordering):
             * physically it cannot describe an edge <50 ms old, so the
             * newest edge is second+1 — which labels it just as well. */
            g->pps_used = true;
            pps_offset(g, (int64_t)(whole + 1000000u) -
                              (int64_t)g->pps_local,
                       local_us);
        } else if (gap > GPS_PPS_DEAD_US) {
            /* PPS died mid-deployment (wiring): degrade to sentence
             * timing rather than freezing the clock while the crystal
             * drifts. Uncertainty reporting downgrades with it. */
            apply_offset(g, (int64_t)rmc_us - (int64_t)local_us,
                         local_us, false);
        }
        /* else: ambiguous pairing — skip; next second disciplines. */
    } else {
        /* No PPS wired: sentence completion labels the reported time,
         * to within the receiver's output latency (covered by the
         * larger GPS_UNC_NOPPS_US). */
        apply_offset(g, (int64_t)rmc_us - (int64_t)local_us, local_us,
                     false);
    }
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
    /* Callers may hold a timestamp sampled just before the pump that
     * produced the newest discipline point; clamp instead of letting
     * the subtraction wrap to ~2^64 (which read as "always listen"). */
    uint64_t elapsed = (local_us > g->disc_local)
                           ? local_us - g->disc_local
                           : 0;
    uint64_t drift = elapsed * GPS_DRIFT_PPM / 1000000u;
    /* Grade by how the CURRENT offset was obtained, not by whether a
     * PPS edge was ever seen — a sentence-timed offset is ±400 ms even
     * on a PPS-wired node whose first edge hasn't paired yet. */
    uint64_t base = g->disc_by_pps ? GPS_UNC_PPS_US : GPS_UNC_NOPPS_US;
    uint64_t total = base + drift;
    return (total > UINT32_MAX) ? UINT32_MAX : (uint32_t)total;
}
