/*
 * test_gps.c — NMEA parsing and clock discipline (examples/gps.c).
 *
 * The GPS core turns receiver bytes + PPS edges into the fleet-shared
 * absolute timebase; a wrong conversion here silently misaligns every
 * superframe in the field, so the calendar math gets known-answer
 * checks and the discipline paths get exact-offset checks.
 */
#include <stdio.h>
#include <string.h>

#include "gps.h"
#include "test_util.h"

/* 2026-07-27T12:00:00Z in Unix µs (independently derived). */
#define T0 (1785153600ull * 1000000ull)

/* Feed one sentence (body without '$'/checksum), all bytes stamped
 * `t_end`, with a correct checksum appended. */
static void feed(gps_t *g, const char *body, uint64_t t_end)
{
    uint8_t sum = 0;
    for (const char *p = body; *p; p++)
        sum ^= (uint8_t)*p;
    char full[128];
    snprintf(full, sizeof(full), "$%s*%02X\r\n", body, sum);
    for (const char *p = full; *p; p++)
        gps_input(g, *p, t_end);
}

#define RMC_NOON "GPRMC,120000.00,A,4807.038,N,01131.000,E,0.0,0.0," \
                 "270726,003.1,W"

static void test_calendar(void)
{
    T_CHECK_EQ(gps_utc_to_us(1970, 1, 1, 0, 0, 0, 0), 0);
    T_CHECK_EQ(gps_utc_to_us(2026, 7, 27, 12, 0, 0, 0), T0);
    /* Leap day (2024-02-29). */
    T_CHECK_EQ(gps_utc_to_us(2024, 2, 29, 0, 0, 0, 0),
               1709164800ull * 1000000ull);
    /* Fraction carried through. */
    T_CHECK_EQ(gps_utc_to_us(2026, 7, 27, 12, 0, 0, 250000) - T0, 250000);
}

static void test_pps_discipline(void)
{
    gps_t g;
    gps_init(&g);
    T_CHECK(!gps_locked(&g, 0));
    T_CHECK_EQ(gps_uncertainty_us(&g, 0), UINT32_MAX);

    /* PPS edge at local 10 s marks 12:00:00; the sentence labels it
     * 300 ms later. */
    gps_on_pps(&g, 10000000);
    feed(&g, RMC_NOON, 10300000);

    T_CHECK(gps_locked(&g, 10300000));
    T_CHECK_EQ(g.disciplines, 1);
    /* Absolute time is exact: edge = T0, so local 10.3 s = T0+300ms. */
    T_CHECK_EQ(gps_now(&g, 10300000), T0 + 300000);
    T_CHECK_EQ(gps_now(&g, 10000000), T0);
    /* PPS-grade uncertainty: 1 ms base + sub-ms drift. */
    T_CHECK(gps_uncertainty_us(&g, 10300000) < 2000);

    /* Next second: consistent edge -> same offset, no step counted. */
    gps_on_pps(&g, 11000000);
    feed(&g, "GPRMC,120001.00,A,4807.038,N,01131.000,E,0.0,0.0,"
             "270726,003.1,W", 11250000);
    T_CHECK_EQ(g.disciplines, 2);
    T_CHECK_EQ(g.steps, 0);
    T_CHECK_EQ(gps_now(&g, 11000000), T0 + 1000000);

    /* Fractional RMC time still labels the whole-second edge. */
    gps_on_pps(&g, 12000000);
    feed(&g, "GPRMC,120002.25,A,4807.038,N,01131.000,E,0.0,0.0,"
             "270726,003.1,W", 12200000);
    T_CHECK_EQ(gps_now(&g, 12000000), T0 + 2000000);

    /* A stale edge (no RMC within the window) must not mislabel: an
     * RMC arriving 2 s after the last edge disciplines nothing. */
    uint32_t disc = g.disciplines;
    gps_on_pps(&g, 13000000);
    feed(&g, "GPRMC,120004.00,A,4807.038,N,01131.000,E,0.0,0.0,"
             "270726,003.1,W", 15200000);
    T_CHECK_EQ(g.disciplines, disc);
}

static void test_no_pps_discipline(void)
{
    gps_t g;
    gps_init(&g);
    feed(&g, RMC_NOON, 5000000);
    T_CHECK(gps_locked(&g, 5000000));
    T_CHECK_EQ(gps_now(&g, 5000000), T0);
    /* Sentence-timing grade uncertainty. */
    uint32_t u = gps_uncertainty_us(&g, 5000000);
    T_CHECK(u >= GPS_UNC_NOPPS_US && u < GPS_UNC_NOPPS_US + 1000);
}

static void test_rejects(void)
{
    gps_t g;
    gps_init(&g);

    /* Corrupted checksum: counted, never disciplined. */
    const char *bad = "$GPRMC,120000.00,A,4807.038,N,01131.000,E,0.0,"
                      "0.0,270726,003.1,W*00\r\n";
    for (const char *p = bad; *p; p++)
        gps_input(&g, *p, 1000000);
    T_CHECK_EQ(g.bad_nmea, 1);
    T_CHECK(!g.disciplined);

    /* Status V (no fix): time may be an RTC guess — never trusted. */
    feed(&g, "GPRMC,120000.00,V,,,,,,,270726,,", 2000000);
    T_CHECK(!g.disciplined);
    T_CHECK(!g.have_fix);

    /* Non-RMC sentences are checksum-checked and ignored. */
    feed(&g, "GPGGA,120000.00,4807.038,N,01131.000,E,1,08,0.9,545.4,M,"
             "46.9,M,,", 3000000);
    T_CHECK(!g.disciplined);
    T_CHECK_EQ(g.bad_nmea, 1);

    /* Garbage bytes then a valid sentence: parser resyncs on '$'. */
    const char *junk = "\xFF" "garbage\r\n$GP\xFF" "broken";
    for (const char *p = junk; *p; p++)
        gps_input(&g, *p, 3500000);
    feed(&g, RMC_NOON, 4000000);
    T_CHECK(g.disciplined);
    T_CHECK_EQ(gps_now(&g, 4000000), T0);
}

static void test_lock_decay_and_freerun(void)
{
    gps_t g;
    gps_init(&g);
    gps_on_pps(&g, 1000000);
    feed(&g, RMC_NOON, 1200000);
    T_CHECK(gps_locked(&g, 1500000));

    /* Receiver goes quiet (antenna fault): lock indicator decays but
     * the timebase free-runs on the last offset with growing
     * uncertainty — §4's graceful degradation. */
    uint64_t later = 1000000 + 600ull * 1000000; /* +10 min */
    T_CHECK(!gps_locked(&g, later));
    T_CHECK_EQ(gps_now(&g, later), T0 + 600ull * 1000000);
    uint32_t u = gps_uncertainty_us(&g, later);
    /* 600 s at 50 ppm = 30 ms of drift allowance. */
    T_CHECK(u >= 30000 && u < 40000);
}

int main(void)
{
    T_RUN(test_calendar);
    T_RUN(test_pps_discipline);
    T_RUN(test_no_pps_discipline);
    T_RUN(test_rejects);
    T_RUN(test_lock_decay_and_freerun);
    return t_summary();
}
