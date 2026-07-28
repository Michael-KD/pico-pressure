/*
 * gps.h — GPS time source: NMEA parsing + clock discipline (M5).
 *
 * Turns a GPS receiver's NMEA stream (plus, ideally, its 1PPS pulse)
 * into the fleet-shared absolute timebase the mesh scheduler needs
 * (design doc §4): absolute microseconds since the Unix epoch, the same
 * on every node, derived from each node's own receiver.
 *
 * Discipline model: the receiver's PPS edge marks the exact top of a
 * UTC second; the RMC sentence that follows (~100–500 ms later) labels
 * which second it was. Each labeled edge yields the offset between the
 * local free-running clock (`local_us`, e.g. time_us_64()) and absolute
 * time; between edges, absolute time is local + offset, with an
 * uncertainty that grows at the crystal's drift rate — which is
 * exactly what mesh_sched_clock_uncertainty() wants (§4 graceful
 * degradation). Without PPS wired, sentence arrival itself labels the
 * time at a few hundred ms of uncertainty: the scheduler still works,
 * it just listens with wider guards.
 *
 * This file is platform-free (bytes and timestamps in, time out) and
 * host-tested; gps_pico.[ch] is the UART/PPS-IRQ glue.
 *
 * The absolute time this module reports can step backward a little
 * when a discipline update lands (crystal drift correction). The mesh
 * needs monotonic time, so the firmware wraps gps_now() in a
 * max(previous+1, ...) clamp — see app_now() in the examples.
 */
#ifndef GPS_H
#define GPS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GPS_LINE_MAX 84 /* NMEA 0183 max sentence length incl. CRLF */

/* Local crystal drift bound while free-running between fixes, in ppm.
 * The Pico's crystal is ±30 ppm; 50 leaves margin for temperature. */
#ifndef GPS_DRIFT_PPM
#define GPS_DRIFT_PPM 50
#endif

/* Base uncertainty of one discipline point: PPS edges are exact to
 * interrupt latency; sentence-arrival timing wears the receiver's
 * output latency. */
#define GPS_UNC_PPS_US   1000u
#define GPS_UNC_NOPPS_US 400000u

/* An RMC sentence labels the most recent PPS edge only if it follows
 * within this window. */
#define GPS_PPS_MAX_AGE_US 1500000u

/* Fix indicator decays if no valid RMC arrives for this long. */
#define GPS_FIX_TIMEOUT_US 5000000u

typedef struct {
    /* sentence assembly */
    char     line[GPS_LINE_MAX];
    uint8_t  len;
    bool     in_line;
    /* PPS */
    uint64_t pps_local; /* newest edge; 0 = none */
    bool     pps_used;  /* edge already consumed by a discipline point */
    bool     pps_ever;
    /* fix + discipline */
    bool     have_fix;
    bool     disciplined;
    int64_t  offset_us;      /* absolute = local + offset */
    uint64_t disc_local;     /* local time of last discipline point */
    uint64_t last_rmc_local; /* local time of last valid RMC */
    /* diagnostics */
    uint32_t sentences;   /* checksum-valid sentences */
    uint32_t bad_nmea;    /* framing/checksum rejects */
    uint32_t disciplines; /* offset updates applied */
    uint32_t steps;       /* updates that moved the clock > 5 ms */
} gps_t;

void gps_init(gps_t *g);

/* A PPS rising edge occurred at local time `local_us` (latched in the
 * ISR, delivered from the main loop). */
void gps_on_pps(gps_t *g, uint64_t local_us);

/* Feed one received NMEA byte; `local_us` timestamps sentence
 * completion. */
void gps_input(gps_t *g, char c, uint64_t local_us);

/* Disciplined and holding a recent valid fix. */
bool gps_locked(const gps_t *g, uint64_t local_us);

/* Absolute Unix time in µs. Valid once disciplined (even during a fix
 * outage — it free-runs on the offset); 0 before first discipline. */
uint64_t gps_now(const gps_t *g, uint64_t local_us);

/* Worst-case clock error vs. true GPS time, for
 * mesh_sched_clock_uncertainty(). UINT32_MAX before first discipline. */
uint32_t gps_uncertainty_us(const gps_t *g, uint64_t local_us);

/* Exposed for tests: UTC calendar date/time -> Unix µs. */
uint64_t gps_utc_to_us(unsigned year, unsigned month, unsigned day,
                       unsigned hh, unsigned mm, unsigned ss,
                       uint32_t frac_us);

#ifdef __cplusplus
}
#endif

#endif /* GPS_H */
