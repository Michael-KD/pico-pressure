/*
 * mesh_sched.h — GPS-time-aligned superframe scheduler (design doc §4).
 *
 * Time is divided into a repeating superframe aligned to absolute GPS
 * time, so every node independently computes the same schedule with no
 * negotiation:
 *
 *   | beacon | command | telemetry | bulk |        sleep        |
 *   ^ superframe start                                          ^ next
 *
 * With the scheduler enabled (mesh_cfg_t.sched.enabled):
 *   - the root originates one beacon automatically at each beacon-window
 *     start — the application stops calling mesh_root_beacon();
 *   - every queued transmission is gated into its window: beacons into
 *     the beacon window, commands and command-ACKs into the command
 *     window, telemetry into the telemetry window (HOP_ACKs are
 *     immediate link responses and are never deferred). Sends requested
 *     outside their window are silently rescheduled for the next one;
 *   - own-origin telemetry is placed in an ID-ordered micro-slot within
 *     the telemetry window to keep a fleet-wide STATUS response from
 *     colliding;
 *   - the radio listens from just before the superframe start until the
 *     end of the last active window (plus a guard margin), and
 *     set_listen(false) turns it off for the sleep segment. mesh_poll()
 *     returns the next schedule edge, so the MCU can sleep too.
 *
 * All nodes must be configured with the same mode (window lengths and
 * period). Mode changes (standby <-> active) are commanded with an
 * absolute effective time so the whole fleet re-aligns at the same
 * instant: carry `at_us` in the application's SLEEP/WAKE command payload
 * and call mesh_sched_set_mode() from the command callback.
 *
 * A node that loses GPS lock keeps free-running; the application should
 * report its growing clock error via mesh_sched_clock_uncertainty(),
 * which widens the listen span symmetrically (and degrades to
 * always-listening when the uncertainty swallows the sleep segment) —
 * §4's graceful degradation.
 *
 * With sched.enabled = false everything above is inert and the library
 * behaves exactly as in M1 (application-driven beaconing, radio always
 * listening).
 */
#ifndef MESH_SCHED_H
#define MESH_SCHED_H

#include <stdbool.h>
#include <stdint.h>

#include "mesh_config.h"

#ifdef __cplusplus
extern "C" {
#endif

struct mesh; /* defined in mesh.h */

/* One operating mode: the superframe layout. Windows are laid out
 * back-to-back from the superframe start in the §4 order; the remainder
 * of the period is the sleep segment. */
typedef struct {
    uint64_t period_us;     /* whole superframe */
    uint32_t beacon_win_us;
    uint32_t cmd_win_us;
    uint32_t telem_win_us;
    uint32_t bulk_win_us;   /* 0 when no session is scheduled (M3) */
    uint32_t telem_slot_us; /* micro-slot width; 0 disables slotting */
} mesh_sched_mode_t;

/* Scheduler configuration inside mesh_cfg_t. */
typedef struct {
    bool enabled;
    mesh_sched_mode_t mode; /* initial mode; identical fleet-wide */
    uint32_t guard_us;      /* radio wakes early / sleeps late by this;
                             * 0 = MESH_SCHED_GUARD_US */
} mesh_sched_cfg_t;

/* Superframe segment at a given instant. */
typedef enum {
    MESH_SEG_BEACON = 0,
    MESH_SEG_CMD,
    MESH_SEG_TELEM,
    MESH_SEG_BULK,
    MESH_SEG_SLEEP,
} mesh_segment_t;

/* §4/§11 default modes (window lengths from the design doc). */
mesh_sched_mode_t mesh_sched_mode_active(void);  /* 60 s superframe */
mesh_sched_mode_t mesh_sched_mode_standby(void); /* 15 min superframe */

/* Switch modes at absolute time `at_us` (must be in the future). The
 * new mode's superframe 0 starts exactly at `at_us`, so a fleet given
 * the same `at_us` stays aligned. Only one switch may be pending. */
bool mesh_sched_set_mode(struct mesh *m, const mesh_sched_mode_t *mode,
                         uint64_t at_us);

/* Current segment / superframe index (counted from the current mode's
 * start). Valid only while the scheduler is enabled. */
mesh_segment_t mesh_sched_segment(const struct mesh *m, uint64_t now_us);
uint32_t mesh_sched_superframe(const struct mesh *m, uint64_t now_us);

/* Report worst-case local clock error (e.g. drift_ppm x seconds since
 * GPS lock was lost). Widens listen windows; pass 0 when lock returns. */
void mesh_sched_clock_uncertainty(struct mesh *m, uint32_t us);

/* Whether the schedule currently has the radio listening. */
bool mesh_sched_listening(const struct mesh *m);

#ifdef __cplusplus
}
#endif

#endif /* MESH_SCHED_H */
