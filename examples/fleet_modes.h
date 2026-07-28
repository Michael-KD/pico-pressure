/*
 * fleet_modes.h — the fleet's shared mode table + mode-switch command
 * convention (M6, design doc §4/§7.4).
 *
 * Mode changes are commands (§4): the base floods opcode 'M' with a
 * mode ID and an absolute effective time, and every node — including
 * the base itself — calls mesh_sched_set_mode() with the *same* mode
 * for the *same* instant, so the fleet re-aligns together. The table
 * must therefore be identical everywhere; it lives in this header,
 * included by the field node, the base station, the base policy
 * engine, and the simulator.
 *
 * Command payload: 'M', mode_id (1 B), effective GPS time (u64 LE).
 *
 * Header-only (static inline) so the simulator can share it without
 * linking example objects.
 */
#ifndef FLEET_MODES_H
#define FLEET_MODES_H

#include "mesh/mesh.h"

#define FLEET_CMD_MODE 'M'
#define FLEET_MODE_PLEN 10 /* opcode + id + at_us */

#define FLEET_MODE_ACTIVE    0u /* default: active + small bulk window */
#define FLEET_MODE_STANDBY   1u /* 15 min superframe, low power */
#define FLEET_MODE_RETRIEVAL 2u /* §7.4: half the superframe is bulk */

/* Look up a fleet mode; false for an unknown ID (reject the command —
 * a fleet split across firmware revisions must not half-switch). */
static inline bool fleet_mode_by_id(uint8_t id, mesh_sched_mode_t *out)
{
    switch (id) {
    case FLEET_MODE_ACTIVE:
        /* The library's active layout, stretched with a modest bulk
         * window and a sleep tail (see node_config.h SCHED_*): ad-hoc
         * `fetch`es work without a mode switch, and the sleep segment
         * gives the route cache its flash-write slot. */
        *out = mesh_sched_mode_active();
        out->period_us = 90ull * 1000000;
        out->bulk_win_us = 20u * 1000000;
        return true;
    case FLEET_MODE_STANDBY:
        *out = mesh_sched_mode_standby();
        return true;
    case FLEET_MODE_RETRIEVAL:
        *out = mesh_sched_mode_retrieval();
        return true;
    default:
        return false;
    }
}

/* Build the 10-byte mode-switch payload. */
static inline void fleet_mode_cmd(uint8_t buf[FLEET_MODE_PLEN],
                                  uint8_t mode_id, uint64_t at_us)
{
    buf[0] = FLEET_CMD_MODE;
    buf[1] = mode_id;
    for (int i = 0; i < 8; i++)
        buf[2 + i] = (uint8_t)(at_us >> (8 * i));
}

/* Parse it (payload after the opcode check, len == FLEET_MODE_PLEN). */
static inline bool fleet_mode_parse(const uint8_t *pl, uint8_t len,
                                    uint8_t *mode_id, uint64_t *at_us)
{
    if (len != FLEET_MODE_PLEN || pl[0] != FLEET_CMD_MODE)
        return false;
    *mode_id = pl[1];
    uint64_t at = 0;
    for (int i = 7; i >= 0; i--)
        at = (at << 8) | pl[2 + i];
    *at_us = at;
    return true;
}

#endif /* FLEET_MODES_H */
