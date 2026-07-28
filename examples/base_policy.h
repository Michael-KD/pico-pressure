/*
 * base_policy.h — base-station retrieval scheduler + fleet checklist
 * (M6, design doc §7.4 / principle 6: "the base is the only smart
 * place").
 *
 * The library moves chunks; this engine owns the policy of a retrieval
 * campaign:
 *
 *   1. Switch the fleet into retrieval mode (§4) at an absolute time —
 *      flooded as a fleet mode command with ACK-checklist re-floods —
 *      so a bulk window exists at all. (Skipped on scheduler-off bench
 *      fleets, where bulk frames fire immediately.)
 *   2. Walk the target nodes one at a time (§7.4: one session per tree
 *      path avoids self-interference on shared relays). Per node:
 *      LIST (bulk session 0) → parse the recording index → FETCH the
 *      event bytes → verify the end-to-end CRC16 against the index.
 *   3. Drive completion: track the received-chunk bitmap, re-FETCH
 *      from the first gap on stall, give up on a node after a retry
 *      budget and move on (its result says why).
 *   4. Switch the fleet back and report per-node results.
 *
 * Platform-free: works against any mesh_t root, so the same engine
 * runs in the base-station firmware CLI and in the host simulator's
 * end-to-end campaign scenario. The app forwards on_bulk_data /
 * on_command_ack into it and calls bp_poll() from its loop.
 */
#ifndef BASE_POLICY_H
#define BASE_POLICY_H

#include "fleet_modes.h"
#include "mesh/mesh.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BP_MAX_NODES 32
#define BP_MAX_CHUNKS 64 /* bitmap width: 64 x 96 B = 6 kB per event */

/* Retry budgets. */
#define BP_MODE_REFLOODS 3
#define BP_FETCH_RETRIES 6

/* Session-space convention (an example-fleet convention, like command
 * opcodes): 0 = the CLI's LIST; 1..127 = manual CLI fetches;
 * 128..239 = campaign FETCH sessions; 240..254 = campaign LIST
 * sessions, a fresh id per node so a late chunk from the previous
 * node's index can never contaminate the current one. Field nodes
 * serve the recording index for 0 and for 240+. */
#define BP_LIST_SESSION0 240

typedef enum {
    BP_IDLE = 0,
    BP_MODE_IN,  /* switching the fleet into retrieval mode */
    BP_LIST,     /* fetching the current node's recording index */
    BP_FETCH,    /* fetching the current node's event bytes */
    BP_MODE_OUT, /* switching the fleet back */
} bp_state_t;

typedef struct {
    bool     attempted;
    bool     listed;   /* index line received and parsed */
    bool     ok;       /* transfer complete and CRC verified */
    uint32_t bytes;    /* event size from the index */
    uint16_t crc_want; /* CRC16 from the index */
    uint16_t crc_got;  /* CRC16 over the reassembled bytes */
    uint8_t  attempts; /* FETCH issues (1 + re-FETCHes) */
} bp_result_t;

typedef struct {
    mesh_t  *m;
    uint8_t *buf;     /* caller-owned reassembly buffer */
    uint32_t buf_max;

    bp_state_t state;
    bool     switch_modes;
    uint8_t  restore_mode; /* fleet mode id to return to */

    uint8_t  nodes[BP_MAX_NODES];
    unsigned n_nodes;
    unsigned cur; /* index into nodes[] */
    bp_result_t res[BP_MAX_NODES];

    /* mode-switch phase */
    uint16_t mode_cmd_id;
    uint64_t mode_at;      /* effective time; 0 = not yet launched */
    uint8_t  mode_id;      /* mode being switched to */
    uint8_t  refloods_left;
    uint32_t acked_mask;   /* bit i: nodes[i] ACKed the mode command */
    uint64_t next_flood_us;

    /* transfer state (LIST and FETCH share the machinery) */
    uint8_t  session;
    uint8_t  next_session;  /* FETCH allocator, 128..239 */
    uint8_t  list_counter;  /* LIST allocator offset above 240 */
    uint8_t  chunk;
    uint32_t total;
    unsigned nchunks;
    uint64_t got_mask;
    unsigned got;
    uint64_t last_progress_us;
    char     list_text[96];
    bool     list_eof; /* short chunk seen: index text is complete */
} base_policy_t;

void bp_init(base_policy_t *p, mesh_t *m, uint8_t *buf,
             uint32_t buf_max);

/* Start a campaign over `nodes` (retrieved in the given order — pass
 * them nearest-first). `switch_modes` floods the fleet into retrieval
 * mode first and back to `restore_mode` after; pass false on
 * scheduler-off bench fleets. False if already busy or no nodes. */
bool bp_start(base_policy_t *p, const uint8_t *nodes, unsigned n_nodes,
              bool switch_modes, uint8_t restore_mode, uint64_t now_us);

/* Forward from the app's callbacks. */
void bp_on_bulk_data(base_policy_t *p, uint8_t session,
                     uint16_t chunk_index, const uint8_t *data,
                     uint8_t len, uint64_t now_us);
void bp_on_command_ack(base_policy_t *p, uint8_t origin,
                       uint16_t cmd_id);

/* Drive the state machine; call from the main loop (after mesh_poll). */
void bp_poll(base_policy_t *p, uint64_t now_us);

static inline bool bp_busy(const base_policy_t *p)
{
    return p->state != BP_IDLE;
}

/* Human-readable progress/result summary into `out`. */
void bp_status(const base_policy_t *p, char *out, unsigned max);

#ifdef __cplusplus
}
#endif

#endif /* BASE_POLICY_H */
