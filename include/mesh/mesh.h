/*
 * mesh.h — public API of the blast-sensor RF mesh library.
 *
 * Radio-agnostic core mesh implementing the design in
 * Blast_Sensor_Mesh_Design.md (Rev B): collection-tree routing rooted at
 * the base station, ETX link metric from beacon reception, controlled
 * flooding for commands, per-hop-reliable tree unicast for ACKs,
 * telemetry, and bulk data chunks (FETCH sessions, §7), an optional
 * GPS-aligned superframe scheduler with duty-cycled listening
 * (mesh_sched.h), and replay-protected AES-CMAC frame authentication
 * (superframe-bound MIC, §10.2).
 *
 * Usage model (single-threaded, externally clocked, no allocation):
 *   - The application owns a mesh_t (statically allocated) and calls
 *     mesh_init() once with its node ID, radio ops, and callbacks.
 *   - The radio driver delivers received frames via mesh_rx().
 *   - The application calls mesh_poll(now) regularly (and after any
 *     mesh_* send call); all transmissions happen inside mesh_poll().
 *     Its return value is the time of the next pending event, so the
 *     MCU may sleep until then.
 *   - "now" is absolute time in microseconds. On hardware this is
 *     GPS-disciplined time; in tests it is a simulated clock. The
 *     library never reads a clock itself.
 *
 * Roles: node ID 0 is the root (base station gateway) — it originates
 * beacons and commands and receives ACKs/telemetry. IDs 1..254 are field
 * nodes. The same library serves both.
 */
#ifndef MESH_H
#define MESH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "mesh_config.h"
#include "mesh_mic.h"
#include "mesh_radio.h"
#include "mesh_sched.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- Identity ---- */

#define MESH_ID_ROOT      0x00u
#define MESH_ID_BROADCAST 0xFFu
/* Sentinel for "no node" (e.g., no parent). Numerically the broadcast
 * address, which can never be a real node or parent. */
#define MESH_ID_NONE      0xFFu

/* Returned by mesh_poll() when nothing is scheduled. */
#define MESH_NO_EVENT UINT64_MAX

/* ---- Frame types (§8) ---- */

typedef enum {
    MESH_FT_BEACON   = 0x01, /* root-originated, flooded; builds the tree */
    MESH_FT_CMD      = 0x02, /* flooded command */
    MESH_FT_CMD_ACK  = 0x03, /* execution ACK, tree unicast to root */
    MESH_FT_TELEM    = 0x04, /* status report, tree unicast to root */
    MESH_FT_HOP_ACK  = 0x05, /* single-hop link ACK for tree unicast */
    MESH_FT_DATA     = 0x06, /* bulk chunk, tree unicast to root (§7) */
    MESH_FT_DATA_ACK = 0x07, /* reserved: store-and-forward bursts */
    MESH_FT_SESSION  = 0x08, /* FETCH request, flooded to target (§7.3) */
} mesh_frame_type_t;

/* ---- Diagnostics: neighbor table entry (read-only to the app) ---- */

/* ETX windows are indexed by OBSERVED epochs — epochs in which this node
 * heard at least one beacon from anyone — rather than by raw beacon
 * sequence delta. Epochs in which the entire beacon wave failed to reach
 * this node punish no one; epochs in which some neighbors delivered and
 * one did not punish the silent one. This de-conflates upstream flood
 * attenuation from genuine link loss. */
typedef struct {
    bool     in_use;
    bool     ever_beacon;     /* heard at least one beacon from them */
    uint8_t  id;
    uint8_t  hops;            /* their advertised hop count from root */
    int16_t  rssi_q4;         /* smoothed RSSI, dBm x16 */
    int16_t  snr_q4;          /* smoothed SNR, dB x16 */
    uint16_t adv_cost;        /* their advertised path cost, Q8.8 */
    uint16_t etx_q8;          /* smoothed link ETX, Q8.8 (0 = unknown) */
    uint16_t beacon_bitmap;   /* reception history, bit0 = newest */
    uint16_t last_beacon_seq;
    uint16_t obs_at;          /* observed-epoch stamp of last beacon */
    uint16_t last_heard_seq;  /* net epoch when any frame was last heard */
    uint16_t epochs_known;    /* capped at MESH_ETX_WINDOW */
} mesh_neighbor_t;

/* ---- Diagnostics: event counters ---- */

typedef struct {
    uint32_t rx_frames;             /* frames accepted by the codec */
    uint32_t rx_drop_len;
    uint32_t rx_drop_net;           /* foreign network ID */
    uint32_t rx_drop_mic;
    uint32_t tx_frames;
    uint32_t tx_radio_fail;
    uint32_t txq_full;              /* sends dropped: no queue slot */
    uint32_t beacons_rx;
    uint32_t beacons_rebroadcast;
    uint32_t cmds_seen;             /* unique flood seqs observed */
    uint32_t cmds_executed;
    uint32_t floods_relayed;
    uint32_t acks_sent;             /* CMD_ACKs originated */
    uint32_t telem_sent;
    uint32_t up_relayed;            /* unicast frames forwarded to parent */
    uint32_t up_dropped_no_parent;
    uint32_t up_dedup_drops;
    uint32_t hop_acks_sent;
    uint32_t hop_retries;
    uint32_t hop_exhausted;
    uint32_t parent_changes;
    uint32_t neighbors_evicted;
    uint32_t tx_deferred;           /* sends rescheduled into their window */
    uint32_t tx_win_dropped;        /* dropped: window absent in this mode */
    uint32_t bulk_fetches_rx;       /* node: FETCH sessions accepted */
    uint32_t bulk_chunks_sent;      /* node: own DATA chunks originated */
    uint32_t bulk_chunks_root;      /* root: chunks delivered to the app */
    uint32_t bulk_crc_drops;        /* root: chunks failing source CRC16 */
} mesh_stats_t;

/* ---- Telemetry as delivered to the root callback ----
 * Each report carries the sender's routing context (§5.3: parent plus
 * top-3 neighbors) so the base can build the global topology map. */

#define MESH_TELEM_NBRS 3

typedef struct {
    uint8_t  origin;                      /* reporting node */
    uint8_t  parent;                      /* MESH_ID_NONE if unattached */
    uint8_t  nbr_id[MESH_TELEM_NBRS];     /* MESH_ID_NONE = empty slot */
    uint16_t nbr_etx_q8[MESH_TELEM_NBRS];
} mesh_telem_info_t;

/* Bytes of routing context prepended to the app telemetry payload. */
#define MESH_TELEM_HDR (2 + MESH_TELEM_NBRS * 3)

/* Library-level payload layout inside CMD frames: 2-byte command ID then
 * the opaque application payload. */
#define MESH_CMD_HDR 2
#define MESH_MAX_CMD_PAYLOAD (MESH_MAX_PAYLOAD - MESH_CMD_HDR)
#define MESH_MAX_TELEM_PAYLOAD (MESH_MAX_PAYLOAD - MESH_TELEM_HDR)

/* Bulk payload layouts (§7.3): a FETCH request rides a SESSION frame as
 * session(1) base_off(4 LE) req_off(4 LE) req_len(4 LE) chunk_size(1);
 * a chunk rides a DATA frame as session(1) chunk_index(2 LE)
 * crc16(2 LE) followed by the chunk bytes. */
#define MESH_FETCH_PLEN 14
#define MESH_BULK_DATA_HDR 5
#define MESH_MAX_BULK_DATA (MESH_MAX_PAYLOAD - MESH_BULK_DATA_HDR)

/* ---- Application callbacks ----
 * All callbacks run synchronously inside mesh_rx()/mesh_poll(). */

typedef struct {
    void *user;

    /* A command addressed to this node (or broadcast). Return true to
     * accept: the library records the command ID as executed and sends a
     * CMD_ACK up the tree. Return false to ignore (no ACK; the base will
     * re-flood). Command semantics — ARM validity windows, the node state
     * machine — are the application's responsibility.
     * Called on field nodes only. */
    bool (*on_command)(void *user, uint16_t cmd_id,
                       const uint8_t *payload, uint8_t len);

    /* Root only: an execution ACK arrived. May be called more than once
     * for the same (origin, cmd_id) if a node re-ACKs after a re-flood;
     * treat it as idempotent checklist marking. */
    void (*on_command_ack)(void *user, uint8_t origin, uint16_t cmd_id,
                           uint8_t status);

    /* Root only: a telemetry report arrived. */
    void (*on_telemetry)(void *user, const mesh_telem_info_t *info,
                         const uint8_t *payload, uint8_t len);

    /* Field node: serve bulk data for a FETCH session (§7). Copy up to
     * `max` bytes at absolute byte offset `offset` into `buf` and
     * return the count. Returning less than `max` marks the end of the
     * data: the short (or absent) chunk closes the session. Session
     * numbering is an application convention — the examples reserve
     * session 0 for the recording index (LIST). NULL = this node serves
     * no bulk data; FETCHes are ignored. */
    uint8_t (*on_bulk_read)(void *user, uint8_t session, uint32_t offset,
                            uint8_t *buf, uint8_t max);

    /* Root only: a CRC-verified bulk chunk arrived. chunk_index is
     * relative to the session's base_offset (chunk k covers bytes
     * [base_offset + k*chunk_size ...)). Completion tracking and
     * re-FETCH of missing indices are the base application's job. */
    void (*on_bulk_data)(void *user, uint8_t session,
                         uint16_t chunk_index, const uint8_t *data,
                         uint8_t len);

    /* Parent changed (attach, switch, or loss — loss reports new parent
     * MESH_ID_NONE). Useful for LEDs/logging; no action required. */
    void (*on_parent_change)(void *user, uint8_t old_parent,
                             uint8_t new_parent);
} mesh_callbacks_t;

/* ---- Configuration passed to mesh_init ---- */

typedef struct {
    uint8_t          node_id;    /* 0 = root, 1..254 = field node */
    uint16_t         network_id; /* deployment identifier (§3.2) */
    mesh_radio_ops_t radio;
    mesh_mic_ops_t   mic;        /* compute==NULL -> mesh_mic_null */
    mesh_callbacks_t cb;         /* individual callbacks may be NULL */
    uint32_t         rng_seed;   /* nonzero; diversify per node */
    mesh_sched_cfg_t sched;      /* superframe scheduler (§4); zeroed =
                                  * disabled (M1 behavior) */
} mesh_cfg_t;

/* ---- Internal state (do not touch fields directly) ----
 * Exposed only so the application can allocate mesh_t statically. */

typedef struct {
    bool     in_use;
    uint8_t  kind;               /* mesh_txk_t */
    uint8_t  len;
    uint8_t  attempts_left;      /* sends remaining (incl. next one) */
    bool     await_hop_ack;
    uint8_t  ack_from;
    uint8_t  ack_type;
    uint16_t ack_seq;
    uint16_t beacon_seq;         /* for build-at-fire beacon entries */
    uint32_t retry_interval_us;
    uint64_t due_us;
    uint8_t  buf[MESH_MAX_FRAME];
} mesh_txq_entry_t;

typedef struct {
    uint8_t  src;
    uint8_t  type;
    uint16_t seq;
} mesh_up_dedup_t;

/* Bulk-source session state (§7.3): one FETCH session per node, one
 * chunk in flight at a time. Replaced wholesale by each new FETCH. */
typedef struct {
    bool     active;
    bool     pending;      /* a chunk is in the TX queue awaiting its hop */
    uint8_t  session;
    uint8_t  chunk_size;
    uint16_t pending_seq;  /* txq ack_seq of the chunk in flight */
    uint32_t base_off;     /* chunk-index origin */
    uint32_t cursor;       /* next byte offset to send */
    uint32_t end_off;      /* one past the last requested byte */
} mesh_bulk_t;

typedef struct mesh {
    /* configuration */
    uint8_t          node_id;
    uint16_t         network_id;
    bool             is_root;
    mesh_radio_ops_t radio;
    mesh_mic_ops_t   mic;
    mesh_callbacks_t cb;
    uint32_t         rng;

    /* tree state */
    uint8_t  parent;         /* MESH_ID_NONE when unattached */
    uint16_t path_cost;      /* Q8.8; 0 at root, MESH_COST_INF unattached */
    uint8_t  hops;           /* 0 at root, 0xFF unattached */
    uint8_t  cand_id;        /* hysteresis challenger */
    uint8_t  cand_epochs;
    uint16_t last_hyst_epoch;
    bool     have_hyst_epoch;
    uint8_t  parent_fail;    /* consecutive exhausted sends to parent */

    /* beacon epoch tracking */
    uint16_t net_epoch;      /* highest beacon seq seen */
    uint16_t obs_epochs;     /* count of epochs this node observed */
    bool     have_epoch;
    uint16_t last_rebc_seq;  /* newest beacon seq already scheduled */
    bool     have_rebc;

    /* superframe scheduler (§4) */
    mesh_sched_cfg_t  sched;          /* cfg copy; .mode = current mode */
    uint64_t          mode_start_us;  /* alignment origin of current mode */
    mesh_sched_mode_t pending_mode;
    uint64_t          pending_mode_at; /* MESH_NO_EVENT = none */
    uint32_t          clock_unc_us;
    bool              listening;      /* last state given to set_listen */
    uint32_t          last_beacon_sf; /* root: superframe already beaconed */
    bool              have_beacon_sf;

    /* sequence counters */
    uint16_t beacon_seq;     /* root: beacon originator */
    uint16_t flood_seq;      /* root: per-flood-transmission counter */
    uint16_t next_cmd_id;    /* root: command ID allocator */
    uint16_t up_seq;         /* per-hop unicast counter */

    /* dedup rings */
    uint16_t flood_seen[MESH_FLOOD_DEDUP];
    uint8_t  flood_seen_idx, flood_seen_cnt;
    uint16_t exec_seen[MESH_EXEC_DEDUP];
    uint8_t  exec_seen_idx, exec_seen_cnt;
    mesh_up_dedup_t up_seen[MESH_UP_DEDUP];
    uint8_t  up_seen_idx, up_seen_cnt;

    /* tables */
    mesh_neighbor_t  nbrs[MESH_MAX_NEIGHBORS];
    mesh_txq_entry_t txq[MESH_TXQ_DEPTH];

    /* bulk-source session (§7.3) */
    mesh_bulk_t bulk;

    mesh_stats_t stats;
    bool initialized;
} mesh_t;

/* ---- API ---- */

/* Initialize. Returns false on bad config (node ID 255, missing radio
 * transmit op, zero RNG seed). */
bool mesh_init(mesh_t *m, const mesh_cfg_t *cfg);

/* Deliver a received frame from the radio driver. rssi_dbm/snr_db are the
 * radio's link measurements for this frame. Call from the main loop. */
void mesh_rx(mesh_t *m, const uint8_t *data, size_t len,
             int16_t rssi_dbm, int8_t snr_db, uint64_t now_us);

/* Run scheduled work (transmissions, retries, timeouts). Returns the
 * absolute time of the next pending event, or MESH_NO_EVENT. Call
 * regularly and after any mesh_rx / send call. */
uint64_t mesh_poll(mesh_t *m, uint64_t now_us);

/* Root only: originate one beacon. With the scheduler disabled the
 * application calls this once per superframe; with it enabled the
 * library beacons automatically at each beacon-window start and the
 * application must not call this. Returns false if not root or queue
 * full. */
bool mesh_root_beacon(mesh_t *m, uint64_t now_us);

/* Root only: flood a command to `target` (a node ID, or
 * MESH_ID_BROADCAST for the whole fleet). `cmd_id` 0 auto-allocates a
 * fresh ID; to re-flood for missing ACKs (§6 step 4), pass the same
 * cmd_id again — nodes that already executed will re-ACK without
 * re-executing. Returns the command ID used, or 0 on error. */
uint16_t mesh_root_command(mesh_t *m, uint8_t target,
                           const uint8_t *payload, uint8_t len,
                           uint16_t cmd_id, uint64_t now_us);

/* Field node: send a telemetry report (app payload, <=
 * MESH_MAX_TELEM_PAYLOAD bytes) up the tree. The library prepends the
 * routing context (parent, top-3 neighbors). Returns false if unattached
 * or queue full. */
bool mesh_node_telemetry(mesh_t *m, const uint8_t *payload, uint8_t len,
                         uint64_t now_us);

/* Root only: flood a FETCH (§7.3) asking `target` to stream
 * `req_length` bytes starting at `req_offset` up the tree in
 * `chunk_size`-byte DATA chunks (arriving at on_bulk_data).
 * `base_offset` is the chunk-index origin: keep it constant across
 * re-FETCHes of missing sub-ranges of the same object so chunk indices
 * stay aligned, and keep req_offset - base_offset a multiple of
 * chunk_size. Tracking received indices and re-FETCHing gaps is the
 * base application's job (design principle 6). Returns false on invalid
 * arguments, full queue, or a scheduler mode with no bulk window. */
bool mesh_root_fetch(mesh_t *m, uint8_t target, uint8_t session,
                     uint32_t base_offset, uint32_t req_offset,
                     uint32_t req_length, uint8_t chunk_size,
                     uint64_t now_us);

/* ---- Route persistence (§5.3) ----
 * The application owns the storage (flash on the Pico). Save produces a
 * self-checking blob; load must be called immediately after mesh_init,
 * before any received traffic.
 *
 * Field node: saves parent + path cost + neighbor ETX estimates + the
 * beacon-epoch position, so a rebooted node resumes its last route
 * instantly and repairs it lazily from subsequent beacons — the §5.4
 * staleness rule still poisons a route that aged out during the outage.
 * Save on parent change (rate-limited; flash wear is the app's budget).
 *
 * Root: saves the beacon/flood/command counters; load restores them
 * advanced by MESH_PERSIST_SEQ_MARGIN so a rebooted base's sequence
 * numbers stay strictly ahead of everything the fleet has seen (a base
 * restarting at seq 0 would look *older* than the fleet's held state
 * and be ignored). Save periodically, well inside the margin. */

/* Worst-case blob size (node role with a full neighbor table); the
 * fixed part is header + scheduler mode/alignment + route fields. */
#define MESH_ROUTE_BLOB_MAX (8 + 36 + 8 + MESH_MAX_NEIGHBORS * 8 + 2)

/* Serialize into buf (>= MESH_ROUTE_BLOB_MAX); returns bytes written,
 * 0 on error. */
size_t mesh_route_save(const mesh_t *m, uint8_t *buf, size_t max);

/* Restore from a saved blob. Returns false (leaving the mesh unchanged)
 * on CRC/version mismatch, identity mismatch (node re-provisioned),
 * role mismatch, or a mesh that has already heard traffic. */
bool mesh_route_load(mesh_t *m, const uint8_t *buf, size_t len);

/* ---- Introspection ---- */

uint8_t  mesh_parent(const mesh_t *m);     /* MESH_ID_NONE if unattached */
uint16_t mesh_path_cost(const mesh_t *m);  /* Q8.8 */
uint8_t  mesh_hops(const mesh_t *m);
/* CRC-16/CCITT-FALSE — the per-chunk integrity check (§7.3), exposed
 * so applications can run end-to-end verification over reassembled
 * data. The _step form (init 0xFFFF, fold one byte) lets a data source
 * checksum a stream without buffering it. */
uint16_t mesh_crc16(const uint8_t *data, size_t len);
uint16_t mesh_crc16_step(uint16_t crc, uint8_t byte);
const mesh_stats_t    *mesh_stats_get(const mesh_t *m);
/* Neighbor table slot idx (0..MESH_MAX_NEIGHBORS-1); NULL if unused. */
const mesh_neighbor_t *mesh_neighbor_at(const mesh_t *m, unsigned idx);

#ifdef __cplusplus
}
#endif

#endif /* MESH_H */
