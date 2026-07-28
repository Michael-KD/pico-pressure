/*
 * mesh_config.h — compile-time tunables for the blast-sensor mesh library.
 *
 * Defaults follow Blast_Sensor_Mesh_Design.md §11. Every value can be
 * overridden by defining it before including any mesh header (or via
 * compiler -D flags), so the firmware and the host simulator can tune
 * without editing the library.
 */
#ifndef MESH_CONFIG_H
#define MESH_CONFIG_H

/* ---- Table sizes (all memory is statically allocated inside mesh_t) ---- */

/* Neighbor table entries per node. Fleet is <=30 nodes; radio range limits
 * audible neighbors well below that in practice. */
#ifndef MESH_MAX_NEIGHBORS
#define MESH_MAX_NEIGHBORS 16
#endif

/* Pending-transmission queue depth. Sized for: 1 beacon rebroadcast +
 * 1 flood relay + own telemetry/ACK + several relayed upward frames. */
#ifndef MESH_TXQ_DEPTH
#define MESH_TXQ_DEPTH 12
#endif

/* Flood duplicate-suppression cache (§11: 32 entries covers TTL x retries). */
#ifndef MESH_FLOOD_DEDUP
#define MESH_FLOOD_DEDUP 32
#endif

/* Executed-command cache: suppresses re-execution when the base re-floods
 * a command whose ACK it lost (§6 step 4). */
#ifndef MESH_EXEC_DEDUP
#define MESH_EXEC_DEDUP 32
#endif

/* Per-hop duplicate cache for tree-routed unicast frames (covers the
 * lost-HOP_ACK retransmit case). */
#ifndef MESH_UP_DEDUP
#define MESH_UP_DEDUP 16
#endif

/* ---- Frame geometry (§8) ---- */

/* LoRa PHY payload ceiling. */
#ifndef MESH_MAX_FRAME
#define MESH_MAX_FRAME 255
#endif

/* Header is 8 bytes + 4-byte MIC trailer = 12 bytes overhead; payload is
 * capped at 240 per the design doc. */
#define MESH_HDR_SIZE 8
#define MESH_MIC_SIZE 4
#define MESH_OVERHEAD (MESH_HDR_SIZE + MESH_MIC_SIZE)
#ifndef MESH_MAX_PAYLOAD
#define MESH_MAX_PAYLOAD 240
#endif

/* ---- Routing metric (§5.2) ----
 * ETX and path cost are Q8.8 fixed point: 256 == 1.0 expected transmission.
 * 0xFFFF is infinity (unreachable / poisoned). */

#define MESH_ETX_ONE   256u
#define MESH_COST_INF  0xFFFFu

/* Beacon reception window K: ETX estimated over the last K beacon epochs.
 * Must be <= 16 (stored as a 16-bit bitmap). */
#ifndef MESH_ETX_WINDOW
#define MESH_ETX_WINDOW 16
#endif

/* Ceiling on a single link's ETX estimate (16.0). */
#ifndef MESH_ETX_MAX_Q8
#define MESH_ETX_MAX_Q8 (16u * MESH_ETX_ONE)
#endif

/* EWMA smoothing: new = old + (sample - old) >> shift. */
#ifndef MESH_ETX_EWMA_SHIFT
#define MESH_ETX_EWMA_SHIFT 2
#endif

/* Parent hysteresis: challenger must beat incumbent by this margin (0.5 ETX)
 * for this many consecutive beacon epochs before the node switches (§5.2). */
#ifndef MESH_HYST_MARGIN_Q8
#define MESH_HYST_MARGIN_Q8 (MESH_ETX_ONE / 2)
#endif
#ifndef MESH_HYST_EPOCHS
#define MESH_HYST_EPOCHS 2
#endif

/* A neighbor whose beacons have not been heard for this many epochs is
 * stale; a stale parent is declared dead and the route poisoned (§5.4). */
#ifndef MESH_STALE_EPOCHS
#define MESH_STALE_EPOCHS 4
#endif

/* ---- Flooding (§6) ---- */

/* Default TTL for flooded commands; also the hop budget for tree unicast. */
#ifndef MESH_FLOOD_TTL
#define MESH_FLOOD_TTL 8
#endif

/* Randomized rebroadcast backoff for flooded commands: 0..this, in us. */
#ifndef MESH_FLOOD_BACKOFF_US
#define MESH_FLOOD_BACKOFF_US 2000000u
#endif

/* Randomized rebroadcast offset for beacons within the beacon window. */
#ifndef MESH_BEACON_BACKOFF_US
#define MESH_BEACON_BACKOFF_US 2000000u
#endif

/* ---- Per-hop reliability for tree-routed unicast (design principle 4) ---- */

/* Retransmissions after the first send (total sends = 1 + retries). */
#ifndef MESH_HOP_RETRIES
#define MESH_HOP_RETRIES 3
#endif

/* Base retransmit interval and added random jitter, in us. */
#ifndef MESH_HOP_RETRY_US
#define MESH_HOP_RETRY_US 400000u
#endif
#ifndef MESH_HOP_RETRY_JITTER_US
#define MESH_HOP_RETRY_JITTER_US 200000u
#endif

/* Consecutive exhausted unicast sends to the parent before the link is
 * declared dead and the node re-parents (§5.4 "missed ACKs"). */
#ifndef MESH_PARENT_FAIL_LIMIT
#define MESH_PARENT_FAIL_LIMIT 3
#endif

/* Random initial delay before a tree-unicast send, in us. Desynchronizes
 * the many nodes that execute a flooded command at nearly the same time. */
#ifndef MESH_UP_JITTER_US
#define MESH_UP_JITTER_US 300000u
#endif

/* ---- Superframe scheduler (§4) ---- */

/* Default window lengths (§4 table) and mode periods (§11). */
#ifndef MESH_SF_BEACON_WIN_US
#define MESH_SF_BEACON_WIN_US 10000000u /* 10 s */
#endif
#ifndef MESH_SF_CMD_WIN_US
#define MESH_SF_CMD_WIN_US 20000000u /* 20 s */
#endif
#ifndef MESH_SF_TELEM_WIN_US
#define MESH_SF_TELEM_WIN_US 30000000u /* 30 s */
#endif
#ifndef MESH_SF_ACTIVE_PERIOD_US
#define MESH_SF_ACTIVE_PERIOD_US 60000000ull /* 60 s */
#endif
#ifndef MESH_SF_STANDBY_PERIOD_US
#define MESH_SF_STANDBY_PERIOD_US 900000000ull /* 15 min */
#endif

/* Telemetry micro-slot width (§4: ID-ordered slots inside the
 * telemetry window). 30 nodes x 1 s fills the default 30 s window. */
#ifndef MESH_SF_TELEM_SLOT_US
#define MESH_SF_TELEM_SLOT_US 1000000u
#endif

/* Listen guard: the radio wakes this early before the superframe start
 * and stays on this long past the last active window. */
#ifndef MESH_SCHED_GUARD_US
#define MESH_SCHED_GUARD_US 100000u /* 100 ms */
#endif

/* ---- Replay protection (§10.2) ----
 * With the scheduler enabled, every frame's MIC binds the transmit-time
 * superframe index; receivers accept the current index ± this tolerance
 * (covers senders straddling a superframe boundary). A recorded frame
 * replayed outside the tolerance fails the MIC statelessly. With the
 * scheduler disabled the index is a constant 0 and replay protection is
 * inactive. */
#ifndef MESH_MIC_SF_TOLERANCE
#define MESH_MIC_SF_TOLERANCE 1
#endif

/* ---- Route persistence (§5.3) ----
 * A rebooted root restores its saved sequence counters advanced by this
 * margin, guaranteeing they are ahead of anything the fleet saw between
 * the last save and the reboot. Save at least once per this many
 * beacon epochs (the examples save every 10 minutes). */
#ifndef MESH_PERSIST_SEQ_MARGIN
#define MESH_PERSIST_SEQ_MARGIN 64
#endif

/* ---- Bulk transfer (§7) ----
 * Recommended default chunk payload for FETCH originators. The chunk
 * size travels in each FETCH frame, so this is a convenience for base
 * applications, not a protocol constant. 96 B is the working point for
 * the RYLR998's hex-encoded AT transport (§3.4: 108 B payload ceiling
 * minus the 5 B DATA header, rounded to a power-of-two-ish size). */
#ifndef MESH_BULK_CHUNK
#define MESH_BULK_CHUNK 96
#endif

#endif /* MESH_CONFIG_H */
