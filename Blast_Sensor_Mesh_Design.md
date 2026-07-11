# Desert Blast-Wave Sensor Network
## LoRa Ad-Hoc Mesh: Protocol Design and Algorithms

*Design Document — Revision A — July 2026*

**Scope:** network architecture, routing, command dissemination, and bulk data retrieval for 2–30 solar-powered sensor nodes with GPS time reference. Implementation details (firmware structure, driver code) are out of scope for this revision.

---

## Contents

1. [System Overview and Goals](#1-system-overview-and-goals)
2. [Design Principles](#2-design-principles)
3. [Network Architecture](#3-network-architecture)
4. [Time Base and Duty Cycling](#4-time-base-and-duty-cycling)
5. [Neighbor Discovery and Collection-Tree Formation](#5-neighbor-discovery-and-collection-tree-formation)
6. [Command Dissemination (Flooding)](#6-command-dissemination-flooding)
7. [Bulk Data Retrieval](#7-bulk-data-retrieval)
8. [Frame Format](#8-frame-format)
9. [Node State Machine](#9-node-state-machine)
10. [Failure Modes, Robustness, and Link Security](#10-failure-modes-robustness-and-link-security)
11. [Tunable Parameters (Defaults)](#11-tunable-parameters-defaults)
12. [Alternatives Considered](#12-alternatives-considered)
13. [Open Questions for Revision B](#13-open-questions-for-revision-b)

---

## 1. System Overview and Goals

Each field node is a self-contained data logger (Raspberry Pi Pico, SD card, GPS receiver, barometric/blast pressure sensor, temperature sensor, solar power) deployed across a desert test range. Nodes record pressure and temperature at 1 kHz for up to one hour around a blast event. The next hardware iteration adds a LoRa radio to every node and to a base station, enabling:

- Command and control from base: arm, disarm, sleep, wake, status, time-window configuration.
- Wireless retrieval of recorded data after a test, without physically visiting each node.
- Passive health monitoring of the deployed fleet (battery, storage, GPS lock).

Terrain blocks line of sight between the base station and many nodes, so nodes must relay traffic for one another: an ad-hoc, self-healing multi-hop mesh. The network must tolerate nodes that are asleep, discharged, damaged, or removed, and must reconverge without operator intervention.

### 1.1 Design Constraints and Key Numbers

| Parameter | Value / Assumption | Design consequence |
|---|---|---|
| Fleet size | 2–30 nodes per deployment | Flat routing is sufficient; no hierarchical addressing needed. |
| Node positions | Mostly fixed between tests | Routes are stable; cache them in flash, refresh periodically. |
| Raw recording | 1 kHz × up to 1 h ≈ 20–50 MB/node | Full-file transfer over LoRa is infeasible; retrieval must be selective. |
| Event of interest | ~1–2 min around blast trigger | 0.5–1 MB raw; 100–300 kB after delta encoding + compression. |
| LoRa effective throughput | ~0.3–2 kB/s per hop (SF-dependent) | A 200 kB event window transfers in ~2–10 min per hop. |
| Latency requirement | Minutes–hours acceptable | Optimize for robustness and power, not speed. |
| Time reference | GPS on every node | Free sub-microsecond sync: enables TDMA slots and duty-cycled listening. |
| Power | Solar + battery | Radio must duty-cycle; scheduled wake windows rather than always-on receive. |

The single most important consequence of these numbers: the protocol is designed around retrieving the **event window**, not the whole recording. The full hour remains on the SD card as the archive of record, and the base can request any additional byte range on demand.

---

## 2. Design Principles

1. **Exploit GPS time.** Every node shares a common clock to within microseconds. All scheduling (listen windows, transfer slots, beacon epochs) is expressed in absolute GPS time, eliminating clock-drift protocols entirely.
2. **Traffic is asymmetric.** Nearly all traffic is base-to-nodes (small commands) or nodes-to-base (telemetry, bulk data). A collection tree rooted at the base matches this pattern better than a general any-to-any mesh.
3. **Two traffic classes, two mechanisms.** Small, rare, must-reach-everyone messages (commands) use controlled flooding. Large, targeted transfers (data retrieval) use tree routing with per-hop reliability.
4. **Per-hop reliability, end-to-end verification.** Each hop ACKs and retransmits locally (cheap over one lossy link). The base verifies file integrity end-to-end with a checksum and a chunk bitmap.
5. **Everything is resumable.** Nodes power-cycle and links fade. Transfers are chunked and idempotent; a node rebooting mid-transfer costs only the chunks in flight.
6. **The base is the only smart place.** Nodes run simple, fixed-memory algorithms; the base station (a real computer) does scheduling, bookkeeping, and retry policy.

---

## 3. Network Architecture

### 3.1 Roles

- **Base station (root):** LoRa gateway attached to the home-base server. Originates commands, schedules transfers, stores retrieved data. Node ID 0.
- **Field node:** sensor logger with LoRa. Relays traffic for neighbors. Node IDs 1–254 assigned at provisioning and written to flash.
- **Broadcast address:** 255, used by flooded commands and beacons.

### 3.2 Addressing and Identity

A 1-byte node ID is sufficient for ≤30 nodes and keeps headers small. Each node also carries a network ID (2 bytes) so co-located deployments or neighboring test ranges do not cross-talk. Frames with a foreign network ID are dropped at the MAC layer.

### 3.3 Radio Profile

Two logical channels are recommended (implemented as different frequency channels or spreading factors):

- **Control profile:** high spreading factor (e.g., SF10–SF12) for maximum range and link margin. Carries beacons, commands, ACKs, telemetry. Slow but very robust.
- **Bulk profile:** lower spreading factor (e.g., SF7–SF8) for throughput during data retrieval, negotiated per link only when both ends have adequate RSSI/SNR margin. Falls back to the control profile on weak links.

**Regulatory note:** confirm the ISM band and duty-cycle rules for the deployment region (e.g., US 915 MHz has no duty-cycle limit but has dwell-time rules; EU 868 MHz imposes 1–10% duty cycle, which materially affects bulk transfer planning).

---

## 4. Time Base and Duty Cycling

GPS provides each node with absolute time (PPS + NMEA). The protocol divides time into a repeating **superframe** aligned to GPS time-of-week, so every node independently computes the same schedule with no negotiation:

| Superframe segment | Default duration | Purpose |
|---|---|---|
| Beacon window | 10 s | Base emits beacon; nodes rebroadcast per Section 5. Tree maintenance. |
| Command window | 20 s | Flooded commands and command ACK collection. |
| Telemetry window | 30 s | Nodes report status up the tree in ID-ordered micro-slots. |
| Bulk window | 0–N min | Scheduled chunk transfers (Section 7). Absent when idle. |
| Sleep | Remainder | Radio off. MCU may also sleep unless armed/recording. |

The superframe period is an operating-mode parameter: e.g., every 15 minutes in standby (very low power), every 60 seconds in an active test window, and continuous during retrieval sessions. Mode changes are themselves commands, and every command carries the mode's start/stop GPS times so a node that misses the original command can still infer the schedule from any relayed copy.

If a node loses GPS lock, it free-runs on its crystal (drift of a few ppm) and widens its receive windows proportionally to time-since-lock — degrading gracefully rather than dropping off the network.

---

## 5. Neighbor Discovery and Collection-Tree Formation

### 5.1 Beacon Flood

Once per superframe the base transmits a beacon containing: network ID, beacon sequence number, sender ID (0), hop count (0), and a path-cost field (0). Each node that hears a beacon waits a short randomized offset within the beacon window, then rebroadcasts it with its own ID, incremented hop count, and updated path cost. Each node rebroadcasts at most one beacon per sequence number — its best one — so the flood costs O(N) transmissions.

### 5.2 Link Metric and Parent Selection

Each node maintains a neighbor table (neighbor ID, smoothed RSSI/SNR, estimated ETX, last-heard time, that neighbor's advertised path cost). The routing metric is cumulative ETX (expected transmission count):

```
path_cost(me) = min over neighbors n of ( path_cost(n) + ETX(me, n) )
```

where ETX of a link is estimated from beacon reception ratio over a sliding window of the last K beacon sequence numbers (e.g., K = 16), smoothed with an EWMA. The neighbor minimizing the sum becomes the parent. Hysteresis is applied: a node switches parents only if the challenger beats the incumbent by a margin (e.g., 0.5 ETX) for two consecutive superframes, preventing route flapping on marginal links.

### 5.3 Stability Optimizations for Fixed Deployments

- The full neighbor table and chosen parent are persisted to flash. On boot, a node resumes its last-known route immediately and repairs it lazily from subsequent beacons.
- The base builds a global topology map from telemetry (each status report includes the node's parent and top-3 neighbors). Operators can view the tree, spot single points of failure, and pre-plan relay node placement.
- Because positions are fixed, beacon frequency can be low in standby; the tree only needs to react to node death, not motion.

### 5.4 Loop Prevention and Route Repair

- A node never selects a parent whose advertised path cost is ≥ its own current cost (standard distance-vector loop guard).
- Beacon sequence numbers invalidate stale state: a node that has not refreshed its cost within M superframes (e.g., M = 4) advertises cost = infinity and stops relaying until re-attached.
- If a parent dies, its children detect missed beacons/ACKs, mark the link dead, and re-select from the neighbor table within one to two superframes — no global recomputation required.

---

## 6. Command Dissemination (Flooding)

Commands are small (≤ 32-byte payload), rare, and must reach every node even when the tree is stale — for example, waking the fleet after weeks of standby. Controlled flooding is the right tool:

1. Base transmits the command in the command window: `{network ID, msg type, command seq (monotonic), target (node ID or 255), TTL (default 8), payload, MAC}`.
2. Every node receiving a command with an unseen seq stores the seq in a small dedup cache, decrements TTL, and rebroadcasts once after a randomized backoff (0–2 s). Seen seqs are never rebroadcast.
3. Targeted commands (target ≠ 255) are still relayed by all nodes but executed only by the target.
4. Execution ACKs return via the tree: each executing node unicasts an ACK to its parent, which relays upward. The base maintains a checklist and re-floods (same seq, bumped retry counter) for missing nodes up to R times before flagging them to the operator.

**Safety-relevant semantics for the ARM command:** ARM carries an explicit validity window (GPS start/stop time). A node that receives ARM outside the window, or after a stop time, ignores it. This makes stale or replayed ARM commands inert (see also Section 10).

| Command | Payload | Effect |
|---|---|---|
| ARM | record window start/stop (GPS), trigger threshold | Enter armed state; begin high-rate logging at trigger or window start. |
| DISARM | — | Exit armed state. |
| STATUS | — | Return telemetry immediately in next telemetry window. |
| SLEEP / WAKE | mode, superframe period, until-time | Change duty-cycle mode. |
| LIST | — | Return recording index: event IDs, sizes, trigger timestamps, checksums. |
| FETCH | event ID or byte range, chunk size, session ID | Begin bulk transfer (Section 7). |
| SET-PARAM | key, value | Update tunable (thresholds, windows, radio profile). |
| TIME-MARK | label | Write an operator marker into the node's log. |

---

## 7. Bulk Data Retrieval

### 7.1 Event Detection and Selective Retrieval

During recording, the node runs a trigger detector (pressure threshold with configurable pre-trigger buffer, e.g., 10 s of pre-roll kept in RAM). On trigger, the node writes an event index entry to the SD card: event ID, GPS trigger time, start/end offsets in the raw file, sample count, and CRC32 of the event window. The LIST command returns this index; the operator (or an automatic base-station policy) then FETCHes event windows — typically 100–300 kB each after encoding — rather than the multi-megabyte raw file. Any byte range of the raw file remains fetchable for forensic follow-up.

### 7.2 On-Node Encoding

- Samples stored as fixed-point deltas (pressure changes slowly except at the shock front); escape code for full-value resync every 256 samples.
- Optional lightweight compression (heatshrink/LZ4-class) applied per 4 kB block so a corrupted chunk damages at most one block.
- Every transfer unit carries offsets into the original file, so decoding is position-independent and resumable.

### 7.3 Chunked Transfer Protocol

1. **Session setup:** base floods FETCH `{node, event ID, chunk size C (default 180 B payload), session ID}`. The target replies (via tree) with a session header: total chunks T, file CRC32, encoding flags.
2. **Streaming:** within its scheduled bulk slot, the source sends chunks `{session ID, chunk index, payload, per-chunk CRC}` to its parent in bursts of W chunks (window, default 16), then pauses for a per-hop selective ACK — a bitmap of the burst.
3. **Store-and-forward:** each relay buffers the burst in RAM, forwards it to its own parent under the same burst/bitmap discipline, and only then ACKs downstream. RAM budget: W × C ≈ 3 kB per active session — comfortably within the Pico's 264 kB.
4. **Completion:** the base assembles chunks, checks per-chunk CRCs, then requests any missing indices with a NACK bitmap (repeat until complete), and finally verifies the file CRC32. On mismatch, re-fetch only the suspect byte range.
5. **Resumption:** session state (session ID, bitmap of received chunks) lives at the base. After any interruption — node reboot, link loss, operator pause — the base re-issues FETCH with the same session ID and its current NACK bitmap; the source resends only what is missing.

### 7.4 Scheduling Transfers Across the Fleet

The base retrieves from nodes one branch at a time by default: only one active session per tree path avoids self-interference on shared relays. When the topology map shows two branches that are radio-independent (no shared relays, adequate spatial separation), the base may schedule them in parallel TDMA slots within the bulk window — GPS time makes the slot boundaries exact. With 30 nodes × ~200 kB event windows at a conservative 0.5 kB/s effective multi-hop rate, full-fleet retrieval is on the order of 3–4 hours serialized, and proportionally less with parallel branches; well within the between-test timeline.

---

## 8. Frame Format

All frames share a common header inside the LoRa PHY payload (max 255 B):

| Field | Size | Notes |
|---|---|---|
| Network ID | 2 B | Deployment identifier; foreign frames dropped. |
| Msg type | 1 B | BEACON, CMD, CMD-ACK, TELEM, DATA, DATA-ACK, SESSION. |
| Src ID / Dst ID | 1 B + 1 B | Dst 255 = broadcast/flood. |
| Seq | 2 B | Per-type sequence for dedup and windows. |
| TTL / hops | 1 B | Flood control; also records hop count in beacons. |
| Payload | 0–240 B | Type-specific. |
| MIC | 4 B | Message integrity code (Section 10). |

Total overhead is 12 bytes, leaving ~180–240 bytes of useful payload per frame depending on radio settings — the basis for the default chunk size.

---

## 9. Node State Machine

| State | Radio behavior | Entry / exit |
|---|---|---|
| STANDBY | Wake only for superframe windows (long period). | Default after boot. Exit on WAKE/ARM. |
| ACTIVE | Short superframe period; responsive to commands. | Enter on WAKE. Exit on SLEEP or timeout. |
| ARMED | As ACTIVE, but sensor pipeline hot; pre-trigger ring buffer running. | Enter on ARM (within validity window). Exit on DISARM, trigger, or window end. |
| RECORDING | Radio silent or minimal (avoid SD contention and EMI near instruments). | Enter on trigger. Exit at record window end → writes event index → ACTIVE. |
| TRANSFER | Continuous scheduled bulk slots. | Enter on FETCH addressed to self or when relaying a session. Exit at session end. |
| FAULT | Beacon-listen only; telemetry reports fault code. | SD failure, sensor failure, brown-out loop. Exit on operator CLEAR or reboot. |

All state transitions and their causes are appended to a small event log on the SD card, retrievable like any other file — invaluable for post-mortem debugging of field behavior.

---

## 10. Failure Modes, Robustness, and Link Security

### 10.1 Failure Modes

| Failure | Detection | Recovery |
|---|---|---|
| Relay node dies mid-network | Children miss beacons / per-hop ACKs. | Children re-parent from neighbor table in 1–2 superframes; base sees topology change in telemetry. |
| Node offline during command | Missing CMD-ACK at base. | Base re-floods up to R times, then flags node; commands with validity windows self-expire. |
| Transfer interrupted | Session timeout at base. | Resume via session ID + NACK bitmap; only missing chunks resent. |
| GPS lock lost | Node-side PPS timeout. | Free-run clock with widened RX windows; telemetry reports lock state. |
| SD corruption | CRC mismatch on read; mount failure. | Node enters FAULT, keeps radio alive so it can still be diagnosed remotely. |
| Network partition (terrain + deaths) | Base topology map shows unreachable subtree. | Unreachable nodes keep logging autonomously; data retrieved after physical relay placement or node revival. |
| Duplicate/stale floods | Seq dedup cache. | Dropped silently. |

### 10.2 Link Security

Even on a private test range, the ARM path warrants protection against accidental or malicious injection (LoRa is trivially receivable and transmittable with hobby hardware):

- All frames authenticated with a 4-byte MIC (AES-CMAC truncated) under a pre-shared network key provisioned at flash time. Frames failing the MIC are dropped before parsing.
- **Replay protection:** the MIC covers the sequence number, and nodes reject non-increasing command seqs. Combined with ARM validity windows, replayed ARM frames are doubly inert.
- Payload encryption (AES-CTR) is optional and cheap on the Pico if data confidentiality is desired; authentication is the non-negotiable part.

---

## 11. Tunable Parameters (Defaults)

| Parameter | Default | Rationale |
|---|---|---|
| Superframe period (standby / active) | 15 min / 60 s | Power vs. responsiveness. |
| Beacon ETX window K | 16 beacons | Stable ETX without long convergence. |
| Parent hysteresis | 0.5 ETX, 2 superframes | Prevents flapping on marginal links. |
| Flood TTL | 8 | > expected network diameter for 30 nodes. |
| Command retries R | 3 | Balance reach vs. air-time. |
| Chunk payload C | 180 B | Fits one frame with header + MIC. |
| Burst window W | 16 chunks | ~3 kB relay RAM; good pipe utilization. |
| Dedup cache | 32 entries | Covers TTL × retry space. |
| Stale-route timeout M | 4 superframes | Fast poisoning of dead paths. |
| Pre-trigger ring buffer | 10 s | Captures onset before threshold crossing. |

---

## 12. Alternatives Considered

- **Pure managed flooding for everything (Meshtastic-style):** simplest possible, and adequate for commands — but bulk transfer floods the entire network per chunk, wasting air-time and power by roughly the node count. Rejected for data; retained for commands.
- **On-demand routing (AODV / LoRaMesher):** general any-to-any routing the traffic pattern does not need; route discovery floods add latency and complexity with no benefit over a proactive tree when the root is fixed and positions are static.
- **Off-the-shelf stacks (Meshtastic, Reticulum):** fastest path to a demo and worth prototyping against, but large-file transfer, GPS-slotted scheduling, and ARM-safety semantics would all be bolt-ons; a purpose-built ~2k-line protocol is likely simpler than the workarounds.
- **LoRaWAN:** assumes all nodes reach a gateway in one hop — precisely the assumption the terrain violates.

---

## 13. Open Questions for Revision B

1. Regulatory region and band (drives duty-cycle budget and bulk-window design).
2. Whether RECORDING should be fully radio-silent (EMI near sensors, SD write contention) or allow beacon listening.
3. Second radio channel feasibility on chosen LoRa module (concurrent control + bulk).
4. Whether the base station should support a mobile mode (vehicle-mounted gateway driving the range to shorten hop counts for retrieval).
5. Key management workflow: per-deployment key rotation at provisioning vs. fixed fleet key.
