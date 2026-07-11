# Desert Blast-Wave Sensor Network
## LoRa Ad-Hoc Mesh: Protocol Design and Algorithms

*Design Document — Revision B — July 2026*

**Scope:** network architecture, routing, command dissemination, and bulk data retrieval for 2–30 solar-powered sensor nodes with GPS time reference. Implementation details (firmware structure, driver code) are out of scope, but Revision B folds back what implementing and simulating the core mesh taught us.

**Implementation status:** the core mesh (§5, §6, §8, and the MIC of §10.2) is implemented as the radio-agnostic C library at the root of this repository, validated on a host simulator (fixed-seed scenario tests plus a 25-seed randomized sweep) — see `README.md`. Superframe scheduling (§4) and bulk transfer (§7) are design-only as of this revision.

### Revision History

| Rev | Date | Changes |
|---|---|---|
| A | July 2026 | Initial protocol design. |
| B | July 2026 | Findings from the M1 implementation: observed-epoch ETX windows and estimator restart (§5.2), HOP-ACK frame type and per-hop seq semantics (§6.4, §8), two-level flood dedup — per-transmission seq vs. persistent command ID (§6), MIC implemented as AES-128-CMAC with replay protection still open (§10.2), REYAX RYLR998 selected as the first radio module with its transport constraints (§3.4), tunables table extended (§11). |

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
13. [Open Questions for Revision C](#13-open-questions-for-revision-c)

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

### 3.4 Selected Radio Module (Rev B): REYAX RYLR998

The first hardware iteration uses the REYAX RYLR998 (Semtech LoRa engine behind a UART AT-command interface; datasheets in `examples/radios/RYLR998/`). The protocol layer stays radio-agnostic — the RYLR998 binding is an example driver — but the module imposes transport constraints the design must respect:

- **ASCII payload transport.** The AT interface specifies the `AT+SEND` data field as ASCII (max 240 bytes), so binary frames are hex-encoded on the air: 2 characters per byte, **capping a frame at 120 bytes** (108 bytes of §8 payload). Control traffic (beacons, commands, ACKs, telemetry: 15–40 B frames) is unaffected; bulk-transfer chunk size (§7.3) must be sized to ~96 B under this transport, roughly doubling chunk count and air time. Options for M3: accept the overhead, validate raw-binary AT payloads on the bench (off-spec), or move to an SPI-attached radio for bulk-heavy deployments.
- **Single channel.** One modem, one RF profile at a time. The control/bulk split of §3.3 becomes *temporal* (reconfigure SF per bulk session) rather than concurrent — or a second module per node, which the answer to open question 3 (§13) now depends on.
- **Modulation limits.** SF5–11 with SF ≤ 9 at 125 kHz, ≤ 10 at 250 kHz, ≤ 11 at 500 kHz — the SF12 end of the §3.3 control profile is unavailable; SF9/125 kHz (module default, ~190 B/s) is the working control profile. TX power 0–22 dBm (≤ 14 dBm for CE).
- **Half duplex with explicit busy signaling.** The module rejects a send while the previous transmission is on the air (`+ERR=17`); the driver spaces transmissions using a conservative air-time estimate and the mesh's retransmit ladder absorbs the rest.
- **Power modes map to §4.** `AT+MODE=1` (sleep, ~10 µA) and the smart-receive mode (`AT+MODE=2`, sub-100 µA duty-cycled listen) are the intended mechanism for superframe duty cycling in M2.
- The module's own address/group fields duplicate mesh functions and are not relied on: every frame goes to module broadcast address 0, and deployment separation comes from the in-frame network ID plus the MIC key (§10.2), not the module's `NETWORKID`. Its `AT+CPIN` "domain password" scrambling is not used — the crypto is unspecified, and §10.2 provides real authentication.

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

where ETX of a link is estimated from beacon reception ratio over a sliding window of the last K beacon epochs (e.g., K = 16), smoothed with an EWMA. The neighbor minimizing the sum becomes the parent. Hysteresis is applied: a node switches parents only if the challenger beats the incumbent by a margin (e.g., 0.5 ETX) for two consecutive superframes, preventing route flapping on marginal links.

**Rev B refinements (from simulator validation).** Randomized multi-seed simulation exposed three defects in the naive "window slides by beacon sequence delta" estimator, all adopted in the implementation:

1. **Observed-epoch windows.** Beacons propagate as a single-shot flood wave (§5.1), so on deep or lossy paths entire epochs legitimately fail to reach a node. If the reception window slides by raw sequence delta, that upstream attenuation is booked as *local* link loss against every neighbor, inflating costs fleet-wide (simulated: clean 3-hop chains converging to ~5.7 ETX instead of 3.0, and deep nodes attaching epochs late). The window therefore slides only by epochs the node *observed* — heard at least one beacon from anyone. An epoch nobody delivered punishes no one; an epoch where some neighbors delivered and one stayed silent punishes exactly the silent one.
2. **In-progress epoch grace.** Within a beacon window, a neighbor whose randomized rebroadcast offset simply hasn't fired yet must not be scored as a miss; the epoch in progress is excluded from the ratio until the next epoch begins. Without this, evaluation order inside the window systematically penalizes late-offset neighbors and can starve parent-switch hysteresis indefinitely.
3. **Estimator restart on reappearance.** A neighbor silent past the staleness threshold M gets a *fresh* estimator when heard again, instead of dragging up to K epochs of outage history (at standby cadence, hours of pessimism against a healthy revived relay). Hysteresis still gates the actual parent switch, so this does not reintroduce flapping.

### 5.3 Stability Optimizations for Fixed Deployments

- The full neighbor table and chosen parent are persisted to flash. On boot, a node resumes its last-known route immediately and repairs it lazily from subsequent beacons.
- The base builds a global topology map from telemetry (each status report includes the node's parent and top-3 neighbors). Operators can view the tree, spot single points of failure, and pre-plan relay node placement.
- Because positions are fixed, beacon frequency can be low in standby; the tree only needs to react to node death, not motion.

### 5.4 Loop Prevention and Route Repair

- A node never selects a parent whose advertised path cost is ≥ its own current cost (standard distance-vector loop guard).
- Beacon sequence numbers invalidate stale state: a node that has not refreshed its cost within M superframes (e.g., M = 4) advertises cost = infinity and stops relaying until re-attached.
- If a parent dies, its children detect missed beacons/ACKs, mark the link dead, and re-select from the neighbor table within one to two superframes — no global recomputation required. Per-hop ACK exhaustion (§6.4) triggers the same failover immediately, without waiting out the staleness timer.
- **Rev B note:** unlike the ETX window (§5.2), staleness deliberately counts *raw* beacon-sequence delta, not observed epochs — the root's sequence counter is ground truth for elapsed superframes, and a dead parent must be poisoned promptly even when the node is observing little traffic.

---

## 6. Command Dissemination (Flooding)

Commands are small (≤ 32-byte payload), rare, and must reach every node even when the tree is stale — for example, waking the fleet after weeks of standby. Controlled flooding is the right tool.

### 6.1 Flood Mechanics and Two-Level Deduplication

**Rev B refinement:** implementation showed that one sequence number cannot serve both flood-relay suppression and exactly-once execution. If a re-flood reuses the original frame seq (as Rev A implied), every relay's dedup cache suppresses it and the retry never propagates. Two identifiers with different lifetimes are required:

- the frame **seq** identifies one flood *transmission* — fresh for every attempt, deduplicated in a small relay cache so each node rebroadcasts a given wave at most once;
- a 2-byte **command ID** inside the payload identifies the *command* — stable across re-floods, checked against an executed-commands cache so a node never executes twice, but re-ACKs when it sees a re-flood of something it already ran (covering the lost-ACK case).

1. Base transmits the command in the command window: `{network ID, msg type, seq (fresh per attempt), target (node ID or 255), TTL (default 8), payload = command ID + opcode/args, MIC}`.
2. Every node receiving a command frame with an unseen seq stores the seq in the relay dedup cache, decrements TTL, and rebroadcasts once after a randomized backoff (0–2 s). Seen seqs are never rebroadcast.
3. Targeted commands (target ≠ 255) are still relayed by all nodes but executed only by the target.
4. Execution: if the command ID is new, the node executes and records it, then ACKs; if already recorded, it re-ACKs without re-executing. ACKs return via the tree (§6.4). The base maintains a checklist and re-floods the same command ID (with a fresh seq) for missing nodes up to R times before flagging them to the operator.

### 6.2 Safety-Relevant Semantics for ARM

ARM carries an explicit validity window (GPS start/stop time). A node that receives ARM outside the window, or after a stop time, ignores it. This makes stale or replayed ARM commands inert (see also Section 10).

### 6.3 Command Set

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

### 6.4 Upward Traffic: Per-Hop Reliable Tree Unicast (Rev B)

Command ACKs and telemetry travel node → parent → … → base. Design principle 4 (per-hop reliability) needs an explicit mechanism that Rev A's frame-type list lacked for small control frames — DATA-ACK exists only inside bulk sessions. Rev B adds a **HOP-ACK** frame type (§8):

- A node unicasts an upward frame (CMD-ACK or TELEM) to its parent and expects an immediate HOP-ACK naming the acknowledged type and seq. Missing HOP-ACKs trigger retransmission (default: 3 retries at ~400 ms + random jitter).
- The receiving parent HOP-ACKs **before** duplicate-checking, so a node whose HOP-ACK was lost (rather than its data frame) is still silenced; a per-hop dedup cache keyed `(src, type, seq)` prevents double-forwarding.
- The frame **seq is re-stamped at every hop** — it identifies one link-layer transfer, not the end-to-end message. End-to-end identity lives in the payload (origin node ID + command ID), which is what the base's checklist keys on.
- Repeated retry exhaustion toward the parent (default: 3 consecutive exhausted sends) declares the parent link dead and triggers the §5.4 failover immediately — per-hop ACKs double as a fast dead-parent detector between beacon epochs.
- Sends are staggered by a random initial jitter (~0–300 ms) because a flooded broadcast command makes many nodes ACK at nearly the same moment.

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
| Msg type | 1 B | BEACON (0x01), CMD (0x02), CMD-ACK (0x03), TELEM (0x04), HOP-ACK (0x05, Rev B — §6.4), DATA (0x06), DATA-ACK (0x07), SESSION (0x08). |
| Src ID / Dst ID | 1 B + 1 B | Dst 255 = broadcast/flood. |
| Seq | 2 B | Per-type sequence for dedup and windows. For tree-unicast frames it is re-stamped at each hop (§6.4); end-to-end identity is payload-level. |
| TTL / hops | 1 B | Flood control; also records hop count in beacons. |
| Payload | 0–240 B | Type-specific. |
| MIC | 4 B | Message integrity code (Section 10). AES-128-CMAC truncated; covers header + payload. |

Total overhead is 12 bytes, leaving ~180–240 bytes of useful payload per frame depending on radio settings — the basis for the default chunk size. **Rev B note:** over the RYLR998's hex-encoded AT transport (§3.4) the frame ceiling is 120 bytes (108 B payload); control frames are unaffected, bulk chunks must be sized down accordingly.

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

- All frames authenticated with a 4-byte MIC under a pre-shared network key provisioned at flash time. Frames failing the MIC are dropped before parsing. **Implemented in Rev B:** AES-128-CMAC (RFC 4493) truncated to 4 bytes, portable software AES verified against the RFC known-answer vectors; the 2⁻³² blind-forgery bound is rate-limited by LoRa air time to a few attempts per second. The library keeps the MIC pluggable (a null stub remains for bench/simulator use only).
- **Replay protection — open as of Rev B.** The MIC makes frames unforgeable but not un-replayable: a recorded valid frame can be retransmitted later. Current mitigations are robustness features, not security boundaries — the executed-command cache suppresses replayed command IDs only until it wraps or the node reboots, and ARM validity windows make stale ARMs inert at the application layer. A real fix needs replay state bound into the MIC: either a persisted per-origin frame counter or — more attractive here — the GPS-time superframe index that M2 introduces anyway, which every node can validate statelessly. Decide with the M2 wire-format revision (open question, §13).
- The RYLR998's `AT+CPIN` module encryption is **not** used as a security control (unspecified algorithm); §10.2 authentication happens above the radio.
- Payload encryption (AES-CTR) is optional and cheap on the Pico if data confidentiality is desired; authentication is the non-negotiable part.

---

## 11. Tunable Parameters (Defaults)

| Parameter | Default | Rationale |
|---|---|---|
| Superframe period (standby / active) | 15 min / 60 s | Power vs. responsiveness. |
| Beacon ETX window K | 16 observed epochs | Stable ETX without long convergence (§5.2 Rev B). |
| Parent hysteresis | 0.5 ETX, 2 superframes | Prevents flapping on marginal links. |
| Flood TTL | 8 | > expected network diameter for 30 nodes. |
| Command retries R | 3 | Balance reach vs. air-time. |
| Flood rebroadcast backoff | 0–2 s random | Desynchronizes relays (§6.1). |
| Hop retries (tree unicast) | 3, at 400 ms + 0–200 ms jitter | Per-hop reliability without hogging air (§6.4). |
| Parent-fail limit | 3 exhausted sends | Fast dead-parent failover between epochs (§6.4). |
| Upward send jitter | 0–300 ms | Desynchronizes post-flood ACK bursts (§6.4). |
| Chunk payload C | 180 B (96 B on RYLR998 AT transport) | Fits one frame with header + MIC (§3.4, §8). |
| Burst window W | 16 chunks | ~3 kB relay RAM; good pipe utilization. |
| Flood relay dedup cache | 32 seqs | Covers TTL × retry space. |
| Executed-command cache | 32 command IDs | Exactly-once execution across re-floods (§6.1). |
| Per-hop dedup cache | 16 (src, type, seq) | Covers the lost-HOP-ACK retransmit case (§6.4). |
| Stale-route timeout M | 4 superframes | Fast poisoning of dead paths. |
| Pre-trigger ring buffer | 10 s | Captures onset before threshold crossing. |

---

## 12. Alternatives Considered

- **Pure managed flooding for everything (Meshtastic-style):** simplest possible, and adequate for commands — but bulk transfer floods the entire network per chunk, wasting air-time and power by roughly the node count. Rejected for data; retained for commands.
- **On-demand routing (AODV / LoRaMesher):** general any-to-any routing the traffic pattern does not need; route discovery floods add latency and complexity with no benefit over a proactive tree when the root is fixed and positions are static.
- **Off-the-shelf stacks (Meshtastic, Reticulum):** fastest path to a demo and worth prototyping against, but large-file transfer, GPS-slotted scheduling, and ARM-safety semantics would all be bolt-ons; a purpose-built ~2k-line protocol is likely simpler than the workarounds.
- **LoRaWAN:** assumes all nodes reach a gateway in one hop — precisely the assumption the terrain violates.

---

## 13. Open Questions for Revision C

Carried over from Rev A (with status) and new from the Rev B work:

1. Regulatory region and band (drives duty-cycle budget and bulk-window design). *Examples default to US 915 MHz; still needs a per-deployment decision.*
2. Whether RECORDING should be fully radio-silent (EMI near sensors, SD write contention) or allow beacon listening. *Open.*
3. ~~Second radio channel feasibility on chosen LoRa module.~~ *Resolved by module selection: the RYLR998 is single-channel (§3.4). The bulk profile becomes temporal (per-session reconfiguration), unless a second module per node is added.*
4. Whether the base station should support a mobile mode (vehicle-mounted gateway driving the range to shorten hop counts for retrieval). *Open.*
5. Key management workflow: per-deployment key rotation at provisioning vs. fixed fleet key. *Sharpened by the Rev B MIC: the key is now load-bearing; the examples compile it in, which is not a production flow.*
6. **New:** replay-protection mechanism (§10.2) — persisted frame counter vs. GPS-superframe-bound MIC; decide together with the M2 superframe wire format.
7. **New:** bulk transfer over the RYLR998's hex transport (§3.4) — accept ~2× air time at 96 B chunks, qualify off-spec binary AT payloads on the bench, or move bulk-heavy deployments to an SPI radio.
