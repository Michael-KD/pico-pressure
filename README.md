# Desert Blast-Wave Sensor Network — RF Mesh Library

Firmware-grade C library implementing the ad-hoc LoRa mesh described in
[Blast_Sensor_Mesh_Design.md](Blast_Sensor_Mesh_Design.md) (Rev B) — the
protocol that lets a base station command, monitor, and retrieve data from
a fleet of solar-powered blast-pressure loggers scattered across a desert
test range with no line of sight between most nodes. Ships with example
Pico firmware and a driver for the REYAX RYLR998 LoRa module
([examples/](examples/)).

## The system

Each field node is a self-contained data logger — Raspberry Pi Pico, SD
card, GPS receiver, barometric/blast pressure sensor, temperature sensor,
solar power. Nodes
record pressure and temperature at 1 kHz around a blast event. The next
hardware iteration adds a LoRa radio to every node and to a base station
so that operators can:

- **command the fleet** from base: arm, disarm, sleep, wake, status,
  time-window configuration;
- **retrieve recorded event windows** wirelessly after a test instead of
  driving to every node;
- **monitor fleet health** passively (battery, storage, GPS lock).

Terrain blocks line of sight between the base and many nodes, so nodes
relay traffic for one another: a self-healing multi-hop mesh, rooted at
the base station, that tolerates nodes that are asleep, discharged,
damaged, or removed.

## What this library is

A portable, radio-agnostic C11 library — the core mesh (M1), the
superframe scheduler (M2), and bulk retrieval (M3) — implementing the
design doc's collection-tree architecture:

- **Frame codec** (§8): 12-byte overhead (8 B header + 4 B MIC trailer),
  network-ID gate, pluggable message-integrity hook.
- **Link authentication** (§10.2): AES-128-CMAC MIC (RFC 4493, verified
  against the RFC test vectors) under a pre-shared network key; frames
  failing the MIC are dropped before parsing. A null stub remains for
  bench/simulator use.
- **Replay protection** (§10.2): the MIC binds the sender's superframe
  index — appended to the MIC input, never transmitted, so the wire
  format is unchanged — sealed at the moment of transmission and
  verified ±1 superframe by the receiver. A recorded frame replayed
  later fails the MIC statelessly. Requires the scheduler (a shared GPS
  clock); without it the index is constant and the binding is inert.
- **Neighbor discovery + ETX link estimation** (§5.2): beacon reception
  ratio over a sliding 16-epoch window, EWMA-smoothed, Q8.8 fixed point
  (no floats — the Pico's M0+ has no FPU).
- **Collection-tree routing** (§5): cumulative-ETX parent selection with
  hysteresis (0.5 ETX × 2 epochs), distance-vector loop guard, stale-route
  poisoning, immediate failover on dead links.
- **Command flooding** (§6): controlled flood with per-seq dedup, TTL,
  randomized rebroadcast backoff; targeted or broadcast; exactly-once
  execution with re-ACK on re-flood.
- **Tree-unicast upward traffic** (§6.4, principle 4): CMD-ACKs and
  telemetry hop from parent to parent with per-hop ACK + retransmit;
  repeated per-hop failures feed dead-link detection.
- **Telemetry with routing context** (§5.3): every report carries the
  sender's parent and top-3 neighbors, letting the base build the global
  topology map.
- **Superframe scheduler** (§4): GPS-time-aligned windows (beacon /
  command / telemetry / bulk / sleep) that every node computes
  independently; transmissions are gated into their windows, telemetry
  gets ID-ordered micro-slots, the radio duty-cycles off in the sleep
  segment, and standby/active mode switches take effect fleet-wide at a
  commanded absolute time. GPS-loss degrades gracefully (widened listen
  windows). Optional — disabled, the library runs M1-style with the
  application scheduling beacons.
- **Bulk data retrieval** (§7): the base FETCHes a byte range from any
  node; the node streams it back up the tree as CRC-protected chunks on
  the same per-hop-reliable path, one in flight at a time. The base
  holds the received-chunk bitmap and re-FETCHes gaps under the same
  session — resumption is base-side state, so a node reboot costs only
  the chunk in flight. Data comes from (and lands in) application
  callbacks, so the SD card and event index stay out of the library.
- **Route persistence** (§5.3): snapshot API for flash caching — a
  rebooted node resumes its last-known route instantly (telemetry flows
  before the first post-reboot beacon) with §5.4 staleness intact, and
  a rebooted base restores its sequence counters ahead of the fleet's
  held epoch instead of wedging the network at seq 0.

The same library serves both roles: node ID 0 is the root (base-station
gateway); IDs 1–254 are field nodes.

### Design rules

- **No dynamic allocation.** All state lives in one caller-allocated
  `mesh_t` (~5 kB). Fixed-size tables, ring dedup caches.
- **Externally clocked.** Every call takes `now_us` (absolute GPS time in
  microseconds). The library never reads a clock, never sleeps, never
  calls the OS — which is why entire fleets run in-process on the host
  simulator and why the Pico port is a thin shim.
- **Radio-agnostic.** All radio contact goes through a two-function
  vtable (`mesh_radio.h`). A LoRa driver, some other sub-GHz radio, and
  the test simulator are interchangeable.
- **The base is the only smart place** (principle 6): the library gives
  the root primitives (`beacon`, `command`, ACK/telemetry callbacks);
  retry policy, checklists, and scheduling belong to the base-station
  application.
- **App semantics stay out:** ARM validity windows, the §9 node state
  machine, SD card, and sensors are the firmware application's job. The
  library delivers verified command payloads via callback and returns
  your ACK decision.

## Repository layout

```
Blast_Sensor_Mesh_Design.md   protocol spec (Rev B) — the design authority
CMakeLists.txt                library + host tests (C11, -Wall -Wextra -Werror)
include/mesh/
  mesh.h                      public API: mesh_t, init/rx/poll, send, callbacks
  mesh_config.h               tunables (§11 defaults), all overridable via -D
  mesh_radio.h                radio HAL vtable — the radio-agnostic boundary
  mesh_mic.h                  MIC interface: AES-128-CMAC + null stub
  mesh_sched.h                superframe scheduler API (§4)
src/
  frame.c                     frame build/parse (§8)
  neighbor.c                  neighbor table, observed-epoch ETX windows
  tree.c                      parent selection, hysteresis, loop guard, poisoning
  beacon.c                    beacon flood, best-beacon rebroadcast (§5.1)
  flood.c                     command flooding, dedup, TTL (§6)
  command.c                   execution + ACK, per-hop-reliable tree unicast
  telemetry.c                 status reports with routing context (§5.3)
  bulk.c                      FETCH sessions, chunk streaming, CRC16 (§7.3)
  persist.c                   route/counter snapshots for flash caching (§5.3)
  txqueue.c                   due-time TX queue: backoffs, retries, exhaustion
  sched.c                     superframe math, window gating, duty cycling
  mic_cmac.c                  AES-128-CMAC MIC (RFC 4493), portable software AES
  mic_null.c                  passthrough MIC stub (bench/simulator only)
  mesh.c                      init, RX dispatch, poll, introspection
  mesh_internal.h             module interfaces (tests may include this)
examples/                     standalone pico-sdk project (see below)
  main.c                      field-node firmware (Pico + RYLR998)
  base_station.c              base-gateway firmware with USB-serial CLI
  node_config.h               deployment radio/timing config + bench fallbacks
  provision.[ch]              flash provisioning blob codec (ID + net + key)
  provision_cli.c             `provision` CLI command (both firmwares)
  flash_store.[ch]            fixed-slot blob store in the Pico's flash
  gps.[ch]                    GPS time source: NMEA + PPS clock discipline
  gps_pico.[ch]               UART/PPS-IRQ glue for the GPS receiver
  rylr998_pico.[ch]           Pico UART/IRQ glue for the radio driver
  radios/RYLR998/             portable AT-protocol driver core + datasheets
tests/
  unit/                       frame / MIC / sched / neighbor / tree / flood / bulk / driver tests
  sim/sim_net.[ch]            multi-node simulator: virtual clock, lossy links
  sim/test_scenarios.c        integration scenarios (see below)
  sim/demo.c                  runnable ASCII demo of the whole protocol
  test_util.h                 tiny assert harness (no framework dependency)
```

## Getting started

Build and test on any Linux host (WSL Ubuntu works; needs `cmake`,
`ninja` or `make`, and a C compiler):

```sh
cmake -B build -G Ninja
cmake --build build
ctest --test-dir build            # 12 suites, all green
./build/tests/mesh_demo           # watch a 10-node fleet form, flood, heal
```

The demo prints the collection tree as it forms, floods an ARM command
and shows the ACK checklist, then kills a relay node and shows the
orphaned branch re-routing around it — and re-optimizing after the relay
revives.

### Running it on real hardware (Pico + RYLR998)

[examples/](examples/) is a standalone pico-sdk project with complete
firmware for both roles plus a driver for the REYAX RYLR998 UART LoRa
module (datasheets included in `examples/radios/RYLR998/`):

```sh
export PICO_SDK_PATH=/path/to/pico-sdk     # needs arm-none-eabi-gcc
cmake -B examples/build -G Ninja examples
cmake --build examples/build
```

Flash `mesh_base_station.uf2` to one Pico and `mesh_field_node.uf2` to
the others — the **same UF2 serves every field node**. Personalize each
device once over its USB serial (`openssl rand -hex 16` makes the key;
same key + netid fleet-wide, unique id per device):

```
> provision 3 D5E7 8f3a9c01d24be677a1509b8c33ef1d42
```

Unprovisioned devices fall back to the compiled `node_config.h` bench
identity. Wire each RYLR998 to 3V3/GND, GP4→RXD, GP5←TXD (and GP6→NRST,
optional). For field operation add a GPS receiver: GPS TX→GP1, and
PPS→GP2 if your module exposes it (worth it — ±1 ms fleet alignment vs
±400 ms). A device that locks GPS at boot runs the superframe scheduler
on absolute time; one that doesn't free-runs in bench mode and says so.
The base station gives you a CLI on USB serial:

```
> arm 30        flood ARM (30 min) — ACKs print as they arrive
> status        nodes answer with telemetry
> top           topology map / ACK checklist
> resend        re-flood the last command for stragglers
> stats         mesh + radio counters
> list 3        node 3's recording index
> fetch 3       retrieve node 3's event data over the mesh
> fstat         chunks received / missing; final CRC
> refetch       resume from the first missing chunk
```

(Full CLI reference, including how to read the topology map and how to
add new commands:
[examples/BASE_STATION_COMMANDS.md](examples/BASE_STATION_COMMANDS.md).)

All frames are authenticated with AES-128-CMAC under the provisioned
key. Field nodes blink fast while searching and slow once attached, and
log everything on their own USB port (which also accepts `provision`
and `stats`).

The driver split is deliberate: `radios/RYLR998/rylr998.[ch]` is the
platform-free AT-protocol state machine (hex framing, +RCV parsing,
config sequence, busy gating — unit-tested on the host), and
`rylr998_pico.c` is ~100 lines of UART glue. A different radio needs
only the two `mesh_radio_ops_t` functions plus a call to `mesh_rx()`
per received frame; `mesh_rx`/`mesh_poll` are main-loop-only, never
ISR.

### Node-side commands

```c
static bool app_on_command(void *user, uint16_t cmd_id,
                           const uint8_t *payload, uint8_t len) {
    /* payload is app-defined: e.g. payload[0] = opcode 'A' (ARM) with a
     * GPS validity window — validate per design doc §6, then: */
    return true;    /* true = library records execution + ACKs to base */
}

mesh_node_telemetry(&mesh, status_bytes, n, now);  /* up the tree */
```

### Base-station side

```c
/* With cfg.sched.enabled the library beacons itself each superframe;
 * otherwise the app calls this on its own schedule: */
mesh_root_beacon(&mesh, now);

uint16_t id = mesh_root_command(&mesh, MESH_ID_BROADCAST,
                                arm_payload, n, 0, now);
/* collect ACKs via cb.on_command_ack; for stragglers, re-flood the SAME
 * command id — executed nodes re-ACK without re-executing: */
mesh_root_command(&mesh, MESH_ID_BROADCAST, arm_payload, n, id, now);
```

Telemetry arrives via `cb.on_telemetry` with each node's parent and
top-3 neighbors — the base's topology map (§5.3) falls straight out.

### Retrieving data

```c
/* Node side: serve bytes for a FETCH. Returning < max ends the
 * session, so a short read is the EOF marker. */
static uint8_t app_on_bulk_read(void *user, uint8_t session,
                                uint32_t offset, uint8_t *buf,
                                uint8_t max) {
    return sd_read(event_base + offset, buf, max);  /* your storage */
}

/* Base side: request a range; chunks arrive via cb.on_bulk_data with
 * an index relative to base_offset. */
mesh_root_fetch(&mesh, node, session, base_off, base_off, len,
                MESH_BULK_CHUNK, now);

/* To fill gaps, re-FETCH the missing sub-range under the SAME session
 * and base_offset — indices stay aligned, so the bitmap just fills in: */
mesh_root_fetch(&mesh, node, session, base_off, gap_off, gap_len,
                MESH_BULK_CHUNK, now);
```

The base owns completion: which indices arrived, when to re-FETCH, and
the end-to-end CRC over the reassembled bytes. The library moves chunks
and verifies each one; it deliberately holds no session state at the
base and only a cursor at the node.

## Testing

- **Unit tests** cover the frame codec (bounds, foreign-network drop,
  MIC tamper detection), the AES-128-CMAC MIC (RFC 4493 known-answer
  vectors, per-byte tamper rejection, wrong-key rejection), the ETX
  estimator (perfect/half/decaying links, fresh restart, staleness),
  parent selection (hysteresis, loop guard, failover), flooding (dedup,
  TTL, targeted execution, re-flood re-ACK, retry exhaustion), bulk
  retrieval (CRC16 known-answer, FETCH argument validation, chunk
  streaming with single-in-flight discipline, re-FETCH session
  replacement, EOF short reads, root-side CRC gate), the RYLR998
  driver core (config sequence, hex round-trip, +RCV parsing, busy
  gating, module-reset recovery), the provisioning blob codec
  (round-trip, per-byte corruption rejection, key-hex parsing), the
  route-persistence snapshots (round-trip, per-byte corruption and
  identity rejection, root counter margins), and the GPS time source
  (calendar known-answers, PPS and sentence-timing discipline, checksum
  rejection, free-run uncertainty growth).
- **Scenario tests** run whole fleets on the simulator: 10-node tree
  formation, deterministic relay-death self-healing with hysteresis-gated
  re-optimization, a 20 %-loss 6-node line flood with exactly-once
  execution, targeted commands, telemetry topology mapping, a
  CMAC-authenticated fleet that locks out a wrong-key node, and a fully
  scheduled fleet (auto-beacons, window-gated traffic, radios asleep
  between superframes, synchronized mode switch). A 25-seed sweep of
  the scheduled scenario shows zero frames ever transmitted into a
  sleeping receiver — window gating and the listen schedule agree.
- **Bulk + replay scenarios:** a 2000-byte retrieval from the far end of
  a lossy 4-node line, with the base re-FETCHing gaps until its bitmap
  fills and the reassembled bytes matching the source exactly; and the
  full production stack (CMAC MIC + scheduler + bulk window) retrieving
  data with zero MIC drops, then rejecting a frame captured off the air
  and replayed three superframes later.
- **Reboot scenarios (§5.3):** a rebooted node restores its flash route
  cache and delivers telemetry before hearing a single beacon (and a
  node down past the staleness threshold correctly re-attaches from
  scratch instead of trusting the stale cache); a rebooted base first
  demonstrates the seq-0 wedge without persistence, then recovers the
  fleet with restored counters and keeps commands exactly-once.
- The protocol was additionally validated with a 25-seed randomized sweep
  of the healing and lossy-flood scenarios (50 runs, 0 failures); the
  committed tests use fixed seeds and are fully deterministic.

The simulator models per-link loss percentages and LoRa-ish air time but
**not** collisions or half-duplex contention — the protocol's randomized
backoffs make that acceptable for logic testing; real-air behavior gets
validated in the field (M5).

### Design envelope

Beacons and floods propagate as single-shot waves (each node rebroadcasts
once per sequence number, §5.1), so per-epoch reach decays with the
product of link success rates along a chain: five consecutive 20 %-loss
hops pass a given wave only ~33 % of the time. The design compensates by
beaconing every superframe and re-flooding commands, but deployments
should place relays so used links keep healthy margin (the control
profile's high spreading factor exists for exactly this reason, §3.3).

## Refinements adopted in design Rev B

Simulator validation of M1 found four places where Rev A's design needed
sharpening; all are now folded into
[Blast_Sensor_Mesh_Design.md](Blast_Sensor_Mesh_Design.md) Rev B:

1. **Observed-epoch ETX windows** with an in-progress-epoch grace period
   (§5.2) — upstream flood attenuation is not miscounted as local link
   loss; staleness deliberately stays on raw sequence delta (§5.4).
2. **Estimator restart on reappearance** (§5.2) — a revived relay is
   re-probed fresh instead of being haunted by its outage for a full
   window; hysteresis still prevents flapping.
3. **`HOP_ACK` frame type and per-hop seq re-stamping** (§6.4, §8) —
   per-hop reliability for tree unicast needed an explicit single-hop
   ACK; end-to-end identity lives in the payload.
4. **Two-level flood dedup** (§6.1) — re-floods need a fresh frame seq
   (or relays suppress them) with a stable in-payload command ID for
   exactly-once execution.

Rev B also records the RYLR998 module selection and its transport
constraints (§3.4) and the implemented MIC with the remaining replay
gap (§10.2).

## Roadmap

| Milestone | Contents | Status |
|---|---|---|
| **M1: Core mesh** | frame codec, radio HAL, ETX + collection tree, command flooding + ACKs, telemetry, host simulator | **done** |
| **M2: Superframe scheduler** | GPS-time-aligned windows (beacon/command/telemetry/bulk), duty-cycled listening, standby/active modes (§4) | **done** |
| **M3: Bulk transfer** | chunked FETCH sessions with single-chunk tree unicast, per-chunk CRC, base-driven resumption by byte range (§7) **done**; store-and-forward burst pipelining + selective-ACK bitmaps deferred | **done (first cut)** |
| **M4: Link security** | AES-128-CMAC MIC (RFC 4493 KAT-verified); superframe-bound replay protection (§10.2, needs GPS time to be active in the field); flash key-provisioning workflow (`provision` CLI, one UF2 per fleet) | **done** |
| **M5: Pico port** | RYLR998 driver + field-node/base examples; GPS/PPS time source (NMEA discipline, scheduler live in the field, absolute ARM windows); flash route cache + root counter persistence (§5.3) | **done** (software; real-air field validation outstanding) |
| **M6: Base-station tooling** | topology map, retrieval scheduler, fleet checklist/retry policy on the base server (the base_station example CLI is the embryo) | planned |

## Current caveats (read before fielding)

- **Replay protection needs GPS lock.** The superframe-bound MIC
  (§10.2) binds the *shared* superframe index, so it is live exactly
  when the scheduler is: nodes that lock GPS at boot get it; a node
  that can't lock free-runs in bench mode with authentication only,
  and says so in its log. Absolute ARM validity windows (§6.2) are the
  application backstop and are also GPS-gated.
- **The key lives in plaintext flash.** Provisioning (M4) keeps the key
  out of the firmware image, but the RP2040 has no secure storage or
  readout protection — physical possession of a node yields the key, so
  rotate the deployment key if a node is lost. Whoever has the key can
  inject ARM commands. The null MIC still exists for the simulator —
  never field it. An unprovisioned device falls back to the compiled
  `node_config.h` bench key and says so at boot.
- **The whole fleet needs GPS for scheduled operation.** Superframe
  alignment assumes every node's `now_us` is the same GPS-disciplined
  timebase; one node without lock degrades itself (widened listening
  via `mesh_sched_clock_uncertainty()`), but a *mixed* fleet where some
  nodes never locked (scheduler off) and some did (scheduler on) won't
  interoperate under the replay-bound MIC. Bring the fleet up together.
  Wire PPS: it is the difference between ±1 ms and ±400 ms alignment.
- **Not yet validated on real air.** Everything above is proven on the
  host simulator and bench hardware; range, collisions, and SF margins
  in real desert terrain are the remaining M5 field work.
- **Bulk retrieval is correct but unpipelined.** One chunk is in flight
  per node, so throughput is round-trip-bound rather than air-time-bound
  — fine at the design's minutes-to-hours latency budget, and the first
  thing to revisit if retrieval time becomes the constraint. Burst
  relaying with selective-ACK bitmaps (§7.3 in full) is the follow-on;
  `DATA_ACK` is reserved for it.
- **Bulk needs a bulk window.** With the scheduler on, chunks only move
  during the §4 bulk window, and neither stock mode defines one — the
  fleet must be switched to a retrieval mode first (a `mesh_root_fetch`
  in a mode without a bulk window fails immediately rather than queueing
  forever). Deciding that policy is §7.4 / M6 work.
- Over the RYLR998's hex transport, frames cap at 120 bytes (§3.4/§8
  Rev B) — irrelevant for control traffic; it is why the default bulk
  chunk is 96 bytes.

## Kanban cards

One card per line, ready to paste into a board. Design-doc sections in
parentheses.

### Ideas

- Store-and-forward burst relaying: W-chunk bursts + per-hop selective-ACK
  bitmaps for bulk throughput (§7.3 full) — the M3 follow-on
- On-wire NACK bitmaps for scattered chunk loss (vs. range re-FETCH)
- Concurrent bulk sessions per node (needs §7.4 scheduling to pay off)
- Qualify off-spec binary AT payloads on the RYLR998, or move bulk-heavy
  deployments to an SPI-attached radio (§13 Q7)
- Second radio module per node for a concurrent bulk channel (§13 Q3)
- Mobile/vehicle-mounted base station mode to shorten retrieval hops (§13 Q4)
- Optional AES-CTR payload encryption on top of the MIC (§10.2)
- On-node delta encoding + per-block compression of event windows (§7.2)
- Parallel retrieval from radio-independent branches in TDMA slots (§7.4)
- Decide RECORDING-state radio policy: fully silent vs. beacon-listen (§13 Q2)
- Per-deployment regulatory review: band, dwell-time/duty-cycle rules (§13 Q1)

### Todo

- Bulk-window scheduling policy: who opens the retrieval window and when
  (§7.4 / §13 Q8)
- Base-station tooling: topology map, retrieval scheduler, fleet
  checklist/retry policy on a real computer (M6)
- Field validation on real air: range, collisions, SF margins (M5)
- Integrate the mesh into the production logger firmware (SD, sensors,
  §9 state machine)

### In progress

- (nothing — M5 landed; M6 base-station tooling is next)

### Done

- M5 GPS time source: NMEA+PPS clock discipline, scheduler live in the
  field, absolute ARM validity windows, replay protection active
- M5 route persistence: instant post-reboot routing (§5.3) + root
  counter persistence (fixes the base-reboot seq-0 wedge)
- M4 key provisioning: flash blob + `provision` CLI on both firmwares,
  one generic UF2 per fleet, PC-generated keys (§13 Q5)
- M1 core mesh: codec, ETX collection tree, command flooding + ACKs,
  telemetry, host simulator
- M2 superframe scheduler: GPS-aligned windows, duty-cycled listening,
  fleet-synchronized mode switches
- M3 bulk retrieval (first cut): FETCH sessions, single-chunk tree
  unicast, per-chunk CRC, base-driven range resumption
- Replay protection: superframe-bound MIC, sealed at transmit time,
  ±1 verify, no wire-format change (§10.2)
- AES-128-CMAC MIC (RFC 4493, KAT-verified)
- RYLR998 AT-protocol driver (host-tested) + Pico UART glue
- Example field-node + base-station firmware, cross-built UF2s
- Base-station CLI + reference doc (examples/BASE_STATION_COMMANDS.md)
- Design doc Rev B: M1/M2 findings folded back into the spec
