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
solar power (see [project_background.txt](project_background.txt)). Nodes
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

The **core mesh** (milestone M1): a portable, radio-agnostic C11 library
implementing the design doc's collection-tree architecture —

- **Frame codec** (§8): 12-byte overhead (8 B header + 4 B MIC trailer),
  network-ID gate, pluggable message-integrity hook.
- **Link authentication** (§10.2): AES-128-CMAC MIC (RFC 4493, verified
  against the RFC test vectors) under a pre-shared network key; frames
  failing the MIC are dropped before parsing. A null stub remains for
  bench/simulator use. Replay protection is still open — see §10.2 Rev B.
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
project_background.txt        original project description
CMakeLists.txt                library + host tests (C11, -Wall -Wextra -Werror)
include/mesh/
  mesh.h                      public API: mesh_t, init/rx/poll, send, callbacks
  mesh_config.h               tunables (§11 defaults), all overridable via -D
  mesh_radio.h                radio HAL vtable — the radio-agnostic boundary
  mesh_mic.h                  MIC interface: AES-128-CMAC + null stub
src/
  frame.c                     frame build/parse (§8)
  neighbor.c                  neighbor table, observed-epoch ETX windows
  tree.c                      parent selection, hysteresis, loop guard, poisoning
  beacon.c                    beacon flood, best-beacon rebroadcast (§5.1)
  flood.c                     command flooding, dedup, TTL (§6)
  command.c                   execution + ACK, per-hop-reliable tree unicast
  telemetry.c                 status reports with routing context (§5.3)
  txqueue.c                   due-time TX queue: backoffs, retries, exhaustion
  mic_cmac.c                  AES-128-CMAC MIC (RFC 4493), portable software AES
  mic_null.c                  passthrough MIC stub (bench/simulator only)
  mesh.c                      init, RX dispatch, poll, introspection
  mesh_internal.h             module interfaces (tests may include this)
examples/                     standalone pico-sdk project (see below)
  main.c                      field-node firmware (Pico + RYLR998)
  base_station.c              base-gateway firmware with USB-serial CLI
  node_config.h               per-device/deployment provisioning (ID, key, pins)
  rylr998_pico.[ch]           Pico UART/IRQ glue for the radio driver
  radios/RYLR998/             portable AT-protocol driver core + datasheets
tests/
  unit/                       frame / MIC / neighbor / tree / flood / driver tests
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
ctest --test-dir build            # 5 suites, all green
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
the others — **set a unique `NODE_ID` in `examples/node_config.h` per
field device** (and change the network key for anything beyond the
bench). Wire each RYLR998 to 3V3/GND, GP4→RXD, GP5←TXD (and GP6→NRST,
optional). The base station gives you a CLI on USB serial:

```
> arm 30        flood ARM (30 min) — ACKs print as they arrive
> status        nodes answer with telemetry
> top           topology map / ACK checklist
> resend        re-flood the last command for stragglers
> stats         mesh + radio counters
```

All frames are authenticated with AES-128-CMAC under the key in
`node_config.h`. Field nodes blink fast while searching and slow once
attached, and log everything on their own USB port.

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
mesh_root_beacon(&mesh, now);   /* once per superframe — app schedules */

uint16_t id = mesh_root_command(&mesh, MESH_ID_BROADCAST,
                                arm_payload, n, 0, now);
/* collect ACKs via cb.on_command_ack; for stragglers, re-flood the SAME
 * command id — executed nodes re-ACK without re-executing: */
mesh_root_command(&mesh, MESH_ID_BROADCAST, arm_payload, n, id, now);
```

Telemetry arrives via `cb.on_telemetry` with each node's parent and
top-3 neighbors — the base's topology map (§5.3) falls straight out.

## Testing

- **Unit tests** cover the frame codec (bounds, foreign-network drop,
  MIC tamper detection), the AES-128-CMAC MIC (RFC 4493 known-answer
  vectors, per-byte tamper rejection, wrong-key rejection), the ETX
  estimator (perfect/half/decaying links, fresh restart, staleness),
  parent selection (hysteresis, loop guard, failover), flooding (dedup,
  TTL, targeted execution, re-flood re-ACK, retry exhaustion), and the
  RYLR998 driver core (config sequence, hex round-trip, +RCV parsing,
  busy gating, module-reset recovery).
- **Scenario tests** run whole fleets on the simulator: 10-node tree
  formation, deterministic relay-death self-healing with hysteresis-gated
  re-optimization, a 20 %-loss 6-node line flood with exactly-once
  execution, targeted commands, telemetry topology mapping, and a
  CMAC-authenticated fleet that locks out a wrong-key node.
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
| **M2: Superframe scheduler** | GPS-time-aligned windows (beacon/command/telemetry/bulk), duty-cycled listening, standby/active modes (§4) | planned |
| **M3: Bulk transfer** | chunked FETCH sessions, store-and-forward burst relaying, selective-ACK bitmaps, resumption by session id (§7) | planned |
| **M4: Link security** | AES-128-CMAC MIC **done** (RFC 4493 KAT-verified, used by tests/demo/examples); replay protection + key provisioning workflow remain (§10.2 Rev B) | **partial** |
| **M5: Pico port** | RYLR998 driver + field-node/base examples **done** (cross-built with pico-sdk); GPS/PPS time source and flash route cache (§5.3) remain | **partial** |
| **M6: Base-station tooling** | topology map, retrieval scheduler, fleet checklist/retry policy on the base server (the base_station example CLI is the embryo) | planned |

## Current caveats (read before fielding)

- **Frames are authenticated but not replay-protected.** The
  AES-128-CMAC MIC stops forgery, but a recorded valid frame can be
  retransmitted later; the exec-dedup cache and ARM validity windows
  blunt this without closing it (§10.2 Rev B). Fix lands with the M2
  wire-format revision.
- **The examples compile the network key in** (`node_config.h`) — fine
  for the bench, not a production provisioning flow. Change the key per
  deployment; whoever has it can inject ARM commands. The null MIC still
  exists for the simulator — never field it.
- **Example firmware clocks from `time_us_64()`** (µs since boot), not
  GPS. The M1 mesh only needs monotonic local time, but real ARM
  validity windows and superframe scheduling need the GPS/PPS time
  source (M2/M5).
- No superframe scheduling yet: the application decides when the root
  beacons and how nodes duty-cycle; the library assumes the radio
  listens whenever the app lets it (M2).
- Neighbor/route state is RAM-only; the §5.3 flash cache for instant
  post-reboot route resumption comes with the Pico port (M5).
- `DATA`/`DATA_ACK`/`SESSION` frame types are reserved but unimplemented
  (M3); over the RYLR998's hex transport, frames cap at 120 bytes
  (§3.4/§8 Rev B) — irrelevant for control traffic, relevant for M3
  chunk sizing.
