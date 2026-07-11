# Base Station CLI — Command Reference

The base-station firmware ([base_station.c](base_station.c)) runs the
mesh root (node 0) and gives you a command line over **USB serial**.
Plug the base Pico into a PC and open its COM port with any terminal
program (PuTTY, TeraTerm, `screen`, the Arduino serial monitor — any
baud rate works, it's USB CDC). You'll get a `>` prompt; commands are
plain text, one per line.

While you work, the base prints events as they arrive:

```
ACK: node 3 executed cmd 17          <- a command ACK came up the tree
TELEM: node 2 (parent 1, standby, 4.9V, 31C)
```

## Commands

| Command | What it does |
|---|---|
| `arm [minutes]` | Floods an ARM command to the whole fleet. Nodes arm for the given duration (default **60** minutes) and then disarm themselves. Watch the ACK lines to see who got it. |
| `disarm` | Floods DISARM. Armed nodes return to standby immediately. |
| `status` | Floods STATUS. Every node answers with a telemetry report (state, battery, temperature) within a second or two — the quickest way to roll-call the fleet. |
| `resend` | Re-floods the **last** command with the *same* command ID. Nodes that already executed it just re-ACK (they never execute twice); nodes that missed it execute now. Use this to chase stragglers until the checklist is complete. |
| `top` | Prints the topology map / ACK checklist (see below). |
| `stats` | Prints mesh and radio driver counters (frames sent/received, MIC drops, retries, module errors) — the first stop when something looks wrong. |

Anything else prints a one-line usage reminder.

## Reading `top`

```
node | parent | state   | batt | temp | acked cmd 17
-----+--------+---------+------+------+----------
   1 |     0  | ARMED   | 5.0V |  29C | yes
     |        |   sees 2 at 1.15 ETX
   2 |     1  | ARMED   | 4.9V |  31C | yes
   3 |     2  | standby | 4.8V |  30C | NO
```

- **parent** — who the node routes through; chains of parents are the
  collection tree. `top` is built from telemetry, so run `status` first
  if it looks stale.
- **sees X at N ETX** — the node's other neighbors and link quality
  (1.00 = perfect; higher = lossier). Useful for spotting nodes with no
  backup path.
- **acked cmd** — the checklist for the most recent command. A `NO`
  after an `arm` means that node has *not confirmed arming* — `resend`
  until every row says `yes`, or go find out why it can't hear the
  mesh.

## How commands travel (30-second version)

A command is flooded: every node rebroadcasts it once, so it reaches
nodes the base can't hear directly. Each node executes it **exactly
once** (repeat floods are recognized by command ID) and sends an ACK
back hop-by-hop through the tree. The base never assumes delivery —
that's what the checklist is for. All frames are authenticated with
AES-128-CMAC under the key in [node_config.h](node_config.h); a node
with the wrong key can neither read nor issue commands.

## Command payloads (for extending the set)

The mesh delivers opaque payloads; the opcode convention between these
two example firmwares is one ASCII byte + arguments:

| Opcode | Payload | Field-node behavior ([main.c](main.c)) |
|---|---|---|
| `'A'` | 2-byte minutes, little-endian | Arm for that duration. (Production: absolute GPS validity window, design doc §6.) |
| `'D'` | — | Disarm. |
| `'S'` | — | Send telemetry within ~1 s. |
| other | — | Rejected — **no ACK**, so the base's checklist shows the gap. |

To add a command: pick an opcode, build the payload in a new branch of
`run_cli_line()` in base_station.c, handle it in `on_command()` in
main.c, and **return `true` only when the node really executed it** —
that return value is what drives the ACK and the checklist.
