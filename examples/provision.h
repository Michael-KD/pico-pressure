/*
 * provision.h — per-device provisioning block (design doc §10.2, §13 Q5).
 *
 * The M4 key-management answer: identity and the network key live in a
 * small self-checking blob written to flash at provisioning time, not in
 * the compiled binary. One generic UF2 then serves every device; a node
 * is personalized over USB serial with the `provision` CLI command, and
 * key rotation is re-provisioning (no rebuild, no reflash).
 *
 * This file is the platform-free codec — encode/decode/validate — so it
 * is unit-tested on the host exactly like the RYLR998 driver core. The
 * flash read/write itself is Pico glue (flash_store.[ch]); wiring the
 * result into mesh_init is the firmware's job.
 *
 * Threat model note: the blob is plaintext in external flash and the
 * RP2040 has no secure-storage or readout protection, so physical
 * possession of a node yields the key. That matches §10.2's threat
 * model (over-the-air injection on a private range, not captured
 * hardware) — rotate the deployment key if a node is lost.
 */
#ifndef PROVISION_H
#define PROVISION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Fixed blob size: magic(4) node_id(1) flags(1) net_id(2 LE) key(16)
 * crc16(2 LE) + 0xFF pad. Kept well under one flash page. */
#define PROV_BLOB_SIZE 32

typedef struct {
    uint8_t  node_id; /* 0 = base station, 1..254 = field node */
    uint16_t net_id;  /* mesh network ID (in-frame, §3.2) */
    uint8_t  key[16]; /* AES-128-CMAC network key (§10.2) */
} prov_t;

/* Serialize `p` into a PROV_BLOB_SIZE blob (pad bytes 0xFF so the image
 * can be compared against erased flash). */
void prov_encode(const prov_t *p, uint8_t blob[PROV_BLOB_SIZE]);

/* Validate magic + CRC + field ranges; false leaves *out untouched
 * (erased or corrupt flash reads as "unprovisioned"). */
bool prov_decode(const uint8_t blob[PROV_BLOB_SIZE], prov_t *out);

/* Parse exactly 32 hex chars into a 16-byte key. */
bool prov_parse_key_hex(const char *hex, uint8_t key[16]);

/* Format a key as 32 lowercase hex chars + NUL. */
void prov_format_key_hex(const uint8_t key[16], char out[33]);

#ifdef __cplusplus
}
#endif

#endif /* PROVISION_H */
