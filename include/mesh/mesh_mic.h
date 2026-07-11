/*
 * mesh_mic.h — pluggable message integrity code (MIC) interface.
 *
 * Every frame carries a 4-byte MIC trailer computed over the header and
 * payload (design doc §10.2). Two implementations ship with the library:
 *
 *   - mesh_mic_cmac_*: AES-128-CMAC (RFC 4493) truncated to 4 bytes
 *     under a pre-shared network key — the real MIC. Use this in the
 *     field.
 *   - mesh_mic_null: passthrough stub (MIC = four zero bytes) for bench
 *     tests and the host simulator. Provides NO authentication.
 *
 * The interface stays pluggable so hardware-accelerated or alternative
 * algorithms can be substituted without touching protocol code.
 */
#ifndef MESH_MIC_H
#define MESH_MIC_H

#include <stddef.h>
#include <stdint.h>

#include "mesh_config.h"

typedef struct {
    /* Opaque key/context (e.g., expanded AES key). */
    void *ctx;

    /* Compute the MIC over data[0..len) into mic[MESH_MIC_SIZE].
     * Verification is compute-and-compare, so one op suffices. */
    void (*compute)(void *ctx, const uint8_t *data, size_t len,
                    uint8_t mic[MESH_MIC_SIZE]);
} mesh_mic_ops_t;

/* Passthrough stub: MIC is four zero bytes. NO authentication — bench
 * and simulator use only. */
extern const mesh_mic_ops_t mesh_mic_null;

/* ---- AES-128-CMAC MIC (design doc §10.2) ----
 *
 * Usage:
 *   static mesh_mic_cmac_t cmac;                (lives as long as mesh_t)
 *   mesh_mic_cmac_init(&cmac, network_key);
 *   cfg.mic = mesh_mic_cmac_ops(&cmac);
 *
 * The context holds the expanded key schedule; the 16-byte key itself
 * need not be kept around after init. */

typedef struct {
    uint8_t rk[176]; /* AES-128 round keys */
    uint8_t k1[16];  /* CMAC subkey: complete final block */
    uint8_t k2[16];  /* CMAC subkey: padded final block */
} mesh_mic_cmac_t;

void mesh_mic_cmac_init(mesh_mic_cmac_t *c, const uint8_t key[16]);

/* mesh_mic_ops_t-compatible compute (ctx = mesh_mic_cmac_t *). */
void mesh_mic_cmac_compute(void *ctx, const uint8_t *data, size_t len,
                           uint8_t mic[MESH_MIC_SIZE]);

/* Convenience: ops struct bound to an initialized context. */
mesh_mic_ops_t mesh_mic_cmac_ops(mesh_mic_cmac_t *c);

/* Full 16-byte CMAC tag (RFC 4493). The 4-byte frame MIC is the first
 * MESH_MIC_SIZE bytes of this. Exposed for known-answer self-tests. */
void mesh_mic_cmac_tag(const mesh_mic_cmac_t *c, const uint8_t *data,
                       size_t len, uint8_t tag[16]);

#endif /* MESH_MIC_H */
