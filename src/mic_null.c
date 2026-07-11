/*
 * mic_null.c — passthrough MIC stub (no authentication).
 * See mesh_mic.h for the security caveat. Replaced by AES-128-CMAC in
 * the link-security milestone.
 */
#include <string.h>

#include "mesh/mesh_mic.h"

static void null_compute(void *ctx, const uint8_t *data, size_t len,
                         uint8_t mic[MESH_MIC_SIZE])
{
    (void)ctx;
    (void)data;
    (void)len;
    memset(mic, 0, MESH_MIC_SIZE);
}

const mesh_mic_ops_t mesh_mic_null = {
    .ctx = NULL,
    .compute = null_compute,
};
