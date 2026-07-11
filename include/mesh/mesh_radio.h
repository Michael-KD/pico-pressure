/*
 * mesh_radio.h — the radio hardware abstraction boundary.
 *
 * The mesh library never touches a radio directly; it calls these ops.
 * A LoRa driver (SX127x/SX126x), a different sub-GHz radio, or the host
 * simulator all plug in here, which is what keeps the protocol layer
 * radio-agnostic.
 *
 * Receive path: the driver delivers frames by calling mesh_rx() (see
 * mesh.h) from the main loop — not from an ISR.
 */
#ifndef MESH_RADIO_H
#define MESH_RADIO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Two logical channels per design doc §3.3: a slow, long-range control
 * profile and a faster bulk profile for data retrieval (bulk is unused
 * until the bulk-transfer milestone but is part of the HAL contract). */
typedef enum {
    MESH_PROFILE_CONTROL = 0,
    MESH_PROFILE_BULK = 1,
} mesh_profile_t;

typedef struct {
    /* Opaque driver context, passed back to every op. */
    void *ctx;

    /* Transmit one frame. May block for the (short) time it takes to hand
     * the frame to the radio, but must not wait for air time to complete.
     * Returns false if the radio could not accept the frame (busy/fault);
     * the library counts the failure and, for reliable frames, retries. */
    bool (*transmit)(void *ctx, const uint8_t *frame, size_t len,
                     mesh_profile_t profile);

    /* Enable or disable receive on the given profile. Used for duty
     * cycling (superframe scheduling milestone); the core library calls
     * set_listen(true, CONTROL) once at init. May be NULL if the radio
     * is always listening. */
    void (*set_listen)(void *ctx, bool enable, mesh_profile_t profile);
} mesh_radio_ops_t;

#endif /* MESH_RADIO_H */
