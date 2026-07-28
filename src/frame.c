/*
 * frame.c — frame encode/parse per design doc §8.
 *
 * Layout: | net_id(2 LE) | type(1) | src(1) | dst(1) | seq(2 LE) | ttl(1) |
 *         | payload(0..MESH_MAX_PAYLOAD) | MIC(4) |
 *
 * The MIC covers header + payload plus, appended to the MIC input only
 * (never transmitted), the sender's 4-byte superframe index — the §10.2
 * replay protection. A frame recorded in one superframe and replayed in
 * another fails verification statelessly. Fleets without the scheduler
 * use a constant index of 0 on both sides, which disables the binding
 * without changing the wire format.
 */
#include "mesh_internal.h"

static void put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)(v >> 8);
}

static uint16_t get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static void put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFF);
    p[1] = (uint8_t)((v >> 8) & 0xFF);
    p[2] = (uint8_t)((v >> 16) & 0xFF);
    p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* The superframe index borrows the trailer bytes as scratch while the
 * MIC is computed, which requires the two to be the same size. */
_Static_assert(MESH_MIC_SIZE == 4, "MIC trailer must hold the sf index");

void mesh_frame_seal(const mesh_mic_ops_t *mic, uint8_t *buf, size_t len,
                     uint32_t sf_index)
{
    size_t body = len - MESH_MIC_SIZE;
    uint8_t tag[MESH_MIC_SIZE];
    put_u32(&buf[body], sf_index);
    mic->compute(mic->ctx, buf, body + 4, tag);
    memcpy(&buf[body], tag, MESH_MIC_SIZE);
}

/* One verification attempt against a candidate superframe index.
 * `scratch` holds a copy of the frame body with 4 spare bytes. */
static bool mic_try(const mesh_mic_ops_t *mic, uint8_t *scratch,
                    size_t body, uint32_t sf, const uint8_t *wire_mic)
{
    uint8_t tag[MESH_MIC_SIZE];
    put_u32(&scratch[body], sf);
    mic->compute(mic->ctx, scratch, body + 4, tag);
    return memcmp(tag, wire_mic, MESH_MIC_SIZE) == 0;
}

size_t mesh_frame_build(uint16_t net_id, const mesh_mic_ops_t *mic,
                        uint8_t buf[MESH_MAX_FRAME],
                        uint8_t type, uint8_t src, uint8_t dst,
                        uint16_t seq, uint8_t ttl,
                        const uint8_t *payload, uint8_t plen)
{
    if (plen > MESH_MAX_PAYLOAD)
        return 0;
    if (plen > 0 && payload == NULL)
        return 0;

    put_u16(&buf[MESH_OFF_NET], net_id);
    buf[MESH_OFF_TYPE] = type;
    buf[MESH_OFF_SRC] = src;
    buf[MESH_OFF_DST] = dst;
    put_u16(&buf[MESH_OFF_SEQ], seq);
    buf[MESH_OFF_TTL] = ttl;
    if (plen > 0)
        memcpy(&buf[MESH_OFF_PAYLOAD], payload, plen);

    size_t body = (size_t)MESH_HDR_SIZE + plen;
    /* Sealed for superframe 0 — the final value on scheduler-off fleets;
     * with the scheduler enabled the TX queue re-seals at fire time with
     * the then-current index (§10.2). */
    mesh_frame_seal(mic, buf, body + MESH_MIC_SIZE, 0);
    return body + MESH_MIC_SIZE;
}

mesh_frame_rc_t mesh_frame_parse_sf(uint16_t net_id,
                                    const mesh_mic_ops_t *mic,
                                    const uint8_t *buf, size_t len,
                                    uint32_t sf_center, uint32_t sf_tol,
                                    mesh_frame_t *out)
{
    if (len < MESH_OVERHEAD || len > MESH_MAX_FRAME)
        return MESH_FRAME_E_LEN;

    /* Network ID is checked before the MIC so foreign-deployment frames
     * are cheap to drop (§3.2); the MIC gate then rejects forgeries. */
    if (get_u16(&buf[MESH_OFF_NET]) != net_id)
        return MESH_FRAME_E_NET;

    /* Try the receiver's own superframe index first (the accept path is
     * one MIC compute), then ±d out to the tolerance. center-d before
     * center+d: the sender sealed before we parsed, so a genuine
     * boundary straddle usually means the frame is from the *previous*
     * index. A replay outside the tolerance fails every candidate. */
    size_t body = len - MESH_MIC_SIZE;
    uint8_t scratch[MESH_MAX_FRAME];
    memcpy(scratch, buf, body);
    bool ok = mic_try(mic, scratch, body, sf_center, &buf[body]);
    for (uint32_t d = 1; d <= sf_tol && !ok; d++) {
        if (sf_center >= d)
            ok = mic_try(mic, scratch, body, sf_center - d, &buf[body]);
        if (!ok)
            ok = mic_try(mic, scratch, body, sf_center + d, &buf[body]);
    }
    if (!ok)
        return MESH_FRAME_E_MIC;

    out->type = buf[MESH_OFF_TYPE];
    out->src = buf[MESH_OFF_SRC];
    out->dst = buf[MESH_OFF_DST];
    out->seq = get_u16(&buf[MESH_OFF_SEQ]);
    out->ttl = buf[MESH_OFF_TTL];
    out->plen = (uint8_t)(body - MESH_HDR_SIZE);
    out->payload = &buf[MESH_OFF_PAYLOAD];
    return MESH_FRAME_OK;
}

/* Scheduler-off parse: index 0 exactly, matching mesh_frame_build. */
mesh_frame_rc_t mesh_frame_parse(uint16_t net_id, const mesh_mic_ops_t *mic,
                                 const uint8_t *buf, size_t len,
                                 mesh_frame_t *out)
{
    return mesh_frame_parse_sf(net_id, mic, buf, len, 0, 0, out);
}
