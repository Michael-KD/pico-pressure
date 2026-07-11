/*
 * frame.c — frame encode/parse per design doc §8.
 *
 * Layout: | net_id(2 LE) | type(1) | src(1) | dst(1) | seq(2 LE) | ttl(1) |
 *         | payload(0..MESH_MAX_PAYLOAD) | MIC(4) |
 * The MIC covers everything before it (header + payload).
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
    mic->compute(mic->ctx, buf, body, &buf[body]);
    return body + MESH_MIC_SIZE;
}

mesh_frame_rc_t mesh_frame_parse(uint16_t net_id, const mesh_mic_ops_t *mic,
                                 const uint8_t *buf, size_t len,
                                 mesh_frame_t *out)
{
    if (len < MESH_OVERHEAD || len > MESH_MAX_FRAME)
        return MESH_FRAME_E_LEN;

    /* Network ID is checked before the MIC so foreign-deployment frames
     * are cheap to drop (§3.2); the MIC gate then rejects forgeries. */
    if (get_u16(&buf[MESH_OFF_NET]) != net_id)
        return MESH_FRAME_E_NET;

    size_t body = len - MESH_MIC_SIZE;
    uint8_t mic_calc[MESH_MIC_SIZE];
    mic->compute(mic->ctx, buf, body, mic_calc);
    if (memcmp(mic_calc, &buf[body], MESH_MIC_SIZE) != 0)
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
