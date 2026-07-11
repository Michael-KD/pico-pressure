/*
 * test_mic.c — AES-128-CMAC known-answer tests (RFC 4493 §4) and
 * frame-level authentication behavior.
 */
#include <string.h>

#include "mesh_internal.h"
#include "test_util.h"

/* RFC 4493 test key and message. */
static const uint8_t rfc_key[16] = {
    0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
    0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};
static const uint8_t rfc_msg[64] = {
    0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
    0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
    0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
    0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
    0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
    0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
    0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
    0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
};

static void check_vector(const mesh_mic_cmac_t *c, size_t len,
                         const uint8_t expect[16])
{
    uint8_t tag[16];
    mesh_mic_cmac_tag(c, rfc_msg, len, tag);
    T_CHECK(memcmp(tag, expect, 16) == 0);
}

static void test_rfc4493_vectors(void)
{
    mesh_mic_cmac_t c;
    mesh_mic_cmac_init(&c, rfc_key);

    /* Subkeys from RFC 4493 §4. */
    static const uint8_t k1[16] = {
        0xfb, 0xee, 0xd6, 0x18, 0x35, 0x71, 0x33, 0x66,
        0x7c, 0x85, 0xe0, 0x8f, 0x72, 0x36, 0xa8, 0xde,
    };
    static const uint8_t k2[16] = {
        0xf7, 0xdd, 0xac, 0x30, 0x6a, 0xe2, 0x66, 0xcc,
        0xf9, 0x0b, 0xc1, 0x1e, 0xe4, 0x6d, 0x51, 0x3b,
    };
    T_CHECK(memcmp(c.k1, k1, 16) == 0);
    T_CHECK(memcmp(c.k2, k2, 16) == 0);

    static const uint8_t t0[16] = {
        0xbb, 0x1d, 0x69, 0x29, 0xe9, 0x59, 0x37, 0x28,
        0x7f, 0xa3, 0x7d, 0x12, 0x9b, 0x75, 0x67, 0x46,
    };
    static const uint8_t t16[16] = {
        0x07, 0x0a, 0x16, 0xb4, 0x6b, 0x4d, 0x41, 0x44,
        0xf7, 0x9b, 0xdd, 0x9d, 0xd0, 0x4a, 0x28, 0x7c,
    };
    static const uint8_t t40[16] = {
        0xdf, 0xa6, 0x67, 0x47, 0xde, 0x9a, 0xe6, 0x30,
        0x30, 0xca, 0x32, 0x61, 0x14, 0x97, 0xc8, 0x27,
    };
    static const uint8_t t64[16] = {
        0x51, 0xf0, 0xbe, 0xbf, 0x7e, 0x3b, 0x9d, 0x92,
        0xfc, 0x49, 0x74, 0x17, 0x79, 0x36, 0x3c, 0xfe,
    };
    check_vector(&c, 0, t0);
    check_vector(&c, 16, t16);
    check_vector(&c, 40, t40);
    check_vector(&c, 64, t64);

    /* The 4-byte frame MIC is the leading truncation of the tag. */
    uint8_t mic[MESH_MIC_SIZE];
    mesh_mic_cmac_compute(&c, rfc_msg, 40, mic);
    T_CHECK(memcmp(mic, t40, MESH_MIC_SIZE) == 0);
}

static void test_frame_auth(void)
{
    mesh_mic_cmac_t good, evil;
    static const uint8_t key2[16] = { 0x42 };
    mesh_mic_cmac_init(&good, rfc_key);
    mesh_mic_cmac_init(&evil, key2);
    mesh_mic_ops_t ops = mesh_mic_cmac_ops(&good);
    mesh_mic_ops_t evil_ops = mesh_mic_cmac_ops(&evil);

    uint8_t payload[5] = { 'A', 'R', 'M', '!', 0 };
    uint8_t buf[MESH_MAX_FRAME];
    size_t len = mesh_frame_build(0xD5E7, &ops, buf, MESH_FT_CMD,
                                  0, MESH_ID_BROADCAST, 7, 8,
                                  payload, sizeof(payload));
    T_CHECK_EQ(len, MESH_OVERHEAD + sizeof(payload));

    mesh_frame_t fr;
    T_CHECK_EQ(mesh_frame_parse(0xD5E7, &ops, buf, len, &fr),
               MESH_FRAME_OK);

    /* Tampering with any single byte must invalidate the frame (network
     * ID bytes fail with E_NET, everything else with E_MIC). */
    bool all_rejected = true;
    for (size_t i = 0; i < len; i++) {
        buf[i] ^= 0x01;
        if (mesh_frame_parse(0xD5E7, &ops, buf, len, &fr) == MESH_FRAME_OK)
            all_rejected = false;
        buf[i] ^= 0x01;
    }
    T_CHECK(all_rejected);

    /* A frame forged under the wrong key is dropped... */
    uint8_t forged[MESH_MAX_FRAME];
    size_t flen = mesh_frame_build(0xD5E7, &evil_ops, forged, MESH_FT_CMD,
                                   0, MESH_ID_BROADCAST, 7, 8,
                                   payload, sizeof(payload));
    T_CHECK_EQ(mesh_frame_parse(0xD5E7, &ops, forged, flen, &fr),
               MESH_FRAME_E_MIC);

    /* ...and so is a null-MIC frame arriving at an authenticated node. */
    size_t nlen = mesh_frame_build(0xD5E7, &mesh_mic_null, forged,
                                   MESH_FT_CMD, 0, MESH_ID_BROADCAST,
                                   7, 8, payload, sizeof(payload));
    T_CHECK_EQ(mesh_frame_parse(0xD5E7, &ops, forged, nlen, &fr),
               MESH_FRAME_E_MIC);
}

int main(void)
{
    T_RUN(test_rfc4493_vectors);
    T_RUN(test_frame_auth);
    return t_summary();
}
