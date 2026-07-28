/* Unit tests: frame codec (build/parse, bounds, net-ID gate, MIC gate). */
#include <string.h>

#include "mesh_internal.h"
#include "test_util.h"

#define NET 0xBEEF

/* A trivial real MIC (XOR-fold) to prove tampering is caught — the null
 * stub cannot detect payload corruption by design. */
static void xor_mic(void *ctx, const uint8_t *data, size_t len,
                    uint8_t mic[MESH_MIC_SIZE])
{
    (void)ctx;
    memset(mic, 0x5A, MESH_MIC_SIZE);
    for (size_t i = 0; i < len; i++)
        mic[i % MESH_MIC_SIZE] ^= data[i];
}
static const mesh_mic_ops_t mic_xor = { .ctx = NULL, .compute = xor_mic };

static void test_roundtrip(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    uint8_t payload[5] = { 1, 2, 3, 4, 5 };

    size_t len = mesh_frame_build(NET, &mesh_mic_null, buf, MESH_FT_CMD,
                                  7, 9, 0x1234, 6, payload,
                                  sizeof(payload));
    T_CHECK_EQ(len, MESH_OVERHEAD + sizeof(payload));

    mesh_frame_t f;
    T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf, len, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(f.type, MESH_FT_CMD);
    T_CHECK_EQ(f.src, 7);
    T_CHECK_EQ(f.dst, 9);
    T_CHECK_EQ(f.seq, 0x1234);
    T_CHECK_EQ(f.ttl, 6);
    T_CHECK_EQ(f.plen, sizeof(payload));
    T_CHECK(memcmp(f.payload, payload, sizeof(payload)) == 0);
}

static void test_empty_and_max_payload(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    mesh_frame_t f;

    size_t len = mesh_frame_build(NET, &mesh_mic_null, buf,
                                  MESH_FT_BEACON, 1, 255, 1, 0, NULL, 0);
    T_CHECK_EQ(len, MESH_OVERHEAD);
    T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf, len, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(f.plen, 0);

    uint8_t big[MESH_MAX_PAYLOAD];
    memset(big, 0xAB, sizeof(big));
    len = mesh_frame_build(NET, &mesh_mic_null, buf, MESH_FT_DATA, 1, 0,
                           2, 0, big, sizeof(big));
    T_CHECK_EQ(len, MESH_OVERHEAD + MESH_MAX_PAYLOAD);
    T_CHECK(len <= MESH_MAX_FRAME);
    T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf, len, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(f.plen, MESH_MAX_PAYLOAD);
    T_CHECK(memcmp(f.payload, big, sizeof(big)) == 0);
}

static void test_build_rejects_oversize(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    uint8_t big[MESH_MAX_PAYLOAD + 1] = { 0 };
    T_CHECK_EQ(mesh_frame_build(NET, &mesh_mic_null, buf, MESH_FT_DATA,
                                1, 0, 2, 0, big, sizeof(big)),
               0);
    T_CHECK_EQ(mesh_frame_build(NET, &mesh_mic_null, buf, MESH_FT_DATA,
                                1, 0, 2, 0, NULL, 4),
               0);
}

static void test_parse_bounds(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    mesh_frame_t f;
    size_t len = mesh_frame_build(NET, &mesh_mic_null, buf, MESH_FT_CMD,
                                  1, 2, 3, 4, (const uint8_t *)"hi", 2);

    /* Every truncation shorter than the minimum frame is E_LEN. */
    for (size_t l = 0; l < MESH_OVERHEAD; l++)
        T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf, l, &f),
                   MESH_FRAME_E_LEN);
    T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf,
                                MESH_MAX_FRAME + 1, &f),
               MESH_FRAME_E_LEN);
    (void)len;
}

static void test_foreign_network_dropped(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    mesh_frame_t f;
    size_t len = mesh_frame_build(0x1111, &mesh_mic_null, buf,
                                  MESH_FT_CMD, 1, 2, 3, 4, NULL, 0);
    T_CHECK_EQ(mesh_frame_parse(NET, &mesh_mic_null, buf, len, &f),
               MESH_FRAME_E_NET);
}

static void test_mic_gate(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    mesh_frame_t f;
    uint8_t payload[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
    size_t len = mesh_frame_build(NET, &mic_xor, buf, MESH_FT_CMD, 1, 2,
                                  3, 4, payload, sizeof(payload));
    T_CHECK_EQ(mesh_frame_parse(NET, &mic_xor, buf, len, &f),
               MESH_FRAME_OK);

    /* Corrupt one payload byte: XOR MIC must reject it. */
    buf[MESH_OFF_PAYLOAD] ^= 0x01;
    T_CHECK_EQ(mesh_frame_parse(NET, &mic_xor, buf, len, &f),
               MESH_FRAME_E_MIC);
    buf[MESH_OFF_PAYLOAD] ^= 0x01;

    /* Corrupt the MIC trailer itself. */
    buf[len - 1] ^= 0xFF;
    T_CHECK_EQ(mesh_frame_parse(NET, &mic_xor, buf, len, &f),
               MESH_FRAME_E_MIC);
}

/* §10.2 replay protection: the MIC binds the transmit-time superframe
 * index. Within the ±tolerance a boundary-straddling frame verifies;
 * outside it — a replay from an earlier superframe — it is rejected. */
static void test_sf_binding(void)
{
    uint8_t buf[MESH_MAX_FRAME];
    mesh_frame_t f;
    uint8_t payload[3] = { 9, 8, 7 };
    size_t len = mesh_frame_build(NET, &mic_xor, buf, MESH_FT_TELEM, 3, 2,
                                  77, 5, payload, sizeof(payload));

    /* Fire-time seal for superframe 42: accepted at 42 and 42±1... */
    mesh_frame_seal(&mic_xor, buf, len, 42);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 42, 1, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(f.seq, 77);
    T_CHECK(memcmp(f.payload, payload, sizeof(payload)) == 0);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 43, 1, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 41, 1, &f),
               MESH_FRAME_OK);

    /* ...but is a rejected replay two superframes away. */
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 44, 1, &f),
               MESH_FRAME_E_MIC);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 40, 1, &f),
               MESH_FRAME_E_MIC);

    /* Zero tolerance accepts only the exact index. */
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 42, 0, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 43, 0, &f),
               MESH_FRAME_E_MIC);

    /* Underflow guard at the epoch start: center 0 must not wrap. */
    mesh_frame_seal(&mic_xor, buf, len, 1);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 0, 1, &f),
               MESH_FRAME_OK);

    /* Build seals with index 0 — exactly what the scheduler-off parse
     * wrapper verifies against; a scheduler-on receiver at a distant
     * index rejects the same bytes. */
    len = mesh_frame_build(NET, &mic_xor, buf, MESH_FT_CMD, 1, 2, 3, 4,
                           NULL, 0);
    T_CHECK_EQ(mesh_frame_parse(NET, &mic_xor, buf, len, &f),
               MESH_FRAME_OK);
    T_CHECK_EQ(mesh_frame_parse_sf(NET, &mic_xor, buf, len, 7, 1, &f),
               MESH_FRAME_E_MIC);
}

int main(void)
{
    T_RUN(test_roundtrip);
    T_RUN(test_empty_and_max_payload);
    T_RUN(test_build_rejects_oversize);
    T_RUN(test_parse_bounds);
    T_RUN(test_foreign_network_dropped);
    T_RUN(test_mic_gate);
    T_RUN(test_sf_binding);
    return t_summary();
}
