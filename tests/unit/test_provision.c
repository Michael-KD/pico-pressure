/*
 * test_provision.c — provisioning-blob codec (examples/provision.c).
 *
 * The blob carries the deployment's load-bearing secret (§10.2), so the
 * codec must fail closed: anything but a byte-exact valid blob reads as
 * "unprovisioned", never as a garbled identity.
 */
#include <string.h>

#include "provision.h"
#include "test_util.h"

static const prov_t sample = {
    .node_id = 7,
    .net_id = 0xD5E7,
    .key = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99,
             0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff },
};

static void test_roundtrip(void)
{
    uint8_t blob[PROV_BLOB_SIZE];
    prov_encode(&sample, blob);

    prov_t back;
    memset(&back, 0, sizeof(back));
    T_CHECK(prov_decode(blob, &back));
    T_CHECK_EQ(back.node_id, sample.node_id);
    T_CHECK_EQ(back.net_id, sample.net_id);
    T_CHECK(memcmp(back.key, sample.key, 16) == 0);

    /* Encoding is deterministic (flash_store's compare-skip relies on
     * byte-identical re-encodes). */
    uint8_t blob2[PROV_BLOB_SIZE];
    prov_encode(&sample, blob2);
    T_CHECK(memcmp(blob, blob2, PROV_BLOB_SIZE) == 0);

    /* Node 0 (base station) is a valid identity. */
    prov_t base = sample;
    base.node_id = 0;
    prov_encode(&base, blob);
    T_CHECK(prov_decode(blob, &back));
    T_CHECK_EQ(back.node_id, 0);
}

static void test_rejects_corruption(void)
{
    uint8_t blob[PROV_BLOB_SIZE];
    prov_t out;

    /* Erased flash (all 0xFF) is the canonical "unprovisioned". */
    memset(blob, 0xFF, sizeof(blob));
    T_CHECK(!prov_decode(blob, &out));

    /* Every corrupted byte of the covered region must be caught. */
    for (unsigned i = 0; i < 26; i++) {
        prov_encode(&sample, blob);
        blob[i] ^= 0x40;
        T_CHECK(!prov_decode(blob, &out));
    }

    /* Pad bytes are outside the CRC: flipping them must NOT invalidate
     * the blob (future fields may live there). */
    prov_encode(&sample, blob);
    blob[PROV_BLOB_SIZE - 1] ^= 0xFF;
    T_CHECK(prov_decode(blob, &out));

    /* node_id 255 is broadcast, never an identity. */
    prov_t bad = sample;
    bad.node_id = 255;
    prov_encode(&bad, blob);
    T_CHECK(!prov_decode(blob, &out));
}

static void test_key_hex(void)
{
    uint8_t key[16];
    T_CHECK(prov_parse_key_hex("00112233445566778899aabbccddeeff", key));
    T_CHECK(memcmp(key, sample.key, 16) == 0);

    /* Mixed case accepted. */
    T_CHECK(prov_parse_key_hex("00112233445566778899AABBCCDDEEFF", key));
    T_CHECK(memcmp(key, sample.key, 16) == 0);

    /* Wrong length and non-hex rejected. */
    T_CHECK(!prov_parse_key_hex("00112233445566778899aabbccddee", key));
    T_CHECK(!prov_parse_key_hex("00112233445566778899aabbccddeeff00",
                                key));
    T_CHECK(!prov_parse_key_hex("0011223344556677g899aabbccddeeff", key));

    char hex[33];
    prov_format_key_hex(sample.key, hex);
    T_CHECK(strcmp(hex, "00112233445566778899aabbccddeeff") == 0);
}

int main(void)
{
    T_RUN(test_roundtrip);
    T_RUN(test_rejects_corruption);
    T_RUN(test_key_hex);
    return t_summary();
}
