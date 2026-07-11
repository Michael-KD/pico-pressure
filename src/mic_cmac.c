/*
 * mic_cmac.c — AES-128-CMAC message integrity code (design doc §10.2).
 *
 * CMAC per RFC 4493 over a portable byte-oriented AES-128 (encrypt-only;
 * CMAC never decrypts). No allocation, no platform dependencies — runs
 * unchanged on the Pico's Cortex-M0+ and on the host. Cost per frame is
 * one AES block per 16 payload bytes (a beacon is 2 blocks), negligible
 * next to LoRa air time.
 *
 * Threat-model note: the S-box lookups are not cache-timing constant.
 * The §10.2 adversary is on the radio side and cannot observe cache
 * timing on the node, so this is acceptable here. The 4-byte truncation
 * (frame format §8) bounds blind-forgery odds at 2^-32 per attempt, and
 * LoRa air time rate-limits attempts to a few per second.
 */
#include <string.h>

#include "mesh/mesh_mic.h"

/* ---- AES-128, encrypt direction only (FIPS-197) ---- */

static const uint8_t sbox[256] = {
    0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5,
    0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
    0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0,
    0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
    0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc,
    0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
    0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a,
    0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
    0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0,
    0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
    0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b,
    0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
    0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85,
    0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
    0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5,
    0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
    0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17,
    0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
    0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88,
    0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
    0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c,
    0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
    0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9,
    0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
    0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6,
    0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
    0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e,
    0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
    0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94,
    0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
    0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68,
    0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16,
};

static uint8_t xtime(uint8_t x)
{
    return (uint8_t)((x << 1) ^ ((x >> 7) * 0x1B));
}

static void aes128_expand(uint8_t rk[176], const uint8_t key[16])
{
    static const uint8_t rcon[10] = {
        0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1B, 0x36,
    };
    memcpy(rk, key, 16);
    unsigned r = 0;
    for (unsigned i = 16; i < 176; i += 4) {
        uint8_t t0 = rk[i - 4], t1 = rk[i - 3];
        uint8_t t2 = rk[i - 2], t3 = rk[i - 1];
        if (i % 16 == 0) {
            uint8_t rot = t0;
            t0 = (uint8_t)(sbox[t1] ^ rcon[r++]);
            t1 = sbox[t2];
            t2 = sbox[t3];
            t3 = sbox[rot];
        }
        rk[i + 0] = rk[i - 16] ^ t0;
        rk[i + 1] = rk[i - 15] ^ t1;
        rk[i + 2] = rk[i - 14] ^ t2;
        rk[i + 3] = rk[i - 13] ^ t3;
    }
}

static void aes128_encrypt(const uint8_t rk[176], const uint8_t in[16],
                           uint8_t out[16])
{
    uint8_t s[16];
    for (unsigned i = 0; i < 16; i++)
        s[i] = in[i] ^ rk[i];

    for (unsigned round = 1; round <= 10; round++) {
        uint8_t t;
        for (unsigned i = 0; i < 16; i++)
            s[i] = sbox[s[i]];
        /* ShiftRows (state is column-major: s[4*col + row]) */
        t = s[1];  s[1] = s[5];   s[5] = s[9];   s[9] = s[13];  s[13] = t;
        t = s[2];  s[2] = s[10];  s[10] = t;
        t = s[6];  s[6] = s[14];  s[14] = t;
        t = s[3];  s[3] = s[15];  s[15] = s[11]; s[11] = s[7];  s[7] = t;
        if (round != 10) {
            for (unsigned c = 0; c < 16; c += 4) {
                uint8_t a0 = s[c], a1 = s[c + 1];
                uint8_t a2 = s[c + 2], a3 = s[c + 3];
                uint8_t x = (uint8_t)(a0 ^ a1 ^ a2 ^ a3);
                s[c + 0] ^= (uint8_t)(x ^ xtime((uint8_t)(a0 ^ a1)));
                s[c + 1] ^= (uint8_t)(x ^ xtime((uint8_t)(a1 ^ a2)));
                s[c + 2] ^= (uint8_t)(x ^ xtime((uint8_t)(a2 ^ a3)));
                s[c + 3] ^= (uint8_t)(x ^ xtime((uint8_t)(a3 ^ a0)));
            }
        }
        for (unsigned i = 0; i < 16; i++)
            s[i] ^= rk[16 * round + i];
    }
    memcpy(out, s, 16);
}

/* ---- CMAC (RFC 4493) ---- */

/* Doubling in GF(2^128) with the CMAC reduction polynomial. */
static void cmac_dbl(uint8_t out[16], const uint8_t in[16])
{
    uint8_t msb = (uint8_t)(in[0] >> 7);
    for (unsigned i = 0; i < 15; i++)
        out[i] = (uint8_t)((in[i] << 1) | (in[i + 1] >> 7));
    out[15] = (uint8_t)((in[15] << 1) ^ (msb ? 0x87 : 0x00));
}

void mesh_mic_cmac_init(mesh_mic_cmac_t *c, const uint8_t key[16])
{
    uint8_t L[16] = { 0 };
    aes128_expand(c->rk, key);
    aes128_encrypt(c->rk, L, L);
    cmac_dbl(c->k1, L);
    cmac_dbl(c->k2, c->k1);
}

void mesh_mic_cmac_tag(const mesh_mic_cmac_t *c, const uint8_t *data,
                       size_t len, uint8_t tag[16])
{
    uint8_t x[16] = { 0 };
    uint8_t last[16];
    size_t nblocks = len ? (len + 15) / 16 : 1;
    size_t rem = len - 16 * (nblocks - 1);

    for (size_t b = 0; b + 1 < nblocks; b++) {
        for (unsigned i = 0; i < 16; i++)
            x[i] ^= data[16 * b + i];
        aes128_encrypt(c->rk, x, x);
    }

    memset(last, 0, 16);
    if (len > 0 && rem == 16) {
        for (unsigned i = 0; i < 16; i++)
            last[i] = data[16 * (nblocks - 1) + i] ^ c->k1[i];
    } else {
        if (rem > 0)
            memcpy(last, data + 16 * (nblocks - 1), rem);
        last[rem] = 0x80;
        for (unsigned i = 0; i < 16; i++)
            last[i] ^= c->k2[i];
    }
    for (unsigned i = 0; i < 16; i++)
        x[i] ^= last[i];
    aes128_encrypt(c->rk, x, tag);
}

void mesh_mic_cmac_compute(void *ctx, const uint8_t *data, size_t len,
                           uint8_t mic[MESH_MIC_SIZE])
{
    uint8_t tag[16];
    mesh_mic_cmac_tag((const mesh_mic_cmac_t *)ctx, data, len, tag);
    memcpy(mic, tag, MESH_MIC_SIZE);
}

mesh_mic_ops_t mesh_mic_cmac_ops(mesh_mic_cmac_t *c)
{
    mesh_mic_ops_t ops = {
        .ctx = c,
        .compute = mesh_mic_cmac_compute,
    };
    return ops;
}
