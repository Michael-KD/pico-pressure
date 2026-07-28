/*
 * provision.c — provisioning-blob codec (see provision.h).
 *
 * Layout (all offsets fixed; integrity = mesh_crc16 over bytes 0..23):
 *   0  magic "MPV1"
 *   4  node_id
 *   5  flags (reserved, 0)
 *   6  net_id (2 LE)
 *   8  key (16)
 *   24 crc16 (2 LE, CRC-16/CCITT-FALSE — the library's mesh_crc16)
 *   26 pad 0xFF to PROV_BLOB_SIZE
 */
#include "provision.h"

#include <string.h>

#include "mesh/mesh.h" /* mesh_crc16 */

static const uint8_t prov_magic[4] = { 'M', 'P', 'V', '1' };

#define PROV_CRC_OFF 24u

void prov_encode(const prov_t *p, uint8_t blob[PROV_BLOB_SIZE])
{
    memset(blob, 0xFF, PROV_BLOB_SIZE);
    memcpy(&blob[0], prov_magic, 4);
    blob[4] = p->node_id;
    blob[5] = 0;
    blob[6] = (uint8_t)(p->net_id & 0xFF);
    blob[7] = (uint8_t)(p->net_id >> 8);
    memcpy(&blob[8], p->key, 16);
    uint16_t crc = mesh_crc16(blob, PROV_CRC_OFF);
    blob[PROV_CRC_OFF] = (uint8_t)(crc & 0xFF);
    blob[PROV_CRC_OFF + 1] = (uint8_t)(crc >> 8);
}

bool prov_decode(const uint8_t blob[PROV_BLOB_SIZE], prov_t *out)
{
    if (memcmp(&blob[0], prov_magic, 4) != 0)
        return false;
    uint16_t crc = (uint16_t)(blob[PROV_CRC_OFF] |
                              ((uint16_t)blob[PROV_CRC_OFF + 1] << 8));
    if (mesh_crc16(blob, PROV_CRC_OFF) != crc)
        return false;
    if (blob[4] == MESH_ID_BROADCAST) /* 255 is not a node identity */
        return false;
    out->node_id = blob[4];
    out->net_id = (uint16_t)(blob[6] | ((uint16_t)blob[7] << 8));
    memcpy(out->key, &blob[8], 16);
    return true;
}

static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9')
        return c - '0';
    if (c >= 'a' && c <= 'f')
        return c - 'a' + 10;
    if (c >= 'A' && c <= 'F')
        return c - 'A' + 10;
    return -1;
}

bool prov_parse_key_hex(const char *hex, uint8_t key[16])
{
    for (unsigned i = 0; i < 16; i++) {
        int hi = hex_nibble(hex[2 * i]);
        if (hi < 0)
            return false;
        int lo = hex_nibble(hex[2 * i + 1]);
        if (lo < 0)
            return false;
        key[i] = (uint8_t)((hi << 4) | lo);
    }
    return hex[32] == '\0';
}

void prov_format_key_hex(const uint8_t key[16], char out[33])
{
    static const char digits[] = "0123456789abcdef";
    for (unsigned i = 0; i < 16; i++) {
        out[2 * i] = digits[key[i] >> 4];
        out[2 * i + 1] = digits[key[i] & 0xF];
    }
    out[32] = '\0';
}
