/*
 * provision_cli.c — see provision_cli.h.
 */
#include "provision_cli.h"

#include <stdio.h>
#include <string.h>

#include "hardware/watchdog.h"
#include "pico/stdlib.h"

#include "flash_store.h"
#include "node_config.h"

static void reboot_to_apply(void)
{
    printf("rebooting to apply...\n");
    sleep_ms(50); /* let USB flush the line */
    watchdog_reboot(0, 0, 0);
    for (;;)
        tight_loop_contents();
}

static void show(void)
{
    prov_t p;
    if (!prov_decode(flash_store_read(FLASH_SLOT_PROVISION), &p)) {
        printf("unprovisioned (compiled defaults active: node %u, "
               "net %04X)\n", NODE_ID, MESH_NET_ID);
        return;
    }
    char keyhex[33];
    prov_format_key_hex(p.key, keyhex);
    /* Print only the key's ends — enough to compare devices, not enough
     * to shoulder-surf the deployment key off a bench log. */
    printf("provisioned: node %u, net %04X, key %.4s...%s\n", p.node_id,
           p.net_id, keyhex, keyhex + 28);
}

bool provision_cli(const char *line)
{
    if (strncmp(line, "provision", 9) != 0)
        return false;
    const char *args = line + 9;
    while (*args == ' ')
        args++;

    if (strcmp(args, "show") == 0 || *args == '\0') {
        show();
        return true;
    }
    if (strcmp(args, "clear") == 0) {
        flash_store_erase(FLASH_SLOT_PROVISION);
        printf("provisioning cleared; ");
        reboot_to_apply();
        return true;
    }

    unsigned id, netid;
    char keyhex[64];
    if (sscanf(args, "%u %x %63s", &id, &netid, keyhex) != 3 ||
        id > 254 || netid > 0xFFFF) {
        printf("usage: provision <id 0-254> <netid hex4> <key hex32>\n"
               "       provision show | clear\n");
        return true;
    }
    prov_t p = { .node_id = (uint8_t)id, .net_id = (uint16_t)netid };
    if (!prov_parse_key_hex(keyhex, p.key)) {
        printf("key must be exactly 32 hex chars "
               "(openssl rand -hex 16)\n");
        return true;
    }

    uint8_t blob[PROV_BLOB_SIZE];
    prov_encode(&p, blob);
    flash_store_write(FLASH_SLOT_PROVISION, blob, sizeof(blob));

    prov_t back;
    if (!prov_decode(flash_store_read(FLASH_SLOT_PROVISION), &back) ||
        back.node_id != p.node_id) {
        printf("flash verify FAILED\n");
        return true;
    }
    printf("provisioned node %u net %04X; ", back.node_id, back.net_id);
    reboot_to_apply();
    return true;
}

bool provision_boot(prov_t *out, uint8_t fallback_id)
{
    if (prov_decode(flash_store_read(FLASH_SLOT_PROVISION), out)) {
        printf("identity: node %u net %04X (provisioned)\n", out->node_id,
               out->net_id);
        return true;
    }
    static const uint8_t default_key[16] = MESH_NET_KEY;
    out->node_id = fallback_id;
    out->net_id = MESH_NET_ID;
    memcpy(out->key, default_key, 16);
    printf("identity: node %u net %04X — UNPROVISIONED, compiled "
           "defaults (bench only; see `provision`)\n", out->node_id,
           out->net_id);
    return false;
}
