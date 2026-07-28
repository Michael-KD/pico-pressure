/*
 * provision_cli.h — `provision` USB-serial command, shared by the field
 * node and base station firmwares (Pico glue).
 *
 *   provision <id> <netid-hex4> <key-hex32>   write + reboot to apply
 *   provision show                            print stored identity
 *   provision clear                           erase + reboot
 *
 * Generate a key on the PC (e.g. `openssl rand -hex 16`) — the RP2040
 * has no hardware TRNG worth trusting for key material.
 */
#ifndef PROVISION_CLI_H
#define PROVISION_CLI_H

#include <stdbool.h>

#include "provision.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Handle a CLI line if it is a provision command; returns true when the
 * line was consumed (a successful write/clear reboots and never
 * returns). */
bool provision_cli(const char *line);

/* Boot-time load: fills *out from flash if a valid blob is stored
 * (returns true), else loads the compiled node_config.h defaults with
 * `fallback_id` (returns false). Prints one line saying which. */
bool provision_boot(prov_t *out, uint8_t fallback_id);

#ifdef __cplusplus
}
#endif

#endif /* PROVISION_CLI_H */
