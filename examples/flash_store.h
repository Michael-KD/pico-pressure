/*
 * flash_store.h — tiny fixed-slot blob store in the Pico's on-board
 * flash (Pico glue; no host build).
 *
 * The top of flash is reserved for per-device persistent state, one
 * 4 kB erase sector per slot, growing downward from the end so the
 * layout never collides with the firmware image at the bottom:
 *
 *   slot 0 (last sector)     provisioning blob (provision.h, M4)
 *   slot 1 (second-to-last)  mesh route cache (§5.3, M5)
 *
 * Writes erase the whole sector and program the blob at its start.
 * Reads return a pointer straight into memory-mapped XIP flash. Callers
 * bring their own framing/CRC (both current users do); a blank sector
 * reads as 0xFF and fails their decode, which is the "empty" case.
 */
#ifndef FLASH_STORE_H
#define FLASH_STORE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_SLOT_PROVISION 0u
#define FLASH_SLOT_ROUTE     1u
#define FLASH_STORE_SLOTS    2u

/* Largest blob a slot accepts (one flash page). */
#define FLASH_STORE_MAX 256u

/* Read-only view of a slot's contents (XIP-mapped, always valid). */
const uint8_t *flash_store_read(unsigned slot);

/* Erase the slot's sector and program `len` (<= FLASH_STORE_MAX) bytes.
 * Skips the erase/program cycle when the stored bytes already match —
 * callers may write on every state change without burning flash wear.
 * Interrupts are disabled for the duration of a real write (~50 ms);
 * pending radio UART bytes survive in the UART FIFO. */
void flash_store_write(unsigned slot, const void *data, size_t len);

/* Erase the slot (contents read back as 0xFF). */
void flash_store_erase(unsigned slot);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_STORE_H */
