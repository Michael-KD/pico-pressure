/*
 * flash_store.c — see flash_store.h.
 *
 * RP2040 flash rules honored here: erase/program run from RAM with
 * interrupts off (any IRQ handler would fault fetching code from the
 * XIP flash being written), offsets are sector-aligned for erase and
 * page-aligned for program. The examples are single-core, so no
 * second-core lockout is needed.
 */
#include "flash_store.h"

#include <string.h>

#include "hardware/flash.h"
#include "hardware/sync.h"

static uint32_t slot_offset(unsigned slot)
{
    return PICO_FLASH_SIZE_BYTES - (slot + 1u) * FLASH_SECTOR_SIZE;
}

const uint8_t *flash_store_read(unsigned slot)
{
    return (const uint8_t *)(XIP_BASE + slot_offset(slot));
}

void flash_store_write(unsigned slot, const void *data, size_t len)
{
    if (slot >= FLASH_STORE_SLOTS || len > FLASH_STORE_MAX)
        return;
    if (memcmp(flash_store_read(slot), data, len) == 0)
        return; /* already stored: save the erase cycle */

    uint8_t page[FLASH_PAGE_SIZE];
    memset(page, 0xFF, sizeof(page));
    memcpy(page, data, len);

    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(slot_offset(slot), FLASH_SECTOR_SIZE);
    flash_range_program(slot_offset(slot), page, sizeof(page));
    restore_interrupts(ints);
}

void flash_store_erase(unsigned slot)
{
    if (slot >= FLASH_STORE_SLOTS)
        return;
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(slot_offset(slot), FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
}
