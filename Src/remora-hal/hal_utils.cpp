#include "hal_utils.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include <cstring>
#include <cstdio>

// ---------------------------------------------------------------------------
// Low-level flash primitives — must run from RAM
//
// Multicore safety: on RP2350 with dual-core use (Core 0 = real-time threads,
// Core 1 = comms/TFTP), either core may call these functions:
//   - Core 1's TFTP handler writes to upload staging flash
//   - Core 0's store_json_in_flash() copies staging to config flash
//
// flash_range_erase/program suspend the XIP interface on the calling core
// but do NOT stop the other core from fetching instructions from flash.
// multicore_lockout_start_blocking() parks the other core in RAM before
// the flash operation begins, and multicore_lockout_end_blocking() releases
// it afterward.
//
// Both cores must call multicore_lockout_victim_init() at startup so that
// either core can be locked out by the other.
// ---------------------------------------------------------------------------

static void __not_in_flash_func(rp2350_flash_erase_sector)(uint32_t flash_offset) {
    // flash_offset must be 4 KB aligned
    multicore_lockout_start_blocking();
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(flash_offset, FLASH_SECTOR_SIZE);   // 4096 bytes
    restore_interrupts(irq_state);
    multicore_lockout_end_blocking();
}

static void __not_in_flash_func(rp2350_flash_write_page)(uint32_t flash_offset,
                                                           const uint8_t* data) {
    // flash_offset must be 256-byte aligned
    multicore_lockout_start_blocking();
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_program(flash_offset, data, FLASH_PAGE_SIZE);   // 256 bytes
    restore_interrupts(irq_state);
    multicore_lockout_end_blocking();
}

// ---------------------------------------------------------------------------
// Public erase functions
// ---------------------------------------------------------------------------

void mass_erase_flash_sector(uint32_t page_index) {
    rp2350_flash_erase_sector(page_index * FLASH_SECTOR_SIZE);
}

void mass_erase_config_storage() {
    // JSON_STORAGE is 16 KB = 4 × 4 KB sectors
    for (uint32_t i = 0; i < 4; i++) {
        rp2350_flash_erase_sector(Platform_Config::JSON_STORAGE_OFFSET + i * FLASH_SECTOR_SIZE);
    }
}

void mass_erase_upload_storage() {
    for (uint32_t i = 0; i < 4; i++) {
        rp2350_flash_erase_sector(Platform_Config::JSON_UPLOAD_OFFSET + i * FLASH_SECTOR_SIZE);
    }
}

// ---------------------------------------------------------------------------
// Buffered flash write system
//
// jsonConfigHandler calls write_to_flash_byte() in a tight loop, one byte at
// a time.  RP2350 flash must be written in 256-byte pages, so we buffer writes
// and flush on page boundaries (or when explicitly asked).
// ---------------------------------------------------------------------------

static uint8_t   s_page_buf[FLASH_PAGE_SIZE];
static uint32_t  s_page_flash_offset = UINT32_MAX;   // UINT32_MAX = no page open
static uint32_t  s_page_pos          = 0;

void flush_write_buffer() {
    if (s_page_flash_offset != UINT32_MAX) {
        rp2350_flash_write_page(s_page_flash_offset, s_page_buf);
        s_page_flash_offset = UINT32_MAX;
        s_page_pos          = 0;
    }
}

uint8_t write_to_flash_byte(uint32_t xip_addr, uint8_t data) {
    uint32_t flash_offset = xip_addr - Platform_Config::FLASH_XIP_BASE;
    uint32_t page_offset  = flash_offset & ~((uint32_t)(FLASH_PAGE_SIZE - 1));  // align to 256

    if (page_offset != s_page_flash_offset) {
        flush_write_buffer();
        memset(s_page_buf, 0xFF, FLASH_PAGE_SIZE);  // init page to erased state
        s_page_flash_offset = page_offset;
        s_page_pos          = 0;
    }

    s_page_buf[flash_offset & (FLASH_PAGE_SIZE - 1)] = data;
    s_page_pos++;

    if (s_page_pos >= FLASH_PAGE_SIZE) {
        flush_write_buffer();
    }
    return 0;  // 0 = HAL_OK equivalent
}

uint8_t write_to_flash_halfword(uint32_t xip_addr, uint16_t data) {
    uint8_t result = write_to_flash_byte(xip_addr,     (uint8_t)(data & 0xFF));
    result        |= write_to_flash_byte(xip_addr + 1, (uint8_t)(data >> 8));
    return result;
}

uint8_t write_to_flash_word(uint32_t xip_addr, uint32_t data) {
    uint8_t result = write_to_flash_byte(xip_addr,     (uint8_t)(data & 0xFF));
    result        |= write_to_flash_byte(xip_addr + 1, (uint8_t)((data >> 8)  & 0xFF));
    result        |= write_to_flash_byte(xip_addr + 2, (uint8_t)((data >> 16) & 0xFF));
    result        |= write_to_flash_byte(xip_addr + 3, (uint8_t)((data >> 24) & 0xFF));
    return result;
}

void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}
