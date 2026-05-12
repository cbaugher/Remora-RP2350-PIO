#pragma once
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "platform_configuration.h"

// System reset using the watchdog peripheral.
// Equivalent to STM32's HAL_NVIC_SystemReset().
// 500 ms delay lets the USB CDC TX buffer drain before the chip resets;
// without it all printf output after the reboot decision is silently dropped.
#define pru_reboot()    watchdog_reboot(0, 0, 500)

// Flash lock/unlock.
// unlock_flash() returns 0 (HAL_OK) so remora-core's TFTP handler compiles.
// lock_flash() flushes the buffered write page so the last partial 256-byte
// page from write_to_flash_byte() calls is committed before flash access ends.
void flush_write_buffer();   // forward declaration — defined in hal_utils.cpp
inline uint8_t unlock_flash() { return 0; }
inline void    lock_flash()   { flush_write_buffer(); }

// ---------------------------------------------------------------------------
// Flash erase/write wrappers
// These functions must run from RAM (annotated with __not_in_flash_func).
// See hal_utils.cpp for implementations.
// ---------------------------------------------------------------------------

// Erase one 4 KB sector by page index
void mass_erase_flash_sector(uint32_t page_index);

// Erase the JSON config-storage region (16 KB / 4 sectors)
void mass_erase_config_storage();

// Erase the JSON upload-staging region (16 KB / 4 sectors)
void mass_erase_upload_storage();

// Write a single byte to an XIP-mapped flash address.
// Internally buffers writes into 256-byte pages; call flush_write_buffer()
// after the last write to commit any partial page.
// Returns 0 on success (HAL_OK equivalent).
uint8_t write_to_flash_byte(uint32_t xip_addr, uint8_t data);

// Write a 16-bit halfword (little-endian) to flash.
uint8_t write_to_flash_halfword(uint32_t xip_addr, uint16_t data);

// Write a 32-bit word (little-endian) to flash.
uint8_t write_to_flash_word(uint32_t xip_addr, uint32_t data);

// Millisecond delay (wraps sleep_ms)
void delay_ms(uint32_t ms);
