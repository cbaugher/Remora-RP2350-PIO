#pragma once
#include <cstdint>

// RP2350 external QSPI flash layout (4 MB device on Pico 2).
// Flash is XIP-mapped at 0x10000000 at runtime.
// flash_range_erase() / flash_range_program() take a byte *offset* from the
// start of flash (not the XIP mapped address).
//
// Layout (top 32 KB reserved for JSON config):
//   [0x000000 – 0x3F7FFF]  Application code (≈4062 KB)
//   [0x3F8000 – 0x3FBFFF]  JSON_UPLOAD staging area   (16 KB)
//   [0x3FC000 – 0x3FFFFF]  JSON_STORAGE active config  (16 KB)

namespace Platform_Config {

    constexpr uint32_t FLASH_XIP_BASE    = 0x10000000u;
    constexpr uint32_t FLASH_TOTAL_BYTES = 4u * 1024u * 1024u;   // 4 MB

    // Byte offsets from start of flash (used with flash_range_erase/program)
    constexpr uint32_t JSON_UPLOAD_OFFSET  = FLASH_TOTAL_BYTES - 32768u;  // 0x3F8000
    constexpr uint32_t JSON_STORAGE_OFFSET = FLASH_TOTAL_BYTES - 16384u;  // 0x3FC000

    // XIP-mapped addresses for reading via pointer dereference
    constexpr uintptr_t JSON_upload_start_address  = FLASH_XIP_BASE + JSON_UPLOAD_OFFSET;
    constexpr uintptr_t JSON_upload_end_address    = JSON_upload_start_address  + 16384u;
    constexpr uintptr_t JSON_storage_start_address = FLASH_XIP_BASE + JSON_STORAGE_OFFSET;
    constexpr uintptr_t JSON_storage_end_address   = JSON_storage_start_address + 16384u;

    // 4 KB page indices (used by mass_erase_flash_sector)
    constexpr uint32_t JSON_Config_Upload_Sector  = JSON_UPLOAD_OFFSET  / 4096u;
    constexpr uint32_t JSON_Config_Storage_Sector = JSON_STORAGE_OFFSET / 4096u;

} // namespace Platform_Config
