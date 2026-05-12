// hardware/rtc.h — RP2350 stub
//
// The RP2350 has no dedicated RTC peripheral (unlike RP2040).  It uses the
// AON (Always-On) timer instead.  The no-OS-FatFS-SD-SPI-RPi-Pico library's
// rtc.c includes this header to obtain file-creation timestamps.  This stub
// satisfies the compile dependency and makes rtc_get_datetime() return false,
// which causes get_fattime() to return 0 (epoch).  File timestamps on the SD
// card will be zeroed, which is acceptable for a read-mostly config loader.
//
// This file must appear in the include path BEFORE the pico-sdk hardware/
// directories so it shadows the missing hardware/rtc.h on RP2350.
// CMakeLists.txt achieves this via:
//   target_include_directories(FatFs_SPI BEFORE PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/Src)

#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef struct {
    int16_t year;   // e.g. 2025
    int8_t  month;  // 1–12
    int8_t  day;    // 1–31
    int8_t  dotw;   // 0 = Sunday
    int8_t  hour;   // 0–23
    int8_t  min;    // 0–59
    int8_t  sec;    // 0–59
} datetime_t;

// No-op stubs — no RTC peripheral on RP2350.
static inline void rtc_init(void) {}
static inline bool rtc_set_datetime(const datetime_t *t) { (void)t; return false; }

// Always returns false — no RTC available on RP2350.
// FatFs rtc.c checks this return value and returns 0 from get_fattime().
static inline bool rtc_get_datetime(datetime_t *t) {
    (void)t;
    return false;
}
