#pragma once

// ---------------------------------------------------------------------------
// fatfs.h — FatFs integration shim
//
// SPI build (SPI_CTRL defined):
//   Includes the real ff.h from the no-OS-FatFS-SD-SPI-RPi-Pico library.
//   SDFatFS, SDFile, SDPath are defined in sd_globals.cpp and declared here.
//   The SD card is wired to SPI1: SCK=GP10 MOSI=GP11 MISO=GP12 CS=GP13.
//   A FAT32-formatted SD card with config.txt in the root is required.
//
// ETH build (SPI_CTRL not defined):
//   Uses a lightweight stub that serves a hardcoded default JSON config so
//   the Remora state machine can reach ST_START without any SD card hardware.
//   This stub is sufficient for Milestones 1–2 and ETH-path testing.
// ---------------------------------------------------------------------------

#if defined(SPI_CTRL) && !defined(NO_FATFS_SPI)

// ---- Real FatFs (SPI build, library submodule present) --------------------
#include "ff.h"   // provided by FatFs_SPI CMake target (no-OS-FatFS-SD-SPI-RPi-Pico)

// Defined in sd_globals.cpp; declared here so jsonConfigHandler.cpp can use them.
extern FATFS  SDFatFS;
extern FIL    SDFile;
extern TCHAR  SDPath[4];

#else
// Covers:
//   - ETH build (ETH_CTRL defined, SPI_CTRL not defined)
//   - SPI build without the FatFs submodule (NO_FATFS_SPI defined)
// In the second case the hardcoded default config is served so the SPI comms
// path can be tested before the SD card is wired up.

// ---- ETH build stub --------------------------------------------------------
#include <cstdint>
#include <cstddef>
#include <cstring>

typedef char     TCHAR;
typedef uint32_t UINT;
typedef uint8_t  BYTE;
typedef uint32_t DWORD;

typedef struct { uint8_t placeholder[64]; } FATFS;
typedef struct { uint8_t placeholder[64]; DWORD fsize; } FIL;

typedef enum {
    FR_OK = 0,
    FR_DISK_ERR, FR_INT_ERR, FR_NOT_READY, FR_NO_FILE, FR_NO_PATH,
    FR_INVALID_NAME, FR_DENIED, FR_EXIST, FR_INVALID_OBJECT,
    FR_WRITE_PROTECTED, FR_INVALID_DRIVE, FR_NOT_ENABLED, FR_NO_FILESYSTEM,
    FR_MKFS_ABORTED, FR_TIMEOUT, FR_LOCKED, FR_NOT_ENOUGH_CORE,
    FR_TOO_MANY_OPEN_FILES, FR_INVALID_PARAMETER
} FRESULT;

#define FA_READ          0x01
#define FA_WRITE         0x02
#define FA_OPEN_EXISTING 0x00
#define FA_CREATE_NEW    0x04
#define FA_CREATE_ALWAYS 0x08
#define FA_OPEN_ALWAYS   0x10

// Global FatFs objects used by jsonConfigHandler.cpp
inline FATFS SDFatFS;
inline FIL   SDFile;
inline TCHAR SDPath[4] = "0:";

// ---------------------------------------------------------------------------
// Default config served when no SD card is present (ETH build only).
//
// Contains a Blink module (servo thread LED) plus two Stepgen modules (base
// thread) for logic-analyzer testing.  For real motion control load a proper
// config.txt via the ETH TFTP path or SPI SD card (SPI build).
// ---------------------------------------------------------------------------
static const char REMORA_DEFAULT_CONFIG[] =
    "{"
    "\"Threads\":["
        "{\"Thread\":\"Base\",\"Frequency\":40000},"
        "{\"Thread\":\"Servo\",\"Frequency\":1000}"
    "],"
    "\"Modules\":["
        "{"
            "\"Thread\":\"Servo\","
            "\"Type\":\"Blink\","
            "\"Pin\":\"GP25\","
            "\"Frequency\":2"
        "},"
        "{"
            "\"Thread\":\"Base\","
            "\"Type\":\"Stepgen\","
            "\"Comment\":\"Joint 0 test\","
            "\"Joint Number\":0,"
            "\"Enable Pin\":\"GP6\","
            "\"Step Pin\":\"GP7\","
            "\"Direction Pin\":\"GP8\""
        "},"
        "{"
            "\"Thread\":\"Base\","
            "\"Type\":\"Stepgen\","
            "\"Comment\":\"Joint 1 test\","
            "\"Joint Number\":1,"
            "\"Enable Pin\":\"GP9\","
            "\"Step Pin\":\"GP10\","
            "\"Direction Pin\":\"GP11\""
        "}"
    "]"
    "}";

inline FRESULT f_mount(FATFS* /*fs*/, const TCHAR* /*path*/, BYTE /*opt*/) {
    return FR_OK;
}

inline FRESULT f_open(FIL* fp, const TCHAR* /*path*/, BYTE /*mode*/) {
    if (fp) fp->fsize = (DWORD)(sizeof(REMORA_DEFAULT_CONFIG) - 1);
    return FR_OK;
}

inline FRESULT f_read(FIL* /*fp*/, void* buf, UINT btr, UINT* br) {
    UINT len = (UINT)(sizeof(REMORA_DEFAULT_CONFIG) - 1);
    UINT n = (btr < len) ? btr : len;
    memcpy(buf, REMORA_DEFAULT_CONFIG, n);
    if (br) *br = n;
    return FR_OK;
}

inline FRESULT f_close(FIL* /*fp*/) { return FR_OK; }

inline DWORD f_size(FIL* fp) { return fp ? fp->fsize : 0; }

#endif  // SPI_CTRL
