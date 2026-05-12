// sd_globals.cpp — SPI build only, real FatFs path
//
// Defines the FatFs workspace objects used by jsonConfigHandler.cpp.
// When NO_FATFS_SPI is defined (submodule absent), these are not needed
// because fatfs.h's stub provides inline versions instead.

#if defined(SPI_CTRL) && !defined(NO_FATFS_SPI)

#include "ff.h"

FATFS SDFatFS;           // FatFs workspace for the mounted SD card volume
FIL   SDFile;            // FatFs file object for config.txt
TCHAR SDPath[4] = "0:";  // Logical drive path — FatFs drive 0

#endif
