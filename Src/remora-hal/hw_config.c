// hw_config.c — SD card hardware configuration for no-OS-FatFS-SD-SPI-RPi-Pico
// Compiled only when the FatFs submodule is present (NO_FATFS_SPI not defined).
//
// The SPI0 peripheral is already occupied by the Remora SPI slave interface
// (RPi comms: CLK=GP2 MOSI=GP3 MISO=GP4 CS=GP5).  The SD card therefore
// uses SPI1 on the following pins:
//
//   SD Card Pin   RP2350 GPIO   SPI1 Function
//   CLK (SCLK)    GP10          SPI1 SCK
//   MOSI (CMD)    GP11          SPI1 TX
//   MISO (DAT0)   GP12          SPI1 RX
//   CS   (DAT3)   GP13          GPIO output (software CS)
//
// Wire these four signals plus 3.3 V and GND between the Waveshare RP2350 Plus
// and the SD card module.  No card-detect pin is used (use_card_detect = false).
//
// Baud rate: 12.5 MHz.  Most SD cards support 25 MHz in SPI mode; 12.5 MHz
// gives comfortable margin.  Increase if SD reads are measurably slow.

#if defined(SPI_CTRL) && !defined(NO_FATFS_SPI)

#include "hw_config.h"

// ---------------------------------------------------------------------------
// SPI bus definition — SPI1
// ---------------------------------------------------------------------------
static spi_t spi_sd = {
    .hw_inst             = spi1,
    .miso_gpio           = 12,   // SPI1 RX  — SD MISO / DAT0
    .mosi_gpio           = 11,   // SPI1 TX  — SD MOSI / CMD
    .sck_gpio            = 10,   // SPI1 SCK — SD CLK  / SCLK
    .baud_rate           = 12500 * 1000,   // 12.5 MHz
    .set_drive_strength  = false,
};

// ---------------------------------------------------------------------------
// SD card definition — single card on SPI1, no card-detect
// ---------------------------------------------------------------------------
static sd_card_t sd_card = {
    .pcName            = "0:",       // FatFs logical drive path
    .spi               = &spi_sd,
    .ss_gpio           = 13,         // Software chip-select — SD CS / DAT3
    .use_card_detect   = false,
    .card_detect_gpio  = 0,          // unused
    .card_detected_true = 0,         // unused
    .set_drive_strength = false,
};

// ---------------------------------------------------------------------------
// Required callback functions — called by the FatFs_SPI library internals
// ---------------------------------------------------------------------------
size_t spi_get_num() { return 1; }

spi_t* spi_get_by_num(size_t num) {
    return (num == 0) ? &spi_sd : NULL;
}

size_t sd_get_num() { return 1; }

sd_card_t* sd_get_by_num(size_t num) {
    return (num == 0) ? &sd_card : NULL;
}

#endif  // SPI_CTRL && !NO_FATFS_SPI
