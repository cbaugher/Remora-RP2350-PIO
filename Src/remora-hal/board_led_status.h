#pragma once
#include <string>

// Board status LED blink-code system.
// Error codes are signalled as blink sequences on the board LED (default GP25).
// Each pattern is 6 pulses; FAST = 250 ms on/off, SLOW = 1000 ms on/off.
// After the pattern a 1500 ms pause before repeating.

// Error codes (passed to flash_led_error)
enum BoardError {
    CRITICAL_HAL_ERROR     = 1,
    SD_CARD_HW_ERROR       = 2,
    SD_CARD_MOUNT_ERROR    = 3,
    SD_CARD_FILE_ERROR     = 4,
    SPI_PERIPH_ERROR       = 5,
};

// Initialise the status LED on the given GPIO pin (e.g. "GP25").
// Must be called once before flash_led_error().
void init_board_status_led(const std::string& pin);

// Blink the status LED repeatedly to indicate the given error code.
// This function never returns — intended for fatal-error reporting.
void flash_led_error(BoardError error);
