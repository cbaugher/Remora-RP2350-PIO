#include "board_led_status.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstdio>

static int s_ledGpio = 25;  // default: onboard LED on Pico 2

static int parseGpio(const std::string& pin) {
    if (pin.size() >= 3 && pin.substr(0, 2) == "GP") {
        return std::stoi(pin.substr(2));
    }
    return 25;  // safe fallback
}

void init_board_status_led(const std::string& pin) {
    s_ledGpio = parseGpio(pin);
    gpio_init(s_ledGpio);
    gpio_set_dir(s_ledGpio, GPIO_OUT);
    gpio_put(s_ledGpio, 0);
}

// Blink a single pulse: on for `on_ms`, off for `off_ms`.
static void blink_pulse(uint32_t on_ms, uint32_t off_ms) {
    gpio_put(s_ledGpio, 1);
    sleep_ms(on_ms);
    gpio_put(s_ledGpio, 0);
    sleep_ms(off_ms);
}

// Flash `count` pulses using FAST (250 ms) timing, then pause 1500 ms.
static void blink_pattern(int count) {
    for (int i = 0; i < count; i++) {
        blink_pulse(250, 125);
    }
    sleep_ms(1500);
}

void flash_led_error(BoardError error) {
    printf("FATAL ERROR: board error code %d — halting\n", (int)error);

    // Error code is mapped to a repeat count and timing.
    // All patterns use the same 6-pulse FAST sequence; the error value
    // determines how many times the group repeats before a longer pause,
    // giving a human-readable identification pattern:
    //   1 = CRITICAL_HAL_ERROR   (1 group of 6 fast pulses)
    //   2 = SD_CARD_HW_ERROR     (2 groups)
    //   3 = SD_CARD_MOUNT_ERROR  (3 groups)
    //   4 = SD_CARD_FILE_ERROR   (4 groups)
    //   5 = SPI_PERIPH_ERROR     (5 groups)
    while (true) {
        for (int g = 0; g < (int)error; g++) {
            blink_pattern(6);
        }
        sleep_ms(3000);  // long pause between the full error-code sequence
    }
}
