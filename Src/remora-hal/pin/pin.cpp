#include "pin.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include <cstdio>
#include <stdexcept>

// Parse "GP0" … "GP47" into an integer GPIO number.
// Returns -1 and prints a diagnostic on failure.
void Pin::configurePin() {
    if (portAndPin.size() >= 3 && portAndPin.substr(0, 2) == "GP") {
        try {
            int n = std::stoi(portAndPin.substr(2));
            if (n >= 0 && n <= 47) {
                gpioNum = n;
                return;
            }
        } catch (...) {}
    }
    printf("Pin::configurePin() — invalid pin name '%s' "
           "(expected 'GP0' through 'GP47')\n", portAndPin.c_str());
    gpioNum = -1;
}

// No clock enable required — all GPIO are always clocked on RP2350.
void Pin::enableClock() {}

void Pin::initialisePin() {
    if (gpioNum < 0) return;
    gpio_init(gpioNum);

    if (mode == INPUT) {
        gpio_set_dir(gpioNum, GPIO_IN);
        if (modifier == PULLUP || modifier == OPENDRAIN) {
            gpio_pull_up(gpioNum);
        } else if (modifier == PULLDOWN) {
            gpio_pull_down(gpioNum);
        } else {
            gpio_disable_pulls(gpioNum);
        }
    } else {
        gpio_set_dir(gpioNum, GPIO_OUT);
        gpio_put(gpioNum, 0);
    }
}

void Pin::initialiseGPIO() {
    if (gpioNum < 0) return;
    gpio_init(gpioNum);
    // gpio_alt holds the gpio_function_t value
    gpio_set_function(gpioNum, static_cast<gpio_function_t>(gpio_alt));
    if (gpio_pull == PULLUP) {
        gpio_pull_up(gpioNum);
    } else if (gpio_pull == PULLDOWN) {
        gpio_pull_down(gpioNum);
    } else {
        gpio_disable_pulls(gpioNum);
    }
}

// Digital GPIO constructor
Pin::Pin(const std::string& portAndPin, int mode, int modifier)
    : portAndPin(portAndPin), gpioNum(-1),
      mode(mode), modifier(modifier), gpio_alt(-1), gpio_pull(NONE)
{
    configurePin();
    enableClock();
    initialisePin();
}

// Convenience constructor — OUTPUT with no pull modifier
Pin::Pin(const std::string& portAndPin, int mode)
    : Pin(portAndPin, mode, NONE)
{}

// Alternate-function constructor (SPI, PWM, UART, PIO, …)
Pin::Pin(const std::string& portAndPin, int gpio_alt_func, int gpio_pull_mode, bool /*altFunc*/)
    : portAndPin(portAndPin), gpioNum(-1),
      mode(-1), modifier(NONE), gpio_alt(gpio_alt_func), gpio_pull(gpio_pull_mode)
{
    configurePin();
    enableClock();
    initialiseGPIO();
}

bool Pin::get() const {
    if (gpioNum < 0) return false;
    return gpio_get(gpioNum);
}

void Pin::set(bool value) {
    if (gpioNum < 0) return;
    gpio_put(gpioNum, value ? 1 : 0);
}

void Pin::toggle() {
    if (gpioNum < 0) return;
    gpio_put(gpioNum, !gpio_get(gpioNum));
}

void Pin::setAsOutput() {
    if (gpioNum < 0) return;
    gpio_set_dir(gpioNum, GPIO_OUT);
}

void Pin::setAsInput() {
    if (gpioNum < 0) return;
    gpio_set_dir(gpioNum, GPIO_IN);
    gpio_disable_pulls(gpioNum);
}

// TODO: review — setPullUp(), setAsInput(), and setAsInput(int) overlap;
//       consider consolidating into a single method.
void Pin::setPullUp() {
    if (gpioNum < 0) return;
    gpio_pull_up(gpioNum);
}

void Pin::setAsInput(int pullMode) {
    if (gpioNum < 0) return;
    gpio_set_dir(gpioNum, GPIO_IN);
    if (pullMode == PULLUP || pullMode == OPENDRAIN) {
        gpio_pull_up(gpioNum);
    } else if (pullMode == PULLDOWN) {
        gpio_pull_down(gpioNum);
    } else {
        gpio_disable_pulls(gpioNum);
    }
}
