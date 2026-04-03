#include "analogIn.h"
#include "hardware/adc.h"
#include <cstdio>

static bool s_adc_initialized = false;

static int parseGpioNum(const std::string& pin) {
    if (pin.size() >= 3 && pin.substr(0, 2) == "GP") {
        try { return std::stoi(pin.substr(2)); }
        catch (...) {}
    }
    return -1;
}

AnalogIn::AnalogIn(const std::string& portAndPin)
    : portAndPin(portAndPin), gpioNum(-1), channel(-1)
{
    gpioNum = parseGpioNum(portAndPin);

    // Map GPIO number to ADC channel
    if (gpioNum >= 26 && gpioNum <= 29) {
        channel = gpioNum - 26;          // GP26→0, GP27→1, GP28→2, GP29→3
    } else if (gpioNum >= 40 && gpioNum <= 47) {
        channel = gpioNum - 36;          // GP40→4 … GP47→11 (RP2350B only)
    } else {
        printf("AnalogIn: GP%d is not an ADC-capable pin on RP2350\n", gpioNum);
        channel = -1;
        return;
    }

    if (!s_adc_initialized) {
        adc_init();
        s_adc_initialized = true;
    }

    adc_gpio_init(gpioNum);  // disables digital function, no pull resistors
}

uint32_t AnalogIn::read() {
    if (channel < 0) return 0;
    adc_select_input((uint)channel);
    return adc_read();   // 12-bit result [0, 4095]
}
