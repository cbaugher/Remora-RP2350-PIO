#pragma once
#include <string>

// GPIO mode constants
#define OUTPUT   0
#define INPUT    1

// GPIO modifier constants
#define NONE        0
#define PULLUP      1
#define PULLDOWN    2
#define OPEN_DRAIN  3   // mapped to PULLUP on RP2350 (no native open-drain)

// GPIO alternate function constants (gpio_function_t values from pico-sdk)
// These are passed as the `gpio_alt` argument to the alternate-function constructor.
#define GPIO_FUNC_XIP_VALUE   0
#define GPIO_FUNC_SPI_VALUE   1
#define GPIO_FUNC_UART_VALUE  2
#define GPIO_FUNC_I2C_VALUE   3
#define GPIO_FUNC_PWM_VALUE   4
#define GPIO_FUNC_SIO_VALUE   5
#define GPIO_FUNC_PIO0_VALUE  6
#define GPIO_FUNC_PIO1_VALUE  7
#define GPIO_FUNC_PIO2_VALUE  8
#define GPIO_FUNC_GPCK_VALUE  9
#define GPIO_FUNC_USB_VALUE  10

class Pin {
private:
    std::string portAndPin;
    int  gpioNum;
    int  mode;
    int  modifier;
    int  gpio_alt;   // gpio_function_t value for alternate-function pins (-1 = digital)
    int  gpio_pull;

    void configurePin();
    void enableClock();
    void initialisePin();
    void initialiseGPIO();

public:
    // Digital GPIO constructor
    Pin(const std::string& portAndPin, int mode, int modifier);

    // Convenience constructor — OUTPUT, no modifier
    Pin(const std::string& portAndPin, int mode);

    // Alternate-function constructor (SPI, PWM, UART, etc.)
    Pin(const std::string& portAndPin, int gpio_alt_func, int gpio_pull_mode, bool altFunc);

    bool get() const;
    void set(bool value);
    void toggle();
    void setAsOutput();
    void setAsInput();
    void setAsInput(int pullMode);
};
