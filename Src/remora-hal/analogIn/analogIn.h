#pragma once
#include <string>
#include <cstdint>
#include <cmath>   // provides logf, expf, etc. needed by thermistor.cpp

// Single-channel ADC input.
// RP2350 has one 12-bit ADC; accessible pins on Pico 2:
//   GP26 → channel 0, GP27 → channel 1, GP28 → channel 2, GP29 → channel 3
//   GP40–GP47 → channels 4–7 (RP2350B variant only)
//
// read() returns a 12-bit value in the range [0, 4095].
// Modules that need a normalised float should divide by 4096.0f.

class AnalogIn {
private:
    std::string portAndPin;
    int         gpioNum;
    int         channel;    // ADC channel number; -1 = invalid pin

public:
    explicit AnalogIn(const std::string& portAndPin);

    // Returns 12-bit ADC result [0, 4095], or 0 on invalid pin.
    uint32_t read();
};
