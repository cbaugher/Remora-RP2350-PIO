#pragma once
#include <string>
#include <cstdint>

// RP2350 hardware PWM.
//
// Every GPIO can produce PWM output.  The slice and channel are derived
// algorithmically:
//   slice   = gpio / 2     (0–11)
//   channel = gpio % 2     (0 = CHAN_A, 1 = CHAN_B)
//
// The clock divider is set so that the counter increments at 1 MHz (1 µs
// resolution at a 150 MHz system clock), meaning `periodUs` directly sets
// the wrap register and `dutyPercent` is converted to a level in µs.
//
// Two GPIOs on the same slice share the same period; changing the period on
// one channel automatically updates both.

class HardwarePWM {
private:
    uint     gpioNum;
    uint     sliceNum;
    uint     channel;       // PWM_CHAN_A or PWM_CHAN_B
    uint32_t timerClkHz;

public:
    // One entry per slice — used to propagate period changes to sibling channels
    static HardwarePWM* sliceInstances[12];

    uint32_t periodUs;
    float    periodPercent;

    // pin: "GPn" string, e.g. "GP1"
    HardwarePWM(uint32_t initialPeriodUs, float initialDutyPercent,
                const std::string& pin);
    ~HardwarePWM();

    void changePeriod(uint32_t newPeriodUs);
    void changePulsewidth(float newDutyPercent);

    // Snake-case aliases for compatibility with remora-core's PWM module
    void change_period(uint32_t p)  { changePeriod(p); }
    void change_pulsewidth(float d) { changePulsewidth(d); }
};
