#include "hardware_pwm.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include <cstdio>

HardwarePWM* HardwarePWM::sliceInstances[12] = {};

static uint parseGpioNum(const std::string& pin) {
    if (pin.size() >= 3 && pin.substr(0, 2) == "GP") {
        try { return (uint)std::stoi(pin.substr(2)); }
        catch (...) {}
    }
    printf("HardwarePWM: invalid pin '%s'\n", pin.c_str());
    return 0;
}

HardwarePWM::HardwarePWM(uint32_t initialPeriodUs, float initialDutyPercent,
                          const std::string& pin)
    : gpioNum(parseGpioNum(pin)),
      sliceNum(pwm_gpio_to_slice_num(gpioNum)),
      channel(pwm_gpio_to_channel(gpioNum)),
      timerClkHz(clock_get_hz(clk_sys)),
      periodUs(initialPeriodUs),
      periodPercent(initialDutyPercent)
{
    sliceInstances[sliceNum] = this;

    gpio_set_function(gpioNum, GPIO_FUNC_PWM);

    // Prescaler: divide system clock down to 1 MHz so the wrap register
    // directly represents microseconds.
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)timerClkHz / 1000000.0f);
    pwm_config_set_wrap(&cfg, (uint16_t)(initialPeriodUs > 0 ? initialPeriodUs - 1 : 0));
    pwm_init(sliceNum, &cfg, true);

    changePulsewidth(initialDutyPercent);

    printf("HardwarePWM: GP%u (slice %u chan %u) period=%lu µs duty=%.1f%%\n",
           gpioNum, sliceNum, channel, (unsigned long)initialPeriodUs, (double)initialDutyPercent);
}

HardwarePWM::~HardwarePWM() {
    pwm_set_enabled(sliceNum, false);
    sliceInstances[sliceNum] = nullptr;
}

void HardwarePWM::changePeriod(uint32_t newPeriodUs) {
    if (newPeriodUs == 0) return;
    periodUs = newPeriodUs;

    pwm_set_wrap(sliceNum, (uint16_t)(newPeriodUs - 1));
    pwm_set_counter(sliceNum, 0);

    // Recalculate duty for this channel
    changePulsewidth(periodPercent);

    // If the sibling channel on this slice has its own HardwarePWM instance,
    // recalculate its duty too (period is shared).
    // Since sliceInstances only stores one instance per slice, a full dual-
    // channel implementation would store sliceInstances[12][2].
    // For now a single PWM output per slice is the common case.
}

void HardwarePWM::changePulsewidth(float newDutyPercent) {
    periodPercent = newDutyPercent;
    if (periodUs == 0) return;

    uint32_t level = (uint32_t)((newDutyPercent / 100.0f) * (float)periodUs);
    if (level > periodUs) level = periodUs;
    pwm_set_chan_level(sliceNum, channel, (uint16_t)level);
}
