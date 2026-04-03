#pragma once
#include "hardware/timer.h"
#include "pico/time.h"
#include "remora-core/thread/pruTimer.h"

// RP2350 implementation of pruTimer using the hardware alarm repeating-timer API.
//
// The RP2350 has two 64-bit timer blocks (TIMER0, TIMER1), each with 4 alarm
// channels.  add_repeating_timer_us() uses these alarms to fire a callback at
// precise intervals.
//
// Thread assignments (see PortingPlan.md §7.4):
//   Base thread   — TIMER0 alarm 0, 40 kHz, IRQ priority 0  (highest)
//   Servo thread  — TIMER0 alarm 1,  1 kHz, IRQ priority 64
//   Serial thread — TIMER1 alarm 0, 57.6 kHz, IRQ priority 128
//
// Using a negative period_us in add_repeating_timer_us() means "fire from the
// end of the previous callback" rather than from its scheduled start time,
// which prevents drift accumulation under load.

class RP2350_timer : public pruTimer {
private:
    repeating_timer_t timerState;
    uint              alarmNum;     // 0–3 within the chosen timer block
    bool              useTimer1;    // false = TIMER0, true = TIMER1
    int               irqPriority;  // 0 (highest) – 255 (lowest)

    // Static callback: returning true keeps the repeating timer alive.
    static bool timerCallback(repeating_timer_t* rt) {
        auto* self = static_cast<RP2350_timer*>(rt->user_data);
        self->timerTick();
        return true;
    }

public:
    // alarmNum:    which alarm channel to use (0–3)
    // useTimer1:   false for TIMER0 (default pool), true for TIMER1
    // frequency:   interrupt rate in Hz
    // ownerPtr:    pruThread that owns this timer (set by pruThread itself)
    // irqPriority: 0 = highest, 255 = lowest
    RP2350_timer(uint alarmNum, bool useTimer1,
                 uint32_t frequency, pruThread* ownerPtr,
                 int irqPriority = 0);

    void configTimer() override;
    void startTimer()  override;
    void stopTimer()   override;
    void timerTick()   override;
};
