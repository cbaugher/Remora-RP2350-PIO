#include "RP2350_timer.h"
#include "remora-core/thread/pruThread.h"
#include "hardware/irq.h"
#include <cstdio>

RP2350_timer::RP2350_timer(uint alarmNum, bool useTimer1,
                            uint32_t frequency, pruThread* ownerPtr,
                            int irqPriority)
    : alarmNum(alarmNum), useTimer1(useTimer1), irqPriority(irqPriority)
{
    this->frequency     = frequency;
    this->timerOwnerPtr = ownerPtr;
    this->timerRunning  = false;
    // TimerInterrupt (Interrupt-table entry) is not used here — the repeating-
    // timer callback calls timerTick() directly, which calls timerOwnerPtr->update().
    // The Interrupt dispatch table is still used by ModuleInterrupt<T> for GPIO
    // and DMA events; timer alarms bypass it for lowest latency.
}

void RP2350_timer::configTimer() {
    // Set IRQ priority for the alarm IRQ line.
    // TIMER0 alarms: IRQ 0–3; TIMER1 alarms: IRQ 8–11.
    uint irqNum = (useTimer1 ? 8u : 0u) + alarmNum;
    irq_set_priority(irqNum, (uint8_t)irqPriority);
    printf("RP2350_timer: alarm %u on TIMER%u, %lu Hz, IRQ %u priority %d\n",
           alarmNum, useTimer1 ? 1u : 0u, (unsigned long)frequency,
           irqNum, irqPriority);
}

void RP2350_timer::startTimer() {
    // Negative period fires from end of previous callback — prevents drift.
    int32_t period_us = -(int32_t)(1000000u / frequency);

    alarm_pool_t* pool;
    if (useTimer1) {
        // TIMER1: create a pool on an unused hardware alarm from the second timer
        pool = alarm_pool_create_with_unused_hardware_alarm(4u);
        if (!pool) {
            printf("RP2350_timer: failed to create TIMER1 alarm pool\n");
            return;
        }
    } else {
        pool = alarm_pool_get_default();   // TIMER0 default pool
    }

    bool ok = alarm_pool_add_repeating_timer_us(pool, period_us,
                                                timerCallback, this,
                                                &timerState);
    if (!ok) {
        printf("RP2350_timer: failed to start repeating timer (no free alarms?)\n");
        return;
    }
    timerRunning = true;
    printf("RP2350_timer: started at %lu Hz\n", (unsigned long)frequency);
}

void RP2350_timer::stopTimer() {
    cancel_repeating_timer(&timerState);
    timerRunning = false;
    printf("RP2350_timer: stopped\n");
}

void RP2350_timer::timerTick() {
    if (timerOwnerPtr) {
        timerOwnerPtr->update();
    }
}
