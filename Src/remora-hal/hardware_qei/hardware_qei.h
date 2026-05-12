#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware.h"
#include "remora-core/modules/moduleInterrupt.h"

// RP2350 hardware quadrature encoder using PIO.
//
// The PIO quadrature encoder program (quadrature_encoder.pio from pico-examples)
// is loaded into a PIO state machine.  It decodes all four transitions per
// electrical cycle and accumulates a signed 32-bit count accessible via
// quadrature_encoder_get_count().
//
// The index pin (Z channel) is handled by a GPIO interrupt that records the
// count at each index pulse.
//
// NOTE: The chA and chB pins must currently be consecutive GPIO numbers because
// the standard PIO program uses IN PINS 2 with base = chAPin.  If non-
// consecutive pins are required, a modified PIO program is needed.
//
// The STM32 version had these pins hardcoded as "PC_6"/"PC_7"/"PA_8".
// The RP2350 constructor accepts pin numbers as arguments.  A matching change
// to remora-core/modules/qei/qei.cpp is required to read "ChA Pin"/"ChB Pin"/
// "Index Pin" from JSON and pass them here (see PortingPlan.md §12.2).

class Hardware_QEI {
private:
    PIO  pioInst;
    uint smNum;
    uint pioOffset;

    int  chAPin;
    int  chBPin;
    int  indexPin;
    bool hasIndex;
    int  modifier;

    ModuleInterrupt<Hardware_QEI>* indexInterrupt;

    void handleIndexInterrupt();

public:
    bool    indexDetected;
    int32_t indexCount;

    // hasIndex: true to enable the Z-channel interrupt
    // modifier: PULLUP / PULLDOWN / NONE for the index pin input
    // chAPin, chBPin: consecutive GPIO numbers for the A and B encoder channels
    // indexPin: GPIO for the Z channel (-1 if unused)
    Hardware_QEI(bool hasIndex, int modifier,
                 int chAPin = 14, int chBPin = 15, int indexPin = -1);

    void     init();
    uint32_t get();   // returns current unsigned encoder count from PIO FIFO
};
