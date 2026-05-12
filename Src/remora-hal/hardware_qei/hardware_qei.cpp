#include "hardware_qei.h"
#include "pin/pin.h"   // for PULLUP / PULLDOWN / NONE constants
#include <cstdio>

// The pico-examples quadrature_encoder PIO program.
// This header is generated from quadrature_encoder.pio by pioasm and provides:
//   quadrature_encoder_program               (the program struct)
//   quadrature_encoder_program_init(pio, sm, offset, pin_ab, max_step_rate)
//   quadrature_encoder_get_count(pio, sm)
//
// Include the .pio.h file generated during the build, or add the .pio source
// to your CMakeLists.txt with pico_generate_pio_header().
//
// For Milestone 1 (LED blink verification) the QEI module is not exercised;
// this implementation becomes active from Milestone 2 onward.
#ifdef HAS_QUADRATURE_ENCODER_PIO_H
#  include "quadrature_encoder.pio.h"
#endif

Hardware_QEI::Hardware_QEI(bool hasIndex, int modifier,
                            int chAPin, int chBPin, int indexPin)
    : pioInst(pio1), smNum(0), pioOffset(0),
      chAPin(chAPin), chBPin(chBPin), indexPin(indexPin),
      hasIndex(hasIndex), modifier(modifier),
      indexInterrupt(nullptr),
      indexDetected(false), indexCount(0)
{}

void Hardware_QEI::init() {
    printf("Hardware_QEI: initialising on GP%d/GP%d%s\n",
           chAPin, chBPin,
           hasIndex ? " with index" : "");

#ifdef HAS_QUADRATURE_ENCODER_PIO_H
    pioInst  = pio1;    // pio0 is used by the SPI slave; QEI must use pio1
    pioOffset = pio_add_program(pioInst, &quadrature_encoder_program);
    smNum    = pio_claim_unused_sm(pioInst, true);

    // chAPin and chBPin must be consecutive (standard PIO program requirement).
    // No offset arg — .origin 0 pins the program to address 0 always.
    quadrature_encoder_program_init(pioInst, smNum, chAPin, 0); // max_step_rate=0 (unlimited)

    if (hasIndex && indexPin >= 0) {
        indexInterrupt = new ModuleInterrupt<Hardware_QEI>(
            indexPin,
            this,
            &Hardware_QEI::handleIndexInterrupt
        );

        gpio_init(indexPin);
        gpio_set_dir(indexPin, GPIO_IN);
        if (modifier == PULLUP || modifier == OPENDRAIN) {
            gpio_pull_up(indexPin);
        } else if (modifier == PULLDOWN) {
            gpio_pull_down(indexPin);
        } else {
            gpio_disable_pulls(indexPin);
        }
        gpio_set_irq_enabled(indexPin, GPIO_IRQ_EDGE_RISE, true);
    }
    printf("Hardware_QEI: PIO%u SM%u loaded at offset %u\n",
           pioInst == pio0 ? 0 : 1, smNum, pioOffset);
#else
    printf("Hardware_QEI: PIO program header not included — "
           "define HAS_QUADRATURE_ENCODER_PIO_H and include the generated header.\n");
#endif
}

uint32_t Hardware_QEI::get() {
#ifdef HAS_QUADRATURE_ENCODER_PIO_H
    return (uint32_t)quadrature_encoder_get_count(pioInst, smNum);
#else
    return 0;
#endif
}

void Hardware_QEI::handleIndexInterrupt() {
    indexDetected = true;
    indexCount    = (int32_t)get();
}
