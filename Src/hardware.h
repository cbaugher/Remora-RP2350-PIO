#ifndef HARDWARECONFIG_H_
#define HARDWARECONFIG_H_

#include <cstdint>

// STM32 CMSIS compatibility — moduleInterrupt.h uses IRQn_Type for IRQ numbers.
// On RP2350 IRQ numbers are plain unsigned integers; map the type accordingly.
typedef uint32_t IRQn_Type;

// RP2350 has 52 peripheral IRQ lines (IRQ0–IRQ51).
// remora-core's Interrupt class allocates a static vector table of this size.
// IRQ numbers used:
//   TIMER0_IRQ_0 (0)  — base thread alarm
//   TIMER0_IRQ_1 (1)  — servo thread alarm
//   TIMER1_IRQ_0 (8)  — serial thread alarm
//   DMA_IRQ_0    (11) — SPI slave RX DMA completion
//   DMA_IRQ_1    (12) — SPI slave TX DMA (if needed)
//   IO_IRQ_BANK0 (13) — all GPIO interrupts (CS rising-edge, QEI index)
//   GPIO pin numbers (0–29) — used as IRQ slot indices for ModuleInterrupt<T>
#define PERIPH_COUNT_IRQn 52

// ARM CMSIS intrinsic — single NOP instruction.
// SoftwareSerial uses __NOP() for bit-timing; pico-sdk doesn't define it.
#ifndef __NOP
#  define __NOP() __asm__ volatile ("nop")
#endif

// STM32 GPIO pull-mode constants used by remora-core QEI module.
// Values match the PULLUP/PULLDOWN/NONE defines in pin.h.
#ifndef GPIO_NOPULL
#  define GPIO_NOPULL   0
#  define GPIO_PULLUP   1
#  define GPIO_PULLDOWN 2
#endif

#endif // HARDWARECONFIG_H_
