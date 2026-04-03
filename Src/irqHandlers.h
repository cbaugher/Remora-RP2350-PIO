#pragma once

// RP2350 ISR glue — routes hardware interrupts into remora-core's dispatch table.
// Include this header exactly once, from main.cpp, after hardware.h.

#include "remora-core/interrupt/interrupt.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

// ---------------------------------------------------------------------------
// GPIO interrupt dispatcher
// All GPIO bank-0 interrupts share a single IRQ line (IO_IRQ_BANK0 = 13).
// The pico-sdk demultiplexes to this callback with the triggering GPIO number.
// We use the GPIO number directly as the Interrupt vector-table slot index —
// all GP0–GP29 fit within PERIPH_COUNT_IRQn = 52.
// ---------------------------------------------------------------------------
static void gpio_irq_dispatcher(uint gpio, uint32_t events) {
    (void)events;
    Interrupt::InvokeHandler(gpio);
}

// ---------------------------------------------------------------------------
// DMA IRQ 0 — SPI slave RX double-buffer completion (channels A and B)
// DMA_IRQ_0 = 11
// ---------------------------------------------------------------------------
static void __isr dma_irq0_handler() {
    Interrupt::InvokeHandler(DMA_IRQ_0);
    // Clear all pending channel interrupts on IRQ0 line
    dma_hw->ints0 = dma_hw->ints0;
}

// ---------------------------------------------------------------------------
// DMA IRQ 1 — available for future use (e.g. second SPI or ETH DMA)
// DMA_IRQ_1 = 12
// ---------------------------------------------------------------------------
static void __isr dma_irq1_handler() {
    Interrupt::InvokeHandler(DMA_IRQ_1);
    dma_hw->ints1 = dma_hw->ints1;
}
