#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "spi_slave.pio.h"
#include "remora-core/comms/commsInterface.h"
#include "hardware.h"
#include "remora-core/modules/moduleInterrupt.h"
#include "remora-core/data.h"

// SPI slave communications — PIO-based replacement for the defective RP2350 A2 PL022.
//
// The PL022 hardware SPI slave on RP2350 A2 silicon captures exactly 1 byte per
// CS assertion regardless of configuration.  This class replaces it with a PIO
// state machine (spi_slave.pio, SPI Mode-0, MSB-first) that directly samples
// GPIO pins, immune to PL022 bugs.
//
// PIO assignment: PIO0 SM0 (leaves PIO1/PIO2 free for 7× QEI decoders).
//
// Data flow (RX):
//   CS LOW → PIO clocks 64 bytes → DMA chan A fills buffer[0] → chains to B
//   DMA chan A completion IRQ → handleDmaRxInterrupt() → checkHeader()
//     → dataCallback(valid)
//   CS HIGH → handleNssInterrupt() → copy txData + re-arm TX DMA for next txn
//
// Data flow (TX):
//   On CS RISE, memcpy ptrTxData→s_txAligned, re-arm TX DMA.
//   TX DMA streams s_txAligned → PIO TX FIFO (high byte of txf word, MSB-first).
//   PIO pre-drives MISO bit7 after pull noblock, before first SCK edge.

class RP2350_SPIComms : public CommsInterface {
private:
    PIO  pio;
    uint pio_sm;
    uint pio_prog_offset;

    volatile rxData_t* ptrRxData;
    volatile txData_t* ptrTxData;

    int gpioMosi;
    int gpioMiso;
    int gpioClk;
    int gpioCs;

    // DMA channels
    int dmaRxChanA;     // RX channel A → rxDMABuffer[0], chains to B
    int dmaRxChanB;     // RX channel B → rxDMABuffer[1], chains to A
    int dmaTxChan;      // TX channel (re-armed each CS RISE)
    int dmaMemCpyChan;  // mem-to-mem copy channel

    volatile DMA_RxBuffer_t* ptrRxDMABuffer;

    uint8_t RXbufferIdx;
    bool    copyRXbuffer;
    bool    newWriteData;

    ModuleInterrupt<RP2350_SPIComms>* nssInterrupt;
    ModuleInterrupt<RP2350_SPIComms>* dmaRxInterrupt;

    void handleNssInterrupt();
    void handleDmaRxInterrupt();
    void initDMA();

public:
    static RP2350_SPIComms*  instance;
    static volatile uint8_t  RxDMAmemoryIdx;

    RP2350_SPIComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData,
                    int mosiGpio, int misoGpio, int clkGpio, int csGpio);

    void init()   override;
    void start()  override;
    void tasks()  override;

    void checkHeader();
};
