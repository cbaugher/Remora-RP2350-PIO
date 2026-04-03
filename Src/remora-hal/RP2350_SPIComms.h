#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "remora-core/comms/commsInterface.h"
#include "remora-core/modules/moduleInterrupt.h"
#include "remora-core/data.h"

// SPI slave communications — RP2350 port of STM32F4_SPIComms.
//
// The RP2350 SPI peripheral can operate as a hardware slave.  Double-buffered
// DMA is implemented using two chained DMA channels (A→B→A), which is the
// RP2350 equivalent of the STM32's HAL_DMAEx_MultiBufferStart_IT() hardware
// double-buffer mode.
//
// Data flow:
//   Master (RPi) clocks data in → SPI DR → DMA channel A → rxDMABuffer[0]
//   Channel A completes → chains to channel B → rxDMABuffer[1]
//   Channel B completes → chains back to A  (ping-pong forever)
//   On each DMA completion the DMA IRQ fires → handleDmaRxInterrupt()
//     → checkHeader() → dataCallback(valid)
//   On CS rising edge → handleNssInterrupt()
//     → schedules copy of the last-written buffer to ptrRxData (tasks())

class RP2350_SPIComms : public CommsInterface {
private:
    spi_inst_t* spiInst;

    int gpioMosi;
    int gpioMiso;
    int gpioClk;
    int gpioCs;

    // DMA channels
    int dmaRxChanA;     // RX channel A → rxDMABuffer[0]
    int dmaRxChanB;     // RX channel B → rxDMABuffer[1]
    int dmaTxChan;      // TX channel (self-chaining circular)
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
