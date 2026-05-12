#pragma once
#include <memory>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "remora-core/comms/commsInterface.h"
#include "remora-core/data.h"
#include "remora-hal/pin/pin.h"

// Ethernet communications (W5500 SPI master) — RP2350 port of STM32F4_EthComms.
//
// The W5500 driver in remora-core/drivers/W5500_Networking/ delegates every SPI
// byte and bulk transfer through the abstract CommsInterface methods (read_byte,
// write_byte, DMA_write, DMA_read).  This class implements those using the
// pico-sdk SPI master API.  The driver itself is completely unchanged.
//
// CS and RST are managed as plain GPIO outputs (not hardware NSS) because the
// W5500 framing protocol uses the CS line for message boundaries, not just for
// SPI enable.
//
// DMA_write and DMA_read use blocking DMA transfers so the calling code sees a
// synchronous interface.  No DMA interrupt registration is required for the ETH
// path.

class RP2350_EthComms : public CommsInterface {
private:
    spi_inst_t* spiInst;

    int gpioMosi;
    int gpioMiso;
    int gpioClk;
    int gpioCs;
    int gpioRst;

    int dmaTxChan;
    int dmaRxChan;

    bool newDataFlagged;

    // Pin objects for W5500 CS and RST — stored as members so they outlive init().
    // W5500_Networking uses ptr_csPin / ptr_rstPin throughout its lifetime.
    std::unique_ptr<Pin> ethCsPin;
    std::unique_ptr<Pin> ethRstPin;

    spi_inst_t* spiInstFromMosi(int mosi);

public:
    RP2350_EthComms(volatile rxData_t* ptrRxData, volatile txData_t* ptrTxData,
                    int mosiGpio, int misoGpio, int clkGpio,
                    int csGpio, int rstGpio);

    void init()   override;
    void start()  override;
    void tasks()  override;

    // W5500 driver calls these for byte-at-a-time register access
    uint8_t read_byte();
    uint8_t write_byte(uint8_t b);

    // W5500 driver calls these for bulk socket buffer transfers
    void DMA_write(uint8_t* data, uint16_t len);
    void DMA_read (uint8_t* data, uint16_t len);

    // Called by W5500 UDP receive callback when a valid packet arrives
    void flag_new_data();
};
