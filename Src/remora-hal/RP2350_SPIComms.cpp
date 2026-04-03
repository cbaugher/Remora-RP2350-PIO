#include "RP2350_SPIComms.h"
#include "irqHandlers.h"
#include "remora-core/configuration.h"
#include "hardware/irq.h"
#include <cstring>
#include <cstdio>

RP2350_SPIComms* RP2350_SPIComms::instance = nullptr;
volatile uint8_t RP2350_SPIComms::RxDMAmemoryIdx = 0;

static DMA_RxBuffer_t s_rxDMABuffer;

RP2350_SPIComms::RP2350_SPIComms(volatile rxData_t* ptrRxData,
                                   volatile txData_t* ptrTxData,
                                   int mosiGpio, int misoGpio,
                                   int clkGpio,  int csGpio)
    : gpioMosi(mosiGpio), gpioMiso(misoGpio),
      gpioClk(clkGpio), gpioCs(csGpio),
      dmaRxChanA(-1), dmaRxChanB(-1), dmaTxChan(-1), dmaMemCpyChan(-1),
      ptrRxDMABuffer(&s_rxDMABuffer),
      RXbufferIdx(0), copyRXbuffer(false), newWriteData(false),
      nssInterrupt(nullptr), dmaRxInterrupt(nullptr)
{
    this->ptrRxData = ptrRxData;
    this->ptrTxData = ptrTxData;
    instance = this;
}

// Determine SPI instance from MOSI GPIO number.
// SPI0 MOSI: GP3, GP7, GP19, GP23 (and mirror range)
// SPI1 MOSI: GP11, GP15, GP27
static spi_inst_t* spiInstFromMosi(int mosi) {
    // Conservative heuristic: SPI0 on lower GPIOs, SPI1 on higher
    if (mosi == 3 || mosi == 7 || mosi == 19 || mosi == 23) return spi0;
    if (mosi == 11 || mosi == 15 || mosi == 27)              return spi1;
    // Fallback based on range (GP0–GP15 → spi0, GP16–GP29 → depends)
    return (mosi < 12) ? spi0 : spi1;
}

void RP2350_SPIComms::initDMA() {
    dmaRxChanA    = dma_claim_unused_channel(true);
    dmaRxChanB    = dma_claim_unused_channel(true);
    dmaTxChan     = dma_claim_unused_channel(true);
    dmaMemCpyChan = dma_claim_unused_channel(true);

    // ---- RX channel A → rxDMABuffer.buffer[0], chains to B ----
    dma_channel_config cfgA = dma_channel_get_default_config(dmaRxChanA);
    channel_config_set_transfer_data_size(&cfgA, DMA_SIZE_8);
    channel_config_set_dreq(&cfgA, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgA, false);   // SPI DR — fixed
    channel_config_set_write_increment(&cfgA, true);   // into buffer[0]
    channel_config_set_chain_to(&cfgA, dmaRxChanB);
    channel_config_set_irq_quiet(&cfgA, false);        // fire IRQ on completion
    dma_channel_configure(dmaRxChanA, &cfgA,
        (void*)ptrRxDMABuffer->buffer[0].rxBuffer,
        &spi_get_hw(spiInst)->dr,
        Config::dataBuffSize,
        false);

    // ---- RX channel B → rxDMABuffer.buffer[1], chains back to A ----
    dma_channel_config cfgB = dma_channel_get_default_config(dmaRxChanB);
    channel_config_set_transfer_data_size(&cfgB, DMA_SIZE_8);
    channel_config_set_dreq(&cfgB, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgB, false);
    channel_config_set_write_increment(&cfgB, true);
    channel_config_set_chain_to(&cfgB, dmaRxChanA);
    channel_config_set_irq_quiet(&cfgB, false);
    dma_channel_configure(dmaRxChanB, &cfgB,
        (void*)ptrRxDMABuffer->buffer[1].rxBuffer,
        &spi_get_hw(spiInst)->dr,
        Config::dataBuffSize,
        false);

    // ---- TX channel — self-chaining circular (transmits txData repeatedly) ----
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, spi_get_dreq(spiInst, true));
    channel_config_set_read_increment(&cfgTx, true);
    channel_config_set_write_increment(&cfgTx, false);  // SPI DR — fixed
    channel_config_set_chain_to(&cfgTx, dmaTxChan);     // self-chain = circular
    dma_channel_configure(dmaTxChan, &cfgTx,
        &spi_get_hw(spiInst)->dr,
        (void*)ptrTxData->txBuffer,
        Config::dataBuffSize,
        false);

    // Enable DMA completion interrupts for both RX channels on IRQ0
    dma_channel_set_irq0_enabled(dmaRxChanA, true);
    dma_channel_set_irq0_enabled(dmaRxChanB, true);
}

void RP2350_SPIComms::init() {
    printf("RP2350_SPIComms::init()\n");

    spiInst = spiInstFromMosi(gpioMosi);

    spi_init(spiInst, 0);                  // frequency set by master in slave mode
    spi_set_slave(spiInst, true);
    spi_set_format(spiInst, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(gpioMosi, GPIO_FUNC_SPI);
    gpio_set_function(gpioMiso, GPIO_FUNC_SPI);
    gpio_set_function(gpioClk,  GPIO_FUNC_SPI);
    gpio_set_function(gpioCs,   GPIO_FUNC_SPI);  // hardware NSS in slave mode

    printf("RP2350_SPIComms: SPI%u slave, MOSI=GP%d MISO=GP%d CLK=GP%d CS=GP%d\n",
           spiInst == spi0 ? 0u : 1u, gpioMosi, gpioMiso, gpioClk, gpioCs);

    initDMA();

    // Register DMA RX completion interrupt (slot DMA_IRQ_0 = 11)
    dmaRxInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        DMA_IRQ_0,
        this,
        &RP2350_SPIComms::handleDmaRxInterrupt
    );
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_priority(DMA_IRQ_0, 64);   // lower than motion threads
    irq_set_enabled(DMA_IRQ_0, true);

    // Register CS rising-edge interrupt — use GPIO pin number as the slot index
    nssInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        gpioCs,
        this,
        &RP2350_SPIComms::handleNssInterrupt
    );
    gpio_set_irq_enabled(gpioCs, GPIO_IRQ_EDGE_RISE, true);
}

void RP2350_SPIComms::start() {
    memset((void*)ptrRxDMABuffer->buffer[0].rxBuffer, 0, Config::dataBuffSize);
    memset((void*)ptrRxDMABuffer->buffer[1].rxBuffer, 0, Config::dataBuffSize);
    dma_channel_start(dmaRxChanA);
    dma_channel_start(dmaTxChan);
    printf("RP2350_SPIComms: DMA started\n");
}

void RP2350_SPIComms::handleDmaRxInterrupt() {
    if (dma_channel_get_irq0_status(dmaRxChanA)) {
        dma_channel_acknowledge_irq0(dmaRxChanA);
        RxDMAmemoryIdx = 0;
        checkHeader();
    } else if (dma_channel_get_irq0_status(dmaRxChanB)) {
        dma_channel_acknowledge_irq0(dmaRxChanB);
        RxDMAmemoryIdx = 1;
        checkHeader();
    }
}

void RP2350_SPIComms::checkHeader() {
    int32_t hdr = ptrRxDMABuffer->buffer[RxDMAmemoryIdx].header;
    if (hdr == Config::pruRead || hdr == Config::pruWrite) {
        dataCallback(true);
        if (hdr == Config::pruWrite) {
            newWriteData  = true;
            RXbufferIdx   = RxDMAmemoryIdx;
        }
    } else {
        dataCallback(false);
    }
}

void RP2350_SPIComms::handleNssInterrupt() {
    if (newWriteData) {
        copyRXbuffer = true;
        newWriteData = false;
    }
}

void RP2350_SPIComms::tasks() {
    if (!copyRXbuffer) return;

    auto* src  = (uint8_t*)ptrRxDMABuffer->buffer[RXbufferIdx].rxBuffer;
    auto* dest = (uint8_t*)ptrRxData->rxBuffer;

    uint32_t irq_state = save_and_disable_interrupts();

    // mem-to-mem DMA copy (no DREQ — runs at system clock speed)
    dma_channel_config cfg = dma_channel_get_default_config(dmaMemCpyChan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    dma_channel_configure(dmaMemCpyChan, &cfg,
        dest, src,
        Config::dataBuffSize / 4,   // 64 bytes / 4 = 16 × 32-bit words
        true);
    dma_channel_wait_for_finish_blocking(dmaMemCpyChan);

    restore_interrupts(irq_state);
    copyRXbuffer = false;
}
