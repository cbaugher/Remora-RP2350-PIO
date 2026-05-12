#include "RP2350_EthComms.h"
#include "remora-core/configuration.h"
#include "hardware/irq.h"
#include <cstdio>
#include <cstring>

#ifdef ETH_CTRL
#include "remora-core/drivers/W5500_Networking/W5500_Networking.h"
#endif

spi_inst_t* RP2350_EthComms::spiInstFromMosi(int mosi) {
    if (mosi == 3 || mosi == 7 || mosi == 19 || mosi == 23) return spi0;
    if (mosi == 11 || mosi == 15 || mosi == 27)              return spi1;
    return (mosi < 12) ? spi0 : spi1;
}

RP2350_EthComms::RP2350_EthComms(volatile rxData_t* ptrRxData,
                                   volatile txData_t* ptrTxData,
                                   int mosiGpio, int misoGpio, int clkGpio,
                                   int csGpio, int rstGpio)
    : gpioMosi(mosiGpio), gpioMiso(misoGpio), gpioClk(clkGpio),
      gpioCs(csGpio), gpioRst(rstGpio),
      dmaTxChan(-1), dmaRxChan(-1),
      newDataFlagged(false)
{
    this->ptrRxData = ptrRxData;
    this->ptrTxData = ptrTxData;
}

void RP2350_EthComms::init() {
    spiInst = spiInstFromMosi(gpioMosi);

    // 4 MHz — conservative for flying-wire / module connections.
    // W5500 chip supports up to 80 MHz; increase once verified on a PCB.
    spi_init(spiInst, 4 * 1000 * 1000);
    spi_set_format(spiInst, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // Master mode is the default after spi_init()

    gpio_set_function(gpioMosi, GPIO_FUNC_SPI);
    gpio_set_function(gpioMiso, GPIO_FUNC_SPI);
    gpio_set_function(gpioClk,  GPIO_FUNC_SPI);

    // CS and RST: create Pin objects that outlive init() so W5500_Networking
    // can call ptr_csPin->set() / ptr_rstPin->set() throughout operation.
    // The Pin constructor calls gpio_init() + gpio_set_dir(OUT) + gpio_put(0).
    // Drive both high (deassert) immediately after construction.
    ethCsPin  = std::make_unique<Pin>("GP" + std::to_string(gpioCs),  OUTPUT);
    ethRstPin = std::make_unique<Pin>("GP" + std::to_string(gpioRst), OUTPUT);
    ethCsPin->set(true);    // deassert CS
    ethRstPin->set(true);   // deassert RST (active-low)

    dmaTxChan = dma_claim_unused_channel(true);
    dmaRxChan = dma_claim_unused_channel(true);

    printf("RP2350_EthComms: SPI%u master, MOSI=GP%d MISO=GP%d CLK=GP%d CS=GP%d RST=GP%d\n",
           spiInst == spi0 ? 0u : 1u,
           gpioMosi, gpioMiso, gpioClk, gpioCs, gpioRst);

#ifdef ETH_CTRL
    // Initialise the W5500 networking stack (LwIP + WIZnet driver).
    // Pass this as the CommsInterface for SPI callbacks, and the Pin objects
    // for CS and RST GPIO control.
    network::EthernetInit(this, ethCsPin.get(), ethRstPin.get());
#endif
}

void RP2350_EthComms::start() {
    // Ethernet polling is done in tasks(); no separate start action needed.
}

void RP2350_EthComms::tasks() {
#ifdef ETH_CTRL
    network::EthernetTasks();
#endif

    if (newDataFlagged) {
        newDataFlagged = false;
        int32_t hdr = ptrRxData->header;
        if (hdr == Config::pruRead || hdr == Config::pruWrite) {
            dataCallback(true);
        } else {
            dataCallback(false);
        }
    }
}

// ---------------------------------------------------------------------------
// W5500 low-level SPI byte accessors
// ---------------------------------------------------------------------------

uint8_t RP2350_EthComms::read_byte() {
    uint8_t tx = 0xFF;
    uint8_t rx = 0;
    spi_write_read_blocking(spiInst, &tx, &rx, 1);
    return rx;
}

uint8_t RP2350_EthComms::write_byte(uint8_t byte) {
    uint8_t rx = 0;
    spi_write_read_blocking(spiInst, &byte, &rx, 1);
    return rx;
}

// ---------------------------------------------------------------------------
// Bulk DMA transfers
// ---------------------------------------------------------------------------

void RP2350_EthComms::DMA_write(uint8_t* data, uint16_t len) {
    static uint8_t discard;

    // TX channel: push data into the SPI TX FIFO.
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, spi_get_dreq(spiInst, true));
    channel_config_set_read_increment(&cfgTx, true);
    channel_config_set_write_increment(&cfgTx, false);
    dma_channel_configure(dmaTxChan, &cfgTx,
        &spi_get_hw(spiInst)->dr, data, len, false);

    // RX channel: drain the SPI RX FIFO into a discard sink.
    // Every TX byte clocked out causes the W5500 to clock one byte back in.
    // Without this channel the RX FIFO fills up, stalls the SPI transfer, and
    // leaves stale bytes that corrupt subsequent byte-level spi_write_read_blocking
    // calls (observed: getVERSIONR returning wrong values after a DMA write).
    dma_channel_config cfgRx = dma_channel_get_default_config(dmaRxChan);
    channel_config_set_transfer_data_size(&cfgRx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgRx, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgRx, false);
    channel_config_set_write_increment(&cfgRx, false); // always overwrite discard
    dma_channel_configure(dmaRxChan, &cfgRx,
        &discard, &spi_get_hw(spiInst)->dr, len, false);

    dma_start_channel_mask((1u << dmaTxChan) | (1u << dmaRxChan));
    dma_channel_wait_for_finish_blocking(dmaRxChan);
}

void RP2350_EthComms::DMA_read(uint8_t* data, uint16_t len) {
    static uint8_t dummy = 0xFF;

    // TX channel: send dummy bytes to clock the SPI
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, spi_get_dreq(spiInst, true));
    channel_config_set_read_increment(&cfgTx, false);   // same dummy byte
    channel_config_set_write_increment(&cfgTx, false);
    dma_channel_configure(dmaTxChan, &cfgTx,
        &spi_get_hw(spiInst)->dr, &dummy, len, false);

    // RX channel: capture the incoming bytes
    dma_channel_config cfgRx = dma_channel_get_default_config(dmaRxChan);
    channel_config_set_transfer_data_size(&cfgRx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgRx, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgRx, false);
    channel_config_set_write_increment(&cfgRx, true);
    dma_channel_configure(dmaRxChan, &cfgRx,
        data, &spi_get_hw(spiInst)->dr, len, false);

    dma_start_channel_mask((1u << dmaTxChan) | (1u << dmaRxChan));
    dma_channel_wait_for_finish_blocking(dmaRxChan);
}

void RP2350_EthComms::flag_new_data() {
    newDataFlagged = true;
}
