#include "RP2350_SPIComms.h"
#include "irqHandlers.h"
#include "hardware/sync.h"
#include "hardware/irq.h"
#include "remora-core/configuration.h"
#include <cstring>
#include <cstdio>

RP2350_SPIComms* RP2350_SPIComms::instance = nullptr;
volatile uint8_t RP2350_SPIComms::RxDMAmemoryIdx = 0;

static DMA_RxBuffer_t s_rxDMABuffer;

// 64-byte-aligned TX shadow buffer.
// txData (from remora-core) is only aligned(32); we copy here and DMA from here
// to the PIO TX FIFO high byte, giving MSB-first SHIFT_LEFT behaviour.
static uint8_t __attribute__((aligned(64))) s_txAligned[64];

RP2350_SPIComms::RP2350_SPIComms(volatile rxData_t* ptrRxData,
                                   volatile txData_t* ptrTxData,
                                   int mosiGpio, int misoGpio,
                                   int clkGpio,  int csGpio)
    : pio(pio0), pio_sm(0), pio_prog_offset(0),
      gpioMosi(mosiGpio), gpioMiso(misoGpio),
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

void RP2350_SPIComms::initDMA() {
    dmaRxChanA    = dma_claim_unused_channel(true);
    dmaRxChanB    = dma_claim_unused_channel(true);
    dmaTxChan     = dma_claim_unused_channel(true);
    dmaMemCpyChan = dma_claim_unused_channel(true);

    uint rxDreq = pio_get_dreq(pio, pio_sm, false);   // RX DREQ
    uint txDreq = pio_get_dreq(pio, pio_sm, true);    // TX DREQ

    // RX FIFO: byte 0 (LSB) of the 32-bit word holds ISR[7:0] = received byte.
    volatile uint8_t* rxFifoAddr = (volatile uint8_t*)&pio->rxf[pio_sm];
    // TX FIFO: byte 3 (MSB) so that SHIFT_LEFT outputs bit31 first = MSB of byte.
    volatile uint8_t* txFifoAddr = ((volatile uint8_t*)&pio->txf[pio_sm]) + 3;

    // ---- RX channel A → buffer[0], chains to B ----
    dma_channel_config cfgA = dma_channel_get_default_config(dmaRxChanA);
    channel_config_set_transfer_data_size(&cfgA, DMA_SIZE_8);
    channel_config_set_dreq(&cfgA, rxDreq);
    channel_config_set_read_increment(&cfgA, false);
    channel_config_set_write_increment(&cfgA, true);
    channel_config_set_chain_to(&cfgA, dmaRxChanB);
    channel_config_set_irq_quiet(&cfgA, false);
    dma_channel_configure(dmaRxChanA, &cfgA,
        (void*)ptrRxDMABuffer->buffer[0].rxBuffer,
        (void*)rxFifoAddr,
        Config::dataBuffSize,
        false);

    // ---- RX channel B → buffer[1], chains back to A ----
    dma_channel_config cfgB = dma_channel_get_default_config(dmaRxChanB);
    channel_config_set_transfer_data_size(&cfgB, DMA_SIZE_8);
    channel_config_set_dreq(&cfgB, rxDreq);
    channel_config_set_read_increment(&cfgB, false);
    channel_config_set_write_increment(&cfgB, true);
    channel_config_set_chain_to(&cfgB, dmaRxChanA);
    channel_config_set_irq_quiet(&cfgB, false);
    dma_channel_configure(dmaRxChanB, &cfgB,
        (void*)ptrRxDMABuffer->buffer[1].rxBuffer,
        (void*)rxFifoAddr,
        Config::dataBuffSize,
        false);

    // ---- TX channel — streams s_txAligned once per transaction ----
    // Re-armed (with fresh txData copy) on each CS RISING edge.
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, txDreq);
    channel_config_set_read_increment(&cfgTx, true);
    channel_config_set_write_increment(&cfgTx, false);
    dma_channel_configure(dmaTxChan, &cfgTx,
        (void*)txFifoAddr,
        s_txAligned,
        Config::dataBuffSize,
        false);

    // Enable completion IRQ for both RX channels on DMA_IRQ_0
    dma_channel_set_irq0_enabled(dmaRxChanA, true);
    dma_channel_set_irq0_enabled(dmaRxChanB, true);
}

void RP2350_SPIComms::init() {
    // ---- Load PIO program ----
    pio            = pio0;
    pio_prog_offset = pio_add_program(pio, &spi_slave_mode0_program);
    pio_sm         = pio_claim_unused_sm(pio, true);

    pio_sm_config c = spi_slave_mode0_program_get_default_config(pio_prog_offset);

    // Pin roles
    sm_config_set_out_pins(&c, gpioMiso, 1);   // OUT pin 0 = MISO (driven by PIO)
    sm_config_set_in_pins(&c, gpioMosi);        // IN  pin 0 = MOSI (sampled by PIO)
    sm_config_set_jmp_pin(&c, gpioCs);          // JMP pin   = CSn  (HIGH=idle)

    // Out shift: LEFT (MSB first), no autopull — explicit pull noblock in program
    // In  shift: LEFT (MSB first), no autopush — explicit push noblock in program.
    //   autopush uses IFFULL=0 semantics: even if enabled, the explicit
    //   'push noblock' (which is IFFULL=0,NOBLOCK=1) would push a second zero
    //   word after autopush cleared the ISR, producing alternating valid/0x00
    //   bytes in the DMA buffer.  Disabling autopush makes push noblock the
    //   sole mechanism: it fires unconditionally after exactly 8 'in pins,1',
    //   so ISR holds the received byte and exactly one word goes to the FIFO.
    sm_config_set_out_shift(&c, false, false, 32);
    sm_config_set_in_shift(&c, false, false, 32);

    // Run at system clock (150 MHz); Pi SPI SCK ≈ 1–8 MHz — ample margin
    sm_config_set_clkdiv(&c, 1.0f);

    // ---- GPIO setup ----
    // MISO: PIO-controlled output
    gpio_set_function(gpioMiso, GPIO_FUNC_PIO0);
    pio_sm_set_consecutive_pindirs(pio, pio_sm, gpioMiso, 1, true);

    // MOSI, SCK, CSn: SIO inputs readable by PIO (wait gpio / in pins / jmp pin
    // all read the raw GPIO input bus regardless of function).
    gpio_init(gpioMosi);
    gpio_set_dir(gpioMosi, GPIO_IN);
    gpio_pull_up(gpioMosi);   // float → 0xFF when disconnected

    gpio_init(gpioClk);
    gpio_set_dir(gpioClk, GPIO_IN);
    gpio_pull_down(gpioClk);  // Mode 0: SCK idles LOW; pull-down prevents false edges

    gpio_init(gpioCs);
    gpio_set_dir(gpioCs, GPIO_IN);
    gpio_pull_up(gpioCs);     // deasserted (high) when master absent

    // ---- Initialise SM, park at wait_cs ----
    pio_sm_init(pio, pio_sm,
                pio_prog_offset + spi_slave_mode0_offset_wait_cs,
                &c);
    // SM enabled in start() after DMA is armed

    printf("RP2350_SPIComms: PIO%u SM%u offset=%u MISO=GP%d MOSI=GP%d CLK=GP%d CS=GP%d\n",
           pio_get_index(pio), pio_sm, pio_prog_offset,
           gpioMiso, gpioMosi, gpioClk, gpioCs);

    initDMA();

    // DMA RX completion interrupt
    dmaRxInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        DMA_IRQ_0,
        this,
        &RP2350_SPIComms::handleDmaRxInterrupt
    );
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_priority(DMA_IRQ_0, 64);
    irq_set_enabled(DMA_IRQ_0, true);

    // CS edge interrupt
    nssInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        gpioCs,
        this,
        &RP2350_SPIComms::handleNssInterrupt
    );
    gpio_set_irq_enabled(gpioCs, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
}

void RP2350_SPIComms::start() {
    memset((void*)ptrRxDMABuffer->buffer[0].rxBuffer, 0, Config::dataBuffSize);
    memset((void*)ptrRxDMABuffer->buffer[1].rxBuffer, 0, Config::dataBuffSize);

    // Pre-load TX shadow buffer so the first transaction has a valid response.
    memcpy(s_txAligned, (const void*)ptrTxData->txBuffer, Config::dataBuffSize);

    // Pre-arm TX DMA — fills PIO TX FIFO before the first CS assertion.
    // Subsequent re-arms happen on each CS RISING edge (handleNssInterrupt).
    dma_channel_set_read_addr(dmaTxChan, s_txAligned, false);
    dma_channel_set_trans_count(dmaTxChan, Config::dataBuffSize, false);
    dma_channel_start(dmaTxChan);

    // Wait for CS to be HIGH (between transactions) before starting the SM.
    // If the Pi is already mid-transaction when the SM enables, the first
    // wait_cs would pass immediately and the PIO would start mid-byte.
    // Synchronising here ensures the SM catches a clean CS LOW → transaction start.
    {
        absolute_time_t dl = make_timeout_time_ms(2000);
        while (!gpio_get(gpioCs) && !time_reached(dl)) tight_loop_contents();
        if (time_reached(dl))
            printf("RP2350_SPIComms: warning — CS never went HIGH during start()\n");
    }

    // Start RX ping-pong (chan A first; chains to B on completion)
    dma_channel_start(dmaRxChanA);

    // Enable PIO SM — begins waiting at wait_cs for CS LOW
    pio_sm_set_enabled(pio, pio_sm, true);

    printf("RP2350_SPIComms: PIO SM started, DMA RX A=%d B=%d TX=%d\n",
           dmaRxChanA, dmaRxChanB, dmaTxChan);
}

void RP2350_SPIComms::handleDmaRxInterrupt() {
    if (dma_channel_get_irq0_status(dmaRxChanA)) {
        dma_channel_acknowledge_irq0(dmaRxChanA);
        // Reset write addr and count so chan A is ready when chan B chains back.
        // Without this, WRITE_ADDR stays at buffer[0]+64 and TRANS_COUNT wraps
        // to 2^32, causing a DMA runaway on the 3rd packet.
        dma_channel_set_write_addr(dmaRxChanA,
            (void*)ptrRxDMABuffer->buffer[0].rxBuffer, false);
        dma_channel_set_trans_count(dmaRxChanA, Config::dataBuffSize, false);
        RxDMAmemoryIdx = 0;
        checkHeader();
    } else if (dma_channel_get_irq0_status(dmaRxChanB)) {
        dma_channel_acknowledge_irq0(dmaRxChanB);
        dma_channel_set_write_addr(dmaRxChanB,
            (void*)ptrRxDMABuffer->buffer[1].rxBuffer, false);
        dma_channel_set_trans_count(dmaRxChanB, Config::dataBuffSize, false);
        RxDMAmemoryIdx = 1;
        checkHeader();
    }
}

void RP2350_SPIComms::checkHeader() {
    uint32_t hdr = (uint32_t)ptrRxDMABuffer->buffer[RxDMAmemoryIdx].header;

    if (hdr == Config::pruRead || hdr == Config::pruWrite) {
        dataCallback(true);
        if (hdr == Config::pruWrite) {
            newWriteData = true;
            RXbufferIdx  = RxDMAmemoryIdx;
        }
    } else {
        dataCallback(false);
    }
}

void RP2350_SPIComms::handleNssInterrupt() {
    if (gpio_get(gpioCs) == 1) {
        // CS RISING edge — transaction ended.
        //
        // BCM2835 SPI holds CS low for ~1 SCK period after the final clock edge.
        // During that window the SM wraps to wait_cs, finds CSN=0, falls through,
        // and starts a phantom byte-65: pull noblock gets 0x00000000 (FIFO empty),
        // drives MISO=0, then stalls at 'wait 1 gpio SCK'.  On the NEXT transaction
        // those stalled SCK pulses complete the phantom byte, shifting every real
        // TX byte one position later on the wire.
        //
        // Injecting a JMP cancels the stall immediately and parks the SM at
        // wait_cs (which blocks on 'wait 0 gpio CSN' while CS is HIGH).
        pio_sm_exec(pio, pio_sm,
            pio_encode_jmp(pio_prog_offset + spi_slave_mode0_offset_wait_cs));

        // Copy fresh txData and re-arm TX DMA during the ~850µs CS HIGH gap
        // so the PIO TX FIFO is pre-filled before the next CS FALL.
        memcpy(s_txAligned, (const void*)ptrTxData->txBuffer, Config::dataBuffSize);
        dma_channel_abort(dmaTxChan);
        dma_channel_set_read_addr(dmaTxChan, s_txAligned, false);
        dma_channel_set_trans_count(dmaTxChan, Config::dataBuffSize, false);
        dma_channel_start(dmaTxChan);

        if (newWriteData) {
            copyRXbuffer = true;
            newWriteData = false;
        }
    }
    // CS FALLING edge: nothing needed — TX FIFO already pre-filled from CS RISE,
    // RX ping-pong is running, PIO SM transitions automatically from wait_cs.
}

void RP2350_SPIComms::tasks() {
    if (!copyRXbuffer) return;

    auto* src  = (uint8_t*)ptrRxDMABuffer->buffer[RXbufferIdx].rxBuffer;
    auto* dest = (uint8_t*)ptrRxData->rxBuffer;

    uint32_t irq_state = save_and_disable_interrupts();

    dma_channel_config cfg = dma_channel_get_default_config(dmaMemCpyChan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    dma_channel_configure(dmaMemCpyChan, &cfg,
        dest, src,
        Config::dataBuffSize / 4,
        true);
    dma_channel_wait_for_finish_blocking(dmaMemCpyChan);

    restore_interrupts(irq_state);
    copyRXbuffer = false;

}
