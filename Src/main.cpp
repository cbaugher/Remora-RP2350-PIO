#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <memory>
#include <cstdio>

// remora-core
#include "remora-core/remora.h"
#include "remora-core/configuration.h"
#include "remora-core/data.h"
#include "remora-core/modules/comms/commsHandler.h"

// remora-hal
#include "remora-hal/board_led_status.h"
#include "remora-hal/RP2350_timer.h"
#include "irqHandlers.h"   // gpio_irq_dispatcher, dma_irq0_handler, dma_irq1_handler

#ifdef ETH_CTRL
#  include "remora-hal/RP2350_EthComms.h"
#else
#  include "remora-hal/RP2350_SPIComms.h"
#endif

#if defined(SPI_CTRL) && !defined(NO_FATFS_SPI)
// The no-OS-FatFS-SD-SPI-RPi-Pico library uses irq_set_exclusive_handler on
// whichever DMA IRQ it is assigned.  Our PIO SPI slave uses DMA_IRQ_0 the
// same way.  Direct FatFs to DMA_IRQ_1 so the two don't collide.
// This must be called before the Remora constructor, which reads the SD card.
extern "C" void set_spi_dma_irq_channel(bool useChannel1, bool shared);
#endif

// rxData and txData are defined in remora-core/remora.cpp.
extern volatile rxData_t rxData;
extern volatile txData_t txData;

// ---------------------------------------------------------------------------
// Dual-core comms isolation
// Core 1 owns all comms I/O (SPI slave or ETH).  Core 0 owns the real-time
// threads (Base 40 kHz, Servo 1 kHz) and the Remora state machine.
//
// g_commsHandler is set by main() before multicore_launch_core1() is called.
// The launch itself contains a memory barrier, so Core 1 sees the assignment.
// ---------------------------------------------------------------------------
static CommsHandler* g_commsHandler = nullptr;

static void core1_entry() {
    // Register as a multicore lockout victim on Core 1.
    // This allows Core 0 to park Core 1 via multicore_lockout_start_blocking()
    // during flash programming (store_json_in_flash runs on Core 0).
    multicore_lockout_victim_init();

    // Register the shared GPIO IRQ dispatcher on Core 1.
    // gpio_set_irq_callback() and irq_set_enabled(IO_IRQ_BANK0) are per-core:
    // calling them here affines the SPI CS-edge interrupt to Core 1.
    // GPIO interrupts for other pins (e.g. QEI index) that were enabled from
    // Core 0 remain affined to Core 0 via the PROC0_INTE registers.
    gpio_set_irq_callback(gpio_irq_dispatcher);
    irq_set_enabled(IO_IRQ_BANK0, true);

    // Initialise and start comms from Core 1 so that DMA_IRQ_0 and the SPI
    // CS GPIO interrupt are registered in Core 1's NVIC.
    g_commsHandler->init();
    g_commsHandler->start();

    printf("Core 1: comms running\n");

    // Run comms tasks forever — polling ETH or processing SPI DMA completions.
    while (true) {
        g_commsHandler->tasks();
    }
}

// ---------------------------------------------------------------------------
// main()
// ---------------------------------------------------------------------------
int main() {
    // Initialise stdio — USB CDC configured in CMakeLists.txt via
    // pico_enable_stdio_usb().  UART stdio is disabled.
    stdio_init_all();

    // Register the shared GPIO interrupt dispatcher before any ModuleInterrupt<T>
    // objects are created.  All GPIO bank-0 interrupts funnel through this single
    // callback; the GPIO number is used as the Interrupt vector-table slot index.
    gpio_set_irq_callback(gpio_irq_dispatcher);
    irq_set_enabled(IO_IRQ_BANK0, true);
    irq_set_priority(IO_IRQ_BANK0, 128);   // lower priority than motion threads

    // Brief startup delay so USB CDC has time to enumerate before first printf
    sleep_ms(2000);

    // Register Core 0 as a multicore lockout victim.
    // This allows Core 1's TFTP handler to park Core 0 via
    // multicore_lockout_start_blocking() during flash programming.
    multicore_lockout_victim_init();

    init_board_status_led("GP25");

    printf("\n=== Remora RP2350 ===\n");
    printf("CPU clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));
    printf("Build: %s\n",
#ifdef ETH_CTRL
        "ETH"
#else
        "SPI"
#endif
    );

    // ---------------------------------------------------------------------------
    // Construct the communications interface
    // ---------------------------------------------------------------------------
    std::unique_ptr<CommsInterface> commsInterface;

#ifdef ETH_CTRL
    commsInterface = std::make_unique<RP2350_EthComms>(
        &rxData, &txData,
        SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_CLK_GPIO,
        SPI_CS_GPIO, WIZ_RST_GPIO
    );
#else
    commsInterface = std::make_unique<RP2350_SPIComms>(
        &rxData, &txData,
        SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_CLK_GPIO, SPI_CS_GPIO
    );
#endif

    // CommsHandler wraps the transport and implements the 100-cycle watchdog.
    // It is registered as a servo-thread module by the Remora constructor.
    auto commsHandler = std::make_shared<CommsHandler>();
    commsHandler->setInterface(std::move(commsInterface));

    // ---------------------------------------------------------------------------
    // Construct timers
    //   Base thread   — TIMER0 alarm 0, 40 kHz, highest priority
    //   Servo thread  — TIMER0 alarm 1,  1 kHz
    //   Serial thread — TIMER1 alarm 0, 57.6 kHz (TMC UART bit clock)
    // ---------------------------------------------------------------------------
    auto baseTimer = std::make_unique<RP2350_timer>(
        0, false, Config::pruBaseFreq, nullptr, 0);

    auto servoTimer = std::make_unique<RP2350_timer>(
        1, false, Config::pruServoFreq, nullptr, 64);

    auto serialTimer = std::make_unique<RP2350_timer>(
        0, true, Config::pruSerialFreq, nullptr, 128);

    // ---------------------------------------------------------------------------
    // Construct the Remora engine and run
    // ---------------------------------------------------------------------------
#if defined(SPI_CTRL) && !defined(NO_FATFS_SPI)
    // Direct the FatFs SD-SPI library to use DMA_IRQ_1 exclusively.
    // Must be called before Remora() because the constructor reads the SD card,
    // which triggers my_spi_init() which claims the DMA IRQ.
    // Our PIO SPI slave claims DMA_IRQ_0 exclusively in RP2350_SPIComms::init();
    // if both claim DMA_IRQ_0 the pico-sdk panics with a double exclusive-handler error.
    set_spi_dma_irq_channel(true, false);
#endif
    // The Remora constructor:
    //   1. Creates pruThread objects and assigns the timers.
    //   2. Creates a JsonConfigHandler, which immediately loads and parses
    //      config (from flash for ETH builds, SD card for SPI builds).
    //      If no config is found it falls back to the built-in defaultConfig
    //      (4 Hz blink on the LED pin — proving Milestone 1).
    //   3. Registers commsHandler on the servo thread.
    //      (commsInterface->init() and start() are called by Core 1, not here)
    //
    // run() enters the state machine:
    //   ST_SETUP → ST_START (starts threads) → ST_IDLE (waits for comms)
    //
    // Even in ST_IDLE the servo thread ISR fires at 1 kHz, so the Blink module
    // from the defaultConfig will toggle GP25 at 4 Hz immediately after boot.
    Remora* remora = new Remora(
        commsHandler,
        std::move(baseTimer),
        std::move(servoTimer),
        std::move(serialTimer)
    );

    // ---------------------------------------------------------------------------
    // Launch Core 1 — comms isolation
    // Launched AFTER the Remora constructor so the SD card read (inside the
    // constructor's JsonConfigHandler) completes before Core 1 starts claiming
    // DMA channels and initialising the PIO SPI slave.  Core 0 enters the
    // run() state machine immediately; it sits in ST_IDLE until Core 1 signals
    // that comms are alive (commsHandler->getStatus() becomes true).
    // ---------------------------------------------------------------------------
    g_commsHandler = commsHandler.get();
    multicore_launch_core1(core1_entry);

    remora->run();   // never returns

    // Unreachable — suppress compiler warning
    return 0;
}
