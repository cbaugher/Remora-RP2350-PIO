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

// remora-hal
#include "remora-hal/board_led_status.h"
#include "remora-hal/RP2350_timer.h"
#include "irqHandlers.h"   // gpio_irq_dispatcher, dma_irq0_handler, dma_irq1_handler

#ifdef ETH_CTRL
#  include "remora-hal/RP2350_EthComms.h"
#else
#  include "remora-hal/RP2350_SPIComms.h"
#endif

// ---------------------------------------------------------------------------
// Global data buffers (aligned as required by the 64-byte packed protocol)
// ---------------------------------------------------------------------------
// These are declared extern in remora-core/data.h and referenced throughout
// the module system.  They are defined here — the single translation unit that
// owns the MCU-side runtime state.
volatile rxData_t rxData;
volatile txData_t txData;

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
    auto commsHandler = std::make_shared<CommsHandler>(std::move(commsInterface));

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
    // The Remora constructor:
    //   1. Creates pruThread objects and assigns the timers.
    //   2. Creates a JsonConfigHandler, which immediately loads and parses
    //      config (from flash for ETH builds, SD card for SPI builds).
    //      If no config is found it falls back to the built-in defaultConfig
    //      (4 Hz blink on the LED pin — proving Milestone 1).
    //   3. Registers commsHandler on the servo thread.
    //   4. Calls commsInterface->init() and commsInterface->start().
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

    remora->run();   // never returns

    // Unreachable — suppress compiler warning
    return 0;
}
