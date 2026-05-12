# Remora RP2350 Porting Plan

> Based on analysis of `Remora-STM32F4xx-PIO-main` and `remora-core-8d3b6090`  
> Target: Raspberry Pi RP2350 (Pico 2 or equivalent board)  
> Toolchain: CMake + pico-sdk  
> Strategy: Replace `remora-hal` only — `remora-core` is left entirely untouched

---

## Table of Contents

1. [Porting Strategy & Scope](#1-porting-strategy--scope)
2. [RP2350 Hardware Overview](#2-rp2350-hardware-overview)
3. [STM32F4 vs RP2350 Peripheral Mapping](#3-stm32f4-vs-rp2350-peripheral-mapping)
4. [Repository Structure](#4-repository-structure)
5. [Phase 1 — Build System & Project Skeleton](#5-phase-1--build-system--project-skeleton)
6. [Phase 2 — Pin Abstraction](#6-phase-2--pin-abstraction)
7. [Phase 3 — Timer & Thread System](#7-phase-3--timer--thread-system)
8. [Phase 4 — Interrupt Dispatch System](#8-phase-4--interrupt-dispatch-system)
9. [Phase 5 — SPI Slave Communications](#9-phase-5--spi-slave-communications)
10. [Phase 6 — Ethernet Communications (W5500)](#10-phase-6--ethernet-communications-w5500)
11. [Phase 7 — Hardware PWM](#11-phase-7--hardware-pwm)
12. [Phase 8 — Hardware Quadrature Encoder (QEI)](#12-phase-8--hardware-quadrature-encoder-qei)
13. [Phase 9 — Analog Input (ADC)](#13-phase-9--analog-input-adc)
14. [Phase 10 — Flash Storage & Platform Configuration](#14-phase-10--flash-storage--platform-configuration)
15. [Phase 11 — Supporting Components](#15-phase-11--supporting-components)
16. [Phase 12 — Serial Thread & TMC Stepper Drivers](#16-phase-12--serial-thread--tmc-stepper-drivers)
17. [Phase 13 — Dual-Core Architecture (Enhancement)](#17-phase-13--dual-core-architecture-enhancement)
18. [remora-core Bug Fixes to Apply During Port](#18-remora-core-bug-fixes-to-apply-during-port)
19. [File Change Summary](#19-file-change-summary)
20. [Implementation Sequence & Milestones](#20-implementation-sequence--milestones)

---

## 1. Porting Strategy & Scope

### The key insight: only the HAL needs to change

Remora's architecture divides cleanly into two layers:

- **`remora-core`** — Platform-agnostic real-time engine containing the state machine, thread system, module system, JSON configuration handler, communications protocol, and all motion-control module logic (Stepgen, QEI, PWM, etc.). This code has zero platform-specific includes and zero STM32 HAL calls.
- **`remora-hal`** — STM32F4-specific hardware drivers that implement the abstract interfaces (`pruTimer`, `CommsInterface`) and hardware-access classes (`Pin`, `HardwarePWM`, `Hardware_QEI`, `AnalogIn`) that remora-core's modules call into.

The porting work is therefore entirely confined to creating a new `remora-hal` directory for the RP2350, structured identically to the STM32F4 one. The remora-core submodule is added unchanged as a Git submodule. The LinuxCNC host-side components (`remora-eth-3.0.c`, `remora-spi.c`) are also unaffected — the protocol and packet format don't change.

### What does and does not change

**Completely unchanged (zero modifications):**
- All of `remora-core/` — state machine, modules, JSON handler, comms interfaces, interrupt dispatch, thread system, CRC32, TMCStepper library, W5500_Networking driver
- `data.h`, `configuration.h`, `remoraStatus.h`
- LinuxCNC host-side HAL components
- The 64-byte SPI/UDP packet protocol

**New files to write (the entire porting effort):**
- `CMakeLists.txt` (replaces `platformio.ini`)
- `Src/hardware.h` (updated `PERIPH_COUNT_IRQn`)
- `Src/main.cpp` (pico-sdk init sequence)
- `Src/irqHandlers.h` (RP2350 ISR glue)
- All files under `Src/remora-hal/` (12 new implementation files)

**remora-core bug fixes** (platform-agnostic, benefit all targets — submitted as separate PRs):
- Thermistor 16-bit ADC divisor
- O(n²) SD config string construction
- VLA stack allocation in SD loader
- Thread frequency setter

---

## 2. RP2350 Hardware Overview

The RP2350 is the second-generation Raspberry Pi microcontroller, used on the Pico 2 board. Key specifications relevant to this port:

| Feature | RP2350 | STM32F446 | Notes |
|---|---|---|---|
| Cores | 2× Cortex-M33 (or 2× RISC-V Hazard3) | 1× Cortex-M4F | Dual-core is a major advantage |
| Clock | 150 MHz stock, up to ~300 MHz | 168 MHz | Comparable |
| SRAM | 520 KB | 128 KB | 4× more — no heap pressure |
| Flash | External (4 MB on Pico 2) | Internal 512 KB | XIP via QSPI; write requires disabling XIP |
| Timers | 2× 64-bit timer blocks, 4 alarms each | Many TIM peripherals | RP2350 uses alarm-based repeating timers |
| PIO | 3 blocks × 4 state machines = 12 SMs | None | Replaces hardware QEI, custom protocols |
| DMA | 12 channels, chaining, endless mode | 16 DMA streams | Similar capability |
| SPI | 2× hardware SPI (master or slave) | SPI1/2/3 | Adequate |
| UART | 2× hardware UART | USART1-6 | Adequate |
| PWM | 12 slices × 2 channels = 24 outputs | Many TIM channels | All GPIOs can produce PWM |
| ADC | 1× 12-bit, 8 channels (500 kSPS) | 3× 12-bit ADC | Single ADC; channels GP26-GP29 on Pico |
| GPIO | 48 (RP2350B) / 30 (RP2350A) | Many (port-based) | Flat numbering: GP0–GP29/47 |
| USB | Native USB device/host | None on F446 | Free printf/debug over USB |

The RP2350's **PIO (Programmable I/O)** subsystem is the critical new capability for this port. It enables hardware-accelerated quadrature encoder decoding and custom serial protocols without consuming CPU cycles, directly replacing the STM32's dedicated timer-encoder mode.

---

## 3. STM32F4 vs RP2350 Peripheral Mapping

| Remora Function | STM32F4 Mechanism | RP2350 Mechanism |
|---|---|---|
| Base thread timer (40 kHz) | TIM3 update interrupt | TIMER0 Alarm 0 repeating timer |
| Servo thread timer (1 kHz) | TIM2 update interrupt | TIMER0 Alarm 1 repeating timer |
| Serial thread timer (57.6 kHz) | TIM4 update interrupt | TIMER1 Alarm 0 repeating timer |
| ISR vector table | NVIC (149 entries) | NVIC (52 entries) |
| GPIO output | `HAL_GPIO_WritePin()` | `gpio_put()` |
| GPIO input | `HAL_GPIO_ReadPin()` | `gpio_get()` |
| GPIO interrupt (CS/index) | EXTI lines → NVIC | `gpio_set_irq_enabled_with_callback()` → IO_IRQ_BANK0 |
| GPIO alternate function | `GPIO_InitTypeDef.Alternate` | `gpio_set_function()` |
| SPI slave (RPi comms) | SPI1/2/3 in slave mode + DMA | SPI0 or SPI1 in slave mode + DMA chaining |
| SPI master (W5500) | SPI1 in master mode + DMA | SPI1 in master mode + DMA |
| DMA double-buffer (SPI RX) | `HAL_DMAEx_MultiBufferStart_IT()` | Two chained DMA channels (channel A → B → A) |
| DMA mem-to-mem | DMA2_Stream1 | Any DMA channel, no DREQ |
| DMA circular TX | Circular DMA stream | Self-chaining or endless-mode DMA channel |
| Hardware PWM | TIMx channels (pin-map lookup) | `pwm_gpio_to_slice_num()` / `pwm_gpio_to_channel()` |
| Quadrature encoder | TIM8 encoder mode (hardware counter) | PIO quadrature encoder program (12 state machines available) |
| ADC | ADC1/2/3, per-peripheral handles | Single ADC, `adc_select_input()` + `adc_read()` |
| Flash erase | `HAL_FLASHEx_Erase()` by sector | `flash_range_erase()` by byte offset (4 KB pages) |
| Flash write | `HAL_FLASH_Program()` byte/word/halfword | `flash_range_program()` (256-byte pages) — must run from RAM |
| System reset | `HAL_NVIC_SystemReset()` | `watchdog_reboot(0, 0, 0)` or `*(uint32_t*)0x40058000 = 1` |
| Printf retarget | Override weak `_write()` → UART | `pico_enable_stdio_usb()` or `pico_enable_stdio_uart()` in CMake |
| Delay | `HAL_Delay(ms)` | `sleep_ms(ms)` |
| System clock frequency | `HAL_RCC_GetSysClockFreq()` | `clock_get_hz(clk_sys)` |
| SD card | SDIO peripheral + FatFs | SPI-based FatFs (`no-OS-FatFS-SD-SPI-RPi-Pico` library) |

---

## 4. Repository Structure

The new repository should be named `Remora-RP2350-PIO` and structured as follows:

```
Remora-RP2350-PIO/
├── CMakeLists.txt                      — Top-level CMake build file
├── pico_sdk_import.cmake               — Standard pico-sdk bootstrap script
├── README.md
├── lib/
│   └── no-OS-FatFS-SD-SPI-RPi-Pico/   — SPI FatFs library (git submodule, SPI builds only)
├── LinkerScripts/
│   └── rp2350_memmap_custom.ld         — Optional: if custom flash layout needed
├── LinuxCNC/                           — Unchanged from STM32 repo (symlink or copy)
│   ├── Components/remora-eth/
│   ├── Components/remora-spi/
│   └── Config/
└── Src/
    ├── main.cpp                        — Entry point (pico-sdk init sequence)
    ├── hardware.h                      — PERIPH_COUNT_IRQn = 52
    ├── irqHandlers.h                   — GPIO, DMA, and timer ISR glue
    └── remora-core/                    — Git submodule (remora-core, unchanged)
    └── remora-hal/
        ├── platform_configuration.h   — Flash offset constants (no linker symbols needed)
        ├── hal_utils.h                 — Macro wrappers + flash helpers
        ├── hal_utils.cpp
        ├── board_led_status.h          — LED error blink system (unchanged interface)
        ├── board_led_status.cpp        — HAL_Delay → sleep_ms
        ├── shared_handlers.h           — Stub (no shared handle pool needed on RP2350)
        ├── shared_handlers.cpp
        ├── pin/
        │   ├── pin.h                   — Unchanged interface
        │   └── pin.cpp                 — Parse "GP0" format, use gpio_init/put/get
        ├── analogIn/
        │   ├── analogIn.h              — Unchanged interface
        │   └── analogIn.cpp            — pico-sdk adc_init/adc_read
        ├── hardware_pwm/
        │   ├── hardware_pwm.h          — Updated: slice-based instead of TIM-based
        │   └── hardware_pwm.cpp        — pwm_gpio_to_slice_num, pwm_set_chan_level
        ├── hardware_qei/
        │   ├── hardware_qei.h          — Updated: PIO-based, configurable pins
        │   └── hardware_qei.cpp        — PIO quadrature encoder program
        ├── RP2350_timer.h              — pruTimer implementation (replaces STM32F4_timer)
        ├── RP2350_timer.cpp
        ├── RP2350_SPIComms.h           — CommsInterface: SPI slave (replaces STM32F4_SPIComms)
        ├── RP2350_SPIComms.cpp
        ├── RP2350_EthComms.h           — CommsInterface: W5500 master (replaces STM32F4_EthComms)
        └── RP2350_EthComms.cpp
```

Files not needed on RP2350 (can be omitted):
- `peripheralPins.h/.c` — mbed-style pin-map lookup tables. Not needed; pico-sdk derives PWM slice/channel from GPIO number algorithmically.
- `PinNamesTypes.h`, `pinNames.h` — STM32-specific PinName enum and bit-encoded function types. Not needed.

---

## 5. Phase 1 — Build System & Project Skeleton

### 5.1 CMakeLists.txt

The STM32 project used PlatformIO's `platformio.ini`. The RP2350 uses CMake with pico-sdk. The top-level `CMakeLists.txt` must:

```cmake
cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(remora_rp2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---- Common sources (remora-core + shared HAL) ----
set(REMORA_CORE_SOURCES
    Src/remora-core/remora.cpp
    Src/remora-core/comms/commsInterface.cpp
    Src/remora-core/thread/pruThread.cpp
    Src/remora-core/thread/pruTimer.cpp
    Src/remora-core/thread/timerInterrupt.cpp
    Src/remora-core/interrupt/interrupt.cpp
    Src/remora-core/modules/module.cpp
    Src/remora-core/modules/moduleFactory.cpp
    Src/remora-core/modules/comms/commsHandler.cpp
    Src/remora-core/modules/blink/blink.cpp
    Src/remora-core/modules/resetPin/resetPin.cpp
    Src/remora-core/modules/digitalPin/digitalPin.cpp
    Src/remora-core/modules/analogPin/analogPin.cpp
    Src/remora-core/modules/pwm/pwm.cpp
    Src/remora-core/modules/qei/qei.cpp
    Src/remora-core/modules/sigmaDelta/sigmaDelta.cpp
    Src/remora-core/modules/stepgen/stepgen.cpp
    Src/remora-core/modules/softEncoder/softEncoder.cpp
    Src/remora-core/modules/temperature/temperature.cpp
    Src/remora-core/modules/tmc/tmc2208.cpp
    Src/remora-core/modules/tmc/tmc2209.cpp
    Src/remora-core/modules/tmc/tmc5160.cpp
    Src/remora-core/json/jsonConfigHandler.cpp
    Src/remora-core/sensors/thermistor/thermistor.cpp
    Src/remora-core/drivers/SoftwareSPI/SoftwareSPI.cpp
    Src/remora-core/drivers/SoftwareSerial/SoftwareSerial.cpp
    # TMCStepper sources
    Src/remora-core/drivers/TMCStepper/TMCStepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC2208Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC2209Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC5160Stepper.cpp
    Src/remora-core/drivers/TMCStepper/CHOPCONF.cpp
    Src/remora-core/drivers/TMCStepper/COOLCONF.cpp
    Src/remora-core/drivers/TMCStepper/DRV_STATUS.cpp
    Src/remora-core/drivers/TMCStepper/GCONF.cpp
    Src/remora-core/drivers/TMCStepper/IHOLD_IRUN.cpp
    Src/remora-core/drivers/TMCStepper/PWMCONF.cpp
)

set(REMORA_HAL_SOURCES
    Src/main.cpp
    Src/remora-hal/pin/pin.cpp
    Src/remora-hal/analogIn/analogIn.cpp
    Src/remora-hal/hardware_pwm/hardware_pwm.cpp
    Src/remora-hal/hardware_qei/hardware_qei.cpp
    Src/remora-hal/board_led_status.cpp
    Src/remora-hal/hal_utils.cpp
    Src/remora-hal/shared_handlers.cpp
    Src/remora-hal/RP2350_timer.cpp
)

set(REMORA_INCLUDE_DIRS
    Src
    Src/remora-core
    Src/remora-hal
)

# ---- SPI build target ----
add_executable(remora_rp2350_spi
    ${REMORA_CORE_SOURCES}
    ${REMORA_HAL_SOURCES}
    Src/remora-hal/RP2350_SPIComms.cpp
)
target_include_directories(remora_rp2350_spi PRIVATE ${REMORA_INCLUDE_DIRS})
target_compile_definitions(remora_rp2350_spi PRIVATE
    SPI_CTRL=1
    SPI_CS_GPIO=13
)
target_link_libraries(remora_rp2350_spi
    pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_dma
    hardware_irq
    hardware_timer
    hardware_pwm
    hardware_adc
    hardware_pio
    hardware_flash
    hardware_sync
    pico_multicore
    FatFs_SPI         # SPI-based FatFs for SD card
)
pico_enable_stdio_usb(remora_rp2350_spi 1)
pico_enable_stdio_uart(remora_rp2350_spi 0)
pico_add_extra_outputs(remora_rp2350_spi)  # generates .uf2, .bin, .hex

# ---- ETH build target ----
add_executable(remora_rp2350_eth
    ${REMORA_CORE_SOURCES}
    ${REMORA_HAL_SOURCES}
    Src/remora-hal/RP2350_EthComms.cpp
    # W5500_Networking sources (from the ZIP library)
    Src/remora-core/drivers/W5500_Networking/W5500_Networking.cpp
)
target_include_directories(remora_rp2350_eth PRIVATE ${REMORA_INCLUDE_DIRS})
target_compile_definitions(remora_rp2350_eth PRIVATE
    ETH_CTRL=1
    _WIZCHIP_=5500
    WIZ_RST_GPIO=15
    SPI_CS_GPIO=17
    SPI_CLK_GPIO=18
    SPI_MISO_GPIO=16
    SPI_MOSI_GPIO=19
)
target_link_libraries(remora_rp2350_eth
    pico_stdlib
    hardware_gpio
    hardware_spi
    hardware_dma
    hardware_irq
    hardware_timer
    hardware_pwm
    hardware_adc
    hardware_pio
    hardware_flash
    hardware_sync
    pico_multicore
)
pico_enable_stdio_usb(remora_rp2350_eth 1)
pico_enable_stdio_uart(remora_rp2350_eth 0)
pico_add_extra_outputs(remora_rp2350_eth)
```

### 5.2 hardware.h

The `Interrupt` class uses a static vector table of size `PERIPH_COUNT_IRQn`, which is defined in `hardware.h`. The RP2350 has 52 external IRQs (IRQ numbers 0–51):

```cpp
// Src/hardware.h
#ifndef HARDWARECONFIG_H_
#define HARDWARECONFIG_H_

// RP2350 has 52 peripheral IRQ lines (IRQ0-IRQ51)
// This must match or exceed the highest IRQ number used by Interrupt::Register()
#define PERIPH_COUNT_IRQn 52

#endif
```

The most significant IRQ numbers used will be:
- `TIMER0_IRQ_0` (0) — base thread
- `TIMER0_IRQ_1` (1) — servo thread
- `TIMER1_IRQ_0` (8) — serial thread
- `DMA_IRQ_0` (11) — SPI DMA RX/TX
- `DMA_IRQ_1` (12) — second DMA channel
- `IO_IRQ_BANK0` (13) — all GPIO interrupts (CS pin, QEI index)

All fit comfortably within 52 slots.

### 5.3 Verification milestone

Before writing any functional code, the project should compile cleanly and produce a working `.uf2` that blinks the onboard LED (GP25) using only the `Blink` module driven by a basic `RP2350_timer` stub. This proves the CMake setup, remora-core compilation, and pico-sdk integration all work together.

---

## 6. Phase 2 — Pin Abstraction

**Files:** `Src/remora-hal/pin/pin.h` (interface unchanged), `Src/remora-hal/pin/pin.cpp` (complete rewrite)

### 6.1 Pin naming format

The STM32 version parses strings like `"PA_0"` (port letter + pin number). The RP2350 has a flat GPIO namespace: GP0 through GP29 (RP2350A / Pico 2) or GP47 (RP2350B). The new format is `"GP0"` through `"GP47"`.

The `pin.h` interface — constructors, `get()`, `set()`, `setAsOutput()`, `setAsInput()`, etc. — is **identical** to the STM32 version. Only `pin.cpp` changes.

### 6.2 configurePin()

```cpp
void Pin::configurePin() {
    if (portAndPin.size() >= 3 && portAndPin.substr(0, 2) == "GP") {
        gpioNum = std::stoi(portAndPin.substr(2));
        if (gpioNum < 0 || gpioNum > 47) {
            printf("Pin::configurePin() — invalid GPIO number in '%s'\n",
                   portAndPin.c_str());
            gpioNum = -1;
        }
    } else {
        printf("Pin::configurePin() — unrecognised pin format '%s' "
               "(expected 'GP0' through 'GP47')\n", portAndPin.c_str());
        gpioNum = -1;
    }
}
```

Replace the STM32 member variable `GPIOx` (pointer to GPIO port register block) and `pin` (bitmask) with a single `int gpioNum`.

### 6.3 enableClock()

GPIO clocks are always enabled on RP2350. Make `enableClock()` a no-op:

```cpp
void Pin::enableClock() {
    // No clock enable required on RP2350 — all GPIO always clocked
}
```

### 6.4 initialisePin() — general GPIO

```cpp
void Pin::initialisePin() {
    if (gpioNum < 0) return;
    gpio_init(gpioNum);
    if (mode == INPUT) {
        gpio_set_dir(gpioNum, GPIO_IN);
        if (modifier == PULLUP)        gpio_pull_up(gpioNum);
        else if (modifier == PULLDOWN) gpio_pull_down(gpioNum);
        else                           gpio_disable_pulls(gpioNum);
    } else {
        gpio_set_dir(gpioNum, GPIO_OUT);
        gpio_put(gpioNum, 0);
    }
}
```

### 6.5 initialiseGPIO() — alternate function pins

The STM32 version used `GPIO_InitTypeDef.Alternate` to set the alternate function number. On RP2350, the alternate function is set via `gpio_set_function()`. The `gpio_alt` member (which stored the STM32 AF number) is repurposed to hold the pico-sdk `gpio_function_t` enum value:

```cpp
void Pin::initialiseGPIO() {
    if (gpioNum < 0) return;
    gpio_init(gpioNum);
    // gpio_alt holds the gpio_function_t value (GPIO_FUNC_SPI, GPIO_FUNC_PWM, etc.)
    gpio_set_function(gpioNum, static_cast<gpio_function_t>(gpio_alt));
    if (gpio_pull == GPIO_PULLUP)        gpio_pull_up(gpioNum);
    else if (gpio_pull == GPIO_PULLDOWN) gpio_pull_down(gpioNum);
    else                                 gpio_disable_pulls(gpioNum);
}
```

The callers that use the alternate-function constructor (`createPinFromPinMap()` on STM32) are replaced on RP2350 by direct calls with the appropriate `gpio_function_t`. For example, SPI pins pass `GPIO_FUNC_SPI`; PWM pins pass `GPIO_FUNC_PWM`.

### 6.6 get() and set()

```cpp
bool Pin::get() const {
    if (gpioNum < 0) return false;
    return gpio_get(gpioNum);
}

void Pin::set(bool value) {
    if (gpioNum < 0) return;
    gpio_put(gpioNum, value ? 1 : 0);
}
```

### 6.7 hal_utils.h — remove pin-map dependencies

The STM32 `hal_utils.h` exposes `getSPIPeripheralName()`, `getPWMName()`, and `createPinFromPinMap()`, all of which depend on the mbed-style `peripheralPins.c` lookup tables. These are not needed on RP2350 (the pico-sdk derives everything from GPIO number algorithmically). Remove these functions from `hal_utils.h` and replace with the RP2350 equivalents described in later phases.

---

## 7. Phase 3 — Timer & Thread System

**Files:** `Src/remora-hal/RP2350_timer.h`, `Src/remora-hal/RP2350_timer.cpp`

### 7.1 Repeating timer approach

The STM32 used TIM2/TIM3 update interrupts. The RP2350 equivalent is the **hardware alarm** system inside its two 64-bit timer peripherals (`TIMER0` and `TIMER1`). Each timer has 4 alarm channels. The pico-sdk `add_repeating_timer_us()` API uses these alarms to fire a callback at precise intervals.

### 7.2 Class definition

```cpp
// RP2350_timer.h
#ifndef RP2350_TIMER_H
#define RP2350_TIMER_H

#include "hardware/timer.h"
#include "pico/time.h"
#include "../remora-core/thread/pruTimer.h"

class RP2350_timer : public pruTimer {
private:
    repeating_timer_t   timerState;
    uint                alarmNum;       // 0-3 within the timer block
    bool                useTimer1;      // false = TIMER0, true = TIMER1
    int                 irqPriority;    // 0 (highest) to 255 (lowest)

    static bool timerCallback(repeating_timer_t* rt) {
        auto* self = static_cast<RP2350_timer*>(rt->user_data);
        self->timerTick();
        return true;    // returning true keeps the repeating timer alive
    }

public:
    RP2350_timer(uint _alarmNum, bool _useTimer1,
                 uint32_t _frequency, pruThread* _ownerPtr,
                 int _irqPriority = 0);

    void configTimer() override;
    void startTimer()  override;
    void stopTimer()   override;
    void timerTick()   override;
};
#endif
```

### 7.3 Implementation

```cpp
// RP2350_timer.cpp
#include "RP2350_timer.h"
#include "../remora-core/thread/timerInterrupt.h"
#include "../remora-core/thread/pruThread.h"
#include "hardware/irq.h"

RP2350_timer::RP2350_timer(uint _alarmNum, bool _useTimer1,
                            uint32_t _frequency, pruThread* _ownerPtr,
                            int _irqPriority)
    : alarmNum(_alarmNum), useTimer1(_useTimer1), irqPriority(_irqPriority)
{
    frequency      = _frequency;
    timerOwnerPtr  = _ownerPtr;
    // TimerInterrupt is not used here directly — timerCallback calls timerTick()
    // which calls timerOwnerPtr->update() — the chain is preserved without
    // needing to register in the Interrupt vector table for timer alarms.
    timerRunning   = false;
}

void RP2350_timer::configTimer() {
    // Set IRQ priority for the alarm IRQ line
    // TIMER0 alarms: IRQ 0-3; TIMER1 alarms: IRQ 8-11
    uint irqNum = (useTimer1 ? 8 : 0) + alarmNum;
    irq_set_priority(irqNum, (uint8_t)irqPriority);
    printf("Timer configured: alarm %u on TIMER%u, frequency %lu Hz, "
           "IRQ %u priority %d\n",
           alarmNum, useTimer1 ? 1 : 0, (unsigned long)frequency, irqNum, irqPriority);
}

void RP2350_timer::startTimer() {
    // Negative period_us means "fire from end of previous callback" — prevents drift
    int32_t period_us = -(int32_t)(1000000 / frequency);
    alarm_pool_t* pool = useTimer1
        ? alarm_pool_create_with_unused_hardware_alarm(4)   // TIMER1
        : alarm_pool_get_default();                          // TIMER0

    bool ok = alarm_pool_add_repeating_timer_us(pool, period_us,
                                                timerCallback, this, &timerState);
    if (!ok) {
        printf("RP2350_timer: failed to start repeating timer (no free alarms?)\n");
        return;
    }
    timerRunning = true;
    printf("Timer started: %lu Hz\n", (unsigned long)frequency);
}

void RP2350_timer::stopTimer() {
    cancel_repeating_timer(&timerState);
    timerRunning = false;
    printf("Timer stopped\n");
}

void RP2350_timer::timerTick() {
    if (timerOwnerPtr) {
        timerOwnerPtr->update();
    }
}
```

### 7.4 Thread assignments

| Thread | Timer block | Alarm | Approximate IRQ | Frequency |
|---|---|---|---|---|
| Base | TIMER0 (default pool) | 0 | 0 | 40 kHz (25 µs) |
| Servo | TIMER0 (default pool) | 1 | 1 | 1 kHz (1 ms) |
| Serial | TIMER1 (custom pool) | 0 | 8 | 57.6 kHz (17.3 µs) |

TIMER1 is kept separate for the serial thread to prevent any interference with the motion-critical base and servo timers.

### 7.5 main.cpp timer construction

```cpp
auto baseTimer   = std::make_unique<RP2350_timer>(0, false, Config::pruBaseFreq,
                                                   nullptr, 0);    // priority 0 = highest
auto servoTimer  = std::make_unique<RP2350_timer>(1, false, Config::pruServoFreq,
                                                   nullptr, 64);
auto serialTimer = std::make_unique<RP2350_timer>(0, true,  Config::pruSerialFreq,
                                                   nullptr, 128);
```

### 7.6 Timer accuracy note

At 40 kHz the period is 25 µs exactly. The RP2350's 64-bit timer runs off the 1 MHz reference clock derived from `clk_ref`. The alarm fires within ±1 µs jitter, which is acceptable for step generation — LinuxCNC's own position controller is running at 1 kHz so sub-25 µs jitter in step timing is invisible to the servo loop.

---

## 8. Phase 4 — Interrupt Dispatch System

**Files:** `Src/hardware.h` (already covered), `Src/irqHandlers.h` (new)

### 8.1 How the dispatch system works

`remora-core`'s `Interrupt` class maintains a static array `ISRVectorTable[PERIPH_COUNT_IRQn]`. Hardware ISRs call `Interrupt::InvokeHandler(irqNum)`, which calls `ISRVectorTable[irqNum]->ISR_Handler()`. `ModuleInterrupt<T>` and `TimerInterrupt` register themselves into this table via `Interrupt::Register(irqNum, this)`.

On the RP2350, the ISR entry points change from STM32-style named functions to pico-sdk registered handlers, but the dispatch mechanism within remora-core is completely unchanged.

### 8.2 GPIO interrupts

All GPIO interrupts on RP2350 share a single IRQ line: `IO_IRQ_BANK0` (IRQ 13). The pico-sdk demultiplexes this to per-pin callbacks when `gpio_set_irq_enabled_with_callback()` is used. The shared callback receives the triggering GPIO number and event mask.

The strategy: use the GPIO pin number as the IRQ index passed to `Interrupt::InvokeHandler()`. This keeps everything within the 52-slot table (GP0–GP47 ≤ 47 < 52):

```cpp
// irqHandlers.h — RP2350 version
#pragma once
#include "remora-core/interrupt/interrupt.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/dma.h"

// Shared GPIO interrupt callback — routes to the registered handler for that pin
static void gpio_irq_dispatcher(uint gpio, uint32_t events) {
    // Use the GPIO number directly as the IRQ index
    // (all GP numbers fit within PERIPH_COUNT_IRQn = 52)
    Interrupt::InvokeHandler(gpio);
}

// DMA interrupt handler — channel 0 handles SPI slave RX (double-buffer)
static void __isr dma_irq0_handler() {
    Interrupt::InvokeHandler(DMA_IRQ_0);   // DMA_IRQ_0 = 11
    dma_hw->ints0 = dma_hw->ints0;        // clear all channel interrupts on IRQ0
}

// DMA interrupt handler — channel 1 handles SPI slave TX
static void __isr dma_irq1_handler() {
    Interrupt::InvokeHandler(DMA_IRQ_1);   // DMA_IRQ_1 = 12
    dma_hw->ints1 = dma_hw->ints1;
}
```

### 8.3 Registering GPIO interrupts

The `gpio_irq_dispatcher` must be registered once, globally, before any module creates a `ModuleInterrupt` for a GPIO pin:

```cpp
// In main.cpp, early in the init sequence:
gpio_set_irq_callback(gpio_irq_dispatcher);
irq_set_enabled(IO_IRQ_BANK0, true);
irq_set_priority(IO_IRQ_BANK0, 128);  // lower priority than motion threads
```

Individual pins then enable their specific edge trigger:

```cpp
// In RP2350_SPIComms::init() for the CS rising-edge interrupt:
gpio_set_irq_enabled(CS_GPIO, GPIO_IRQ_EDGE_RISE, true);

// In Hardware_QEI constructor for the index pin:
gpio_set_irq_enabled(indexPin, GPIO_IRQ_EDGE_RISE, true);
```

The `ModuleInterrupt<RP2350_SPIComms>` is registered with the CS GPIO pin number as its IRQ index. When `gpio_irq_dispatcher` fires with `gpio == CS_GPIO`, `Interrupt::InvokeHandler(CS_GPIO)` calls the registered `ModuleInterrupt::ISR_Handler()`, which calls `RP2350_SPIComms::handleNssInterrupt()` — exactly the same call chain as on STM32, just with a different IRQ number.

### 8.4 DMA interrupt registration

```cpp
// In RP2350_SPIComms::init():
irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
irq_set_priority(DMA_IRQ_0, 64);
irq_set_enabled(DMA_IRQ_0, true);
```

The `ModuleInterrupt<RP2350_SPIComms>` for DMA is registered with `DMA_IRQ_0` (= 11) as its IRQ index.

---

## 9. Phase 5 — SPI Slave Communications

**Files:** `Src/remora-hal/RP2350_SPIComms.h`, `Src/remora-hal/RP2350_SPIComms.cpp`

This is the most architecturally complex component. The STM32 version used hardware DMA double-buffering (`HAL_DMAEx_MultiBufferStart_IT`). The RP2350 achieves the same effect with two chained DMA channels.

### 9.1 Class overview

```cpp
// RP2350_SPIComms.h
#pragma once
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "../remora-core/comms/commsInterface.h"
#include "../remora-core/modules/moduleInterrupt.h"
#include "pin/pin.h"

class RP2350_SPIComms : public CommsInterface {
private:
    spi_inst_t*   spiInst;          // spi0 or spi1

    int  gpioMosi, gpioMiso, gpioClk, gpioCs;

    // DMA channels
    int  dmaRxChanA;    // RX channel A → rxDMABuffer[0]
    int  dmaRxChanB;    // RX channel B → rxDMABuffer[1]
    int  dmaTxChan;     // TX channel (circular/endless)
    int  dmaMemCpyChan; // mem-to-mem copy channel

    volatile DMA_RxBuffer_t* ptrRxDMABuffer;

    uint8_t  RXbufferIdx;
    bool     copyRXbuffer;
    bool     newWriteData;

    ModuleInterrupt<RP2350_SPIComms>* nssInterrupt;
    ModuleInterrupt<RP2350_SPIComms>* dmaRxInterrupt;

    void handleNssInterrupt();
    void handleDmaRxInterrupt();
    void initDMA();

public:
    static RP2350_SPIComms* instance;
    static volatile uint8_t RxDMAmemoryIdx;

    RP2350_SPIComms(volatile rxData_t*, volatile txData_t*,
                    int mosiGpio, int misoGpio, int clkGpio, int csGpio);

    void init()   override;
    void start()  override;
    void tasks()  override;

    void checkHeader();
};
```

### 9.2 SPI slave configuration

```cpp
void RP2350_SPIComms::init() {
    printf("SPIComms Init\n");

    // Determine which SPI instance from the GPIO numbers
    // SPI0: MISO=GP0/4/16/20, MOSI=GP3/7/19/23, SCK=GP2/6/18/22, CS=GP1/5/17/21
    // SPI1: MISO=GP8/12/24/28, MOSI=GP11/15/27, SCK=GP10/14/26, CS=GP9/13/25/29
    spiInst = (gpioMosi <= 7 || (gpioMosi >= 16 && gpioMosi <= 23)) ? spi0 : spi1;

    spi_init(spiInst, 0);           // frequency is set by master in slave mode
    spi_set_slave(spiInst, true);
    spi_set_format(spiInst, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);

    gpio_set_function(gpioMosi, GPIO_FUNC_SPI);
    gpio_set_function(gpioMiso, GPIO_FUNC_SPI);
    gpio_set_function(gpioClk,  GPIO_FUNC_SPI);
    gpio_set_function(gpioCs,   GPIO_FUNC_SPI);  // hardware NSS in slave mode

    printf("SPI slave configured on SPI%u\n", spiInst == spi0 ? 0 : 1);

    initDMA();

    // Register DMA completion interrupt
    dmaRxInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        DMA_IRQ_0,
        this,
        &RP2350_SPIComms::handleDmaRxInterrupt
    );
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq0_handler);
    irq_set_priority(DMA_IRQ_0, Config::spiDmaRxIrqPriority * 32); // map 0-7 to 0-224
    irq_set_enabled(DMA_IRQ_0, true);

    // Register CS rising-edge interrupt using GPIO pin number as IRQ index
    nssInterrupt = new ModuleInterrupt<RP2350_SPIComms>(
        gpioCs,   // GPIO number used as IRQ slot index
        this,
        &RP2350_SPIComms::handleNssInterrupt
    );
    gpio_set_irq_enabled(gpioCs, GPIO_IRQ_EDGE_RISE, true);
}
```

### 9.3 Double-buffer DMA (chained channels)

The STM32 used `HAL_DMAEx_MultiBufferStart_IT()` for hardware double-buffering. The RP2350 equivalent is two DMA channels chained to each other, alternating between the two RX buffers:

```cpp
void RP2350_SPIComms::initDMA() {
    dmaRxChanA   = dma_claim_unused_channel(true);
    dmaRxChanB   = dma_claim_unused_channel(true);
    dmaTxChan    = dma_claim_unused_channel(true);
    dmaMemCpyChan = dma_claim_unused_channel(true);

    // ---- RX Channel A → rxDMABuffer.buffer[0] ----
    dma_channel_config cfgA = dma_channel_get_default_config(dmaRxChanA);
    channel_config_set_transfer_data_size(&cfgA, DMA_SIZE_8);
    channel_config_set_dreq(&cfgA, spi_get_dreq(spiInst, false)); // SPI RX DREQ
    channel_config_set_read_increment(&cfgA, false);  // SPI DR — fixed address
    channel_config_set_write_increment(&cfgA, true);  // into rxDMABuffer[0]
    channel_config_set_chain_to(&cfgA, dmaRxChanB);   // chain to B when complete
    channel_config_set_irq_quiet(&cfgA, false);        // fire IRQ on completion
    dma_channel_configure(dmaRxChanA, &cfgA,
        (void*)ptrRxDMABuffer->buffer[0].rxBuffer,  // write destination
        &spi_get_hw(spiInst)->dr,                   // read source (SPI data register)
        Config::dataBuffSize,                        // 64 bytes per transfer
        false);                                      // don't start yet

    // ---- RX Channel B → rxDMABuffer.buffer[1], chains back to A ----
    dma_channel_config cfgB = dma_channel_get_default_config(dmaRxChanB);
    channel_config_set_transfer_data_size(&cfgB, DMA_SIZE_8);
    channel_config_set_dreq(&cfgB, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgB, false);
    channel_config_set_write_increment(&cfgB, true);
    channel_config_set_chain_to(&cfgB, dmaRxChanA);   // chain back to A
    channel_config_set_irq_quiet(&cfgB, false);
    dma_channel_configure(dmaRxChanB, &cfgB,
        (void*)ptrRxDMABuffer->buffer[1].rxBuffer,
        &spi_get_hw(spiInst)->dr,
        Config::dataBuffSize,
        false);

    // ---- TX Channel — continuously clocks out txData (endless mode) ----
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, spi_get_dreq(spiInst, true)); // SPI TX DREQ
    channel_config_set_read_increment(&cfgTx, true);
    channel_config_set_write_increment(&cfgTx, false); // SPI DR — fixed address
    channel_config_set_chain_to(&cfgTx, dmaTxChan);    // self-chain = circular
    dma_channel_configure(dmaTxChan, &cfgTx,
        &spi_get_hw(spiInst)->dr,            // write destination (SPI DR)
        (void*)ptrTxData->txBuffer,          // read source
        Config::dataBuffSize,
        false);

    // Enable DMA interrupt for RX channels on DMA_IRQ_0
    dma_channel_set_irq0_enabled(dmaRxChanA, true);
    dma_channel_set_irq0_enabled(dmaRxChanB, true);
}
```

### 9.4 DMA start

```cpp
void RP2350_SPIComms::start() {
    // Clear both RX buffers
    memset((void*)ptrRxDMABuffer->buffer[0].rxBuffer, 0, Config::dataBuffSize);
    memset((void*)ptrRxDMABuffer->buffer[1].rxBuffer, 0, Config::dataBuffSize);

    // Start the ping-pong: channel A fires first, chains to B, chains to A, etc.
    dma_channel_start(dmaRxChanA);
    dma_channel_start(dmaTxChan);
}
```

### 9.5 DMA completion interrupt handler

The STM32 version used a half-complete callback to check headers early. On RP2350, the simpler approach is to check headers on full completion. At 1 kHz servo rate and a SPI transaction rate well below that, full-complete is adequate:

```cpp
void RP2350_SPIComms::handleDmaRxInterrupt() {
    // Determine which channel just completed
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
    switch (ptrRxDMABuffer->buffer[RxDMAmemoryIdx].header) {
        case Config::pruRead:
            dataCallback(true);
            break;
        case Config::pruWrite:
            dataCallback(true);
            newWriteData = true;
            RXbufferIdx = RxDMAmemoryIdx;
            break;
        default:
            dataCallback(false);
            break;
    }
}
```

### 9.6 NSS (CS) interrupt handler

```cpp
void RP2350_SPIComms::handleNssInterrupt() {
    // SPI packet complete — if a WRITE was received, schedule the buffer copy
    if (newWriteData) {
        copyRXbuffer = true;
        newWriteData = false;
    }
}
```

### 9.7 tasks() — memory-to-memory DMA copy

```cpp
void RP2350_SPIComms::tasks() {
    if (!copyRXbuffer) return;

    uint8_t* src  = (uint8_t*)ptrRxDMABuffer->buffer[RXbufferIdx].rxBuffer;
    uint8_t* dest = (uint8_t*)ptrRxData->rxBuffer;

    // Disable interrupts during the copy to prevent partial reads
    uint32_t irq_state = save_and_disable_interrupts();

    // Use a DMA mem-to-mem transfer for the copy
    dma_channel_config cfg = dma_channel_get_default_config(dmaMemCpyChan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, true);
    // No DREQ — mem-to-mem runs at system clock speed
    dma_channel_configure(dmaMemCpyChan, &cfg,
        dest, src,
        Config::dataBuffSize / 4,   // 64 bytes / 4 = 16 × 32-bit transfers
        true);                       // start immediately
    dma_channel_wait_for_finish_blocking(dmaMemCpyChan);

    restore_interrupts(irq_state);
    copyRXbuffer = false;
}
```

---

## 10. Phase 6 — Ethernet Communications (W5500)

**Files:** `Src/remora-hal/RP2350_EthComms.h`, `Src/remora-hal/RP2350_EthComms.cpp`

### 10.1 Architecture is almost identical to STM32

The STM32 `STM32F4_EthComms` was SPI master to the W5500. The RP2350 version is also SPI master. The `W5500_Networking.cpp` driver in remora-core **does not change at all** — it already delegates every SPI operation through the abstract `CommsInterface*` methods (`read_byte`, `write_byte`, `DMA_write`, `DMA_read`). The RP2350 ETH comms class simply implements those methods using pico-sdk calls.

### 10.2 SPI master configuration

```cpp
void RP2350_EthComms::init() {
    printf("EthComms Init\n");

    // Determine SPI instance from MOSI GPIO (same logic as SPIComms)
    spiInst = (gpioMosi <= 7 || ...) ? spi0 : spi1;

    spi_init(spiInst, 42 * 1000 * 1000);  // 42 MHz — W5500 supports up to 80 MHz
    spi_set_format(spiInst, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    // Master mode is the default

    gpio_set_function(gpioMosi, GPIO_FUNC_SPI);
    gpio_set_function(gpioMiso, GPIO_FUNC_SPI);
    gpio_set_function(gpioClk,  GPIO_FUNC_SPI);

    // CS and RST are plain GPIO outputs (not hardware NSS)
    gpio_init(gpioCs);
    gpio_set_dir(gpioCs, GPIO_OUT);
    gpio_put(gpioCs, 1);  // deassert CS

    gpio_init(gpioRst);
    gpio_set_dir(gpioRst, GPIO_OUT);
    gpio_put(gpioRst, 1);

    // Allocate DMA channels for bulk transfers
    dmaTxChan = dma_claim_unused_channel(true);
    dmaRxChan = dma_claim_unused_channel(true);

    printf("EthComms SPI initialised\n");
}
```

### 10.3 read_byte() and write_byte()

These are called by the WIZnet driver library for register access. Replace STM32 direct register access with pico-sdk:

```cpp
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
```

### 10.4 DMA_write() and DMA_read()

These are called for bulk W5500 buffer transfers:

```cpp
void RP2350_EthComms::DMA_write(uint8_t* data, uint16_t len) {
    dma_channel_config cfg = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfg, DMA_SIZE_8);
    channel_config_set_dreq(&cfg, spi_get_dreq(spiInst, true));
    channel_config_set_read_increment(&cfg, true);
    channel_config_set_write_increment(&cfg, false);
    dma_channel_configure(dmaTxChan, &cfg,
        &spi_get_hw(spiInst)->dr, data, len, true);
    dma_channel_wait_for_finish_blocking(dmaTxChan);
}

void RP2350_EthComms::DMA_read(uint8_t* data, uint16_t len) {
    // For SPI full-duplex, a simultaneous TX of dummy bytes is needed
    static uint8_t dummy = 0xFF;
    dma_channel_config cfgTx = dma_channel_get_default_config(dmaTxChan);
    channel_config_set_transfer_data_size(&cfgTx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgTx, spi_get_dreq(spiInst, true));
    channel_config_set_read_increment(&cfgTx, false); // same dummy byte repeatedly
    channel_config_set_write_increment(&cfgTx, false);
    dma_channel_configure(dmaTxChan, &cfgTx,
        &spi_get_hw(spiInst)->dr, &dummy, len, false);

    dma_channel_config cfgRx = dma_channel_get_default_config(dmaRxChan);
    channel_config_set_transfer_data_size(&cfgRx, DMA_SIZE_8);
    channel_config_set_dreq(&cfgRx, spi_get_dreq(spiInst, false));
    channel_config_set_read_increment(&cfgRx, false);
    channel_config_set_write_increment(&cfgRx, true);
    dma_channel_configure(dmaRxChan, &cfgRx,
        data, &spi_get_hw(spiInst)->dr, len, false);

    // Start both channels simultaneously
    dma_start_channel_mask((1u << dmaTxChan) | (1u << dmaRxChan));
    dma_channel_wait_for_finish_blocking(dmaRxChan);
}
```

### 10.5 flag_new_data() and tasks()

These are identical in structure to the STM32 version — `flag_new_data()` sets a boolean, `tasks()` checks it:

```cpp
void RP2350_EthComms::flag_new_data() {
    newDataFlagged = true;
}

void RP2350_EthComms::tasks() {
    network::EthernetTasks();  // polls W5500 for incoming Ethernet frames

    if (newDataFlagged) {
        newDataFlagged = false;
        switch (ptrRxData->header) {
            case Config::pruRead:
            case Config::pruWrite:
                dataCallback(true);
                break;
            default:
                dataCallback(false);
                break;
        }
    }
}
```

No DMA interrupt registration is needed for the ETH path — all DMA operations are blocking. The SPI IRQ handlers registered for the SPI slave path are on a different channel and don't interfere.

---

## 11. Phase 7 — Hardware PWM

**Files:** `Src/remora-hal/hardware_pwm/hardware_pwm.h`, `hardware_pwm.cpp`

### 11.1 RP2350 PWM architecture

The RP2350 PWM is fundamentally simpler than the STM32's timer-channel system: every GPIO can produce PWM output, and the slice number and channel are derived algorithmically:

```
slice_num = gpio_num / 2    (0-11)
channel   = gpio_num % 2    (0 = CHAN_A, 1 = CHAN_B)
```

Two GPIOs sharing the same slice share the same counter and period, exactly analogous to two STM32 timer channels sharing ARR.

### 11.2 Shared-slice synchronization

The STM32 version used an intrusive linked list to synchronize period changes across channels sharing the same TIM. On RP2350, use a simple static array indexed by slice number:

```cpp
// hardware_pwm.h
class HardwarePWM {
private:
    uint     gpioNum;
    uint     sliceNum;
    uint     channel;       // PWM_CHAN_A or PWM_CHAN_B
    uint32_t timerClkHz;    // system clock frequency

public:
    // One entry per slice — tracks all HardwarePWM instances sharing that slice
    static HardwarePWM* sliceInstances[12];

    uint32_t periodUs;
    float    periodPercent;

    HardwarePWM(uint32_t initialPeriodUs, float initialDutyPercent,
                const std::string& pin);
    ~HardwarePWM();

    void changePeriod(uint32_t newPeriodUs);
    void changePulsewidth(float newDutyPercent);

    // Keep the old snake_case names for compatibility with the PWM module in remora-core
    void change_period(uint32_t p)    { changePeriod(p); }
    void change_pulsewidth(float d)   { changePulsewidth(d); }
};
```

### 11.3 Initialization

```cpp
HardwarePWM::HardwarePWM(uint32_t initialPeriodUs, float initialDutyPercent,
                          const std::string& pin)
{
    gpioNum   = parseGpioNum(pin);  // extract int from "GP1" etc.
    sliceNum  = pwm_gpio_to_slice_num(gpioNum);
    channel   = pwm_gpio_to_channel(gpioNum);
    timerClkHz = clock_get_hz(clk_sys);  // 150,000,000 at default clock

    // Register in the slice instance table
    sliceInstances[sliceNum] = this;

    // Set the GPIO to PWM function
    gpio_set_function(gpioNum, GPIO_FUNC_PWM);

    // Prescaler: divide clock to get 1 MHz (1 µs resolution)
    // At 150 MHz: divider = 150.0, giving 1 MHz tick rate
    pwm_config cfg = pwm_get_default_config();
    pwm_config_set_clkdiv(&cfg, (float)timerClkHz / 1000000.0f);
    pwm_config_set_wrap(&cfg, initialPeriodUs - 1);
    pwm_init(sliceNum, &cfg, true);  // true = start immediately

    changePulsewidth(initialDutyPercent);
}
```

### 11.4 change_period()

```cpp
void HardwarePWM::changePeriod(uint32_t newPeriodUs) {
    periodUs = newPeriodUs;

    // Update all instances sharing this slice so their duty cycles stay correct
    for (int i = 0; i < 2; i++) {
        // Check both channels on this slice
        uint gpio_a = sliceNum * 2;
        uint gpio_b = sliceNum * 2 + 1;
        // Find the HardwarePWM instance for each channel if it exists
        // (sliceInstances only tracks one entry per slice — for dual-channel
        //  support, expand to sliceInstances[12][2])
    }

    // Set the new wrap value
    pwm_set_wrap(sliceNum, newPeriodUs - 1);

    // Trigger an immediate counter reload
    // (pwm_set_wrap takes effect at the next wrap, so force a reset)
    pwm_set_counter(sliceNum, 0);

    // Recalculate duty for this channel
    changePulsewidth(periodPercent);
}
```

### 11.5 change_pulsewidth()

```cpp
void HardwarePWM::changePulsewidth(float newDutyPercent) {
    periodPercent = newDutyPercent;

    if (periodUs == 0) return;

    uint32_t level = (uint32_t)((newDutyPercent / 100.0f) * periodUs);
    // Clamp to valid range
    if (level > periodUs) level = periodUs;
    pwm_set_chan_level(sliceNum, channel, (uint16_t)level);
}
```

### 11.6 No timer conflict warning needed

The STM32 version warned if TIM2/TIM3 (used by Remora threads) were selected. On RP2350, the hardware alarm timers used for thread scheduling are completely separate from the PWM slice hardware. All 12 slices (24 channels) are available for user PWM modules with no restrictions.

---

## 12. Phase 8 — Hardware Quadrature Encoder (QEI)

**Files:** `Src/remora-hal/hardware_qei/hardware_qei.h`, `hardware_qei.cpp`

### 12.1 PIO-based quadrature encoder (recommended approach)

The STM32 used TIM8 in encoder mode — a dedicated hardware quadrature counter. The RP2350 has no equivalent dedicated peripheral, but its PIO system can implement one using a well-known program from the Raspberry Pi pico-examples repository (`quadrature_encoder.pio`).

The PIO program uses two input pins (A and B), detects all four transitions per electrical cycle, and accumulates a signed count into a 32-bit value accessible via the PIO FIFO. The CPU reads the current count at any time by polling the FIFO or using a DMA channel.

### 12.2 Configurable pins — fixing the STM32 limitation

The STM32 `Hardware_QEI` had pins hardcoded as `"PC_6"`, `"PC_7"`, and `"PA_8"`. The RP2350 port fixes this by accepting pins as constructor arguments, which the `QEI` module in remora-core then passes from its JSON configuration.

**Required remora-core change** (one-time, affects all platforms): Update `QEI::create()` in `modules/qei/qei.cpp` to read `"ChA Pin"`, `"ChB Pin"`, and optionally `"Index Pin"` from the JSON config and pass them to the `Hardware_QEI` constructor. These keys already exist in `SoftEncoder::create()`, so the JSON format is consistent.

### 12.3 Class definition

```cpp
// hardware_qei.h
#pragma once
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "../remora-core/modules/moduleInterrupt.h"

class Hardware_QEI {
private:
    PIO      pioInst;       // pio0 or pio1
    uint     smNum;         // state machine number (0-3)
    uint     pioOffset;     // loaded program offset

    int      chAPin;
    int      chBPin;
    int      indexPin;
    bool     hasIndex;
    int      modifier;

    ModuleInterrupt<Hardware_QEI>* indexInterrupt;

    void handleIndexInterrupt();

public:
    bool    indexDetected;
    int32_t indexCount;

    Hardware_QEI(bool hasIndex, int modifier, int chAPin, int chBPin,
                 int indexPin = -1);
    void     init();
    uint32_t get();   // returns current encoder count
};
```

### 12.4 Implementation

```cpp
void Hardware_QEI::init() {
    printf("Initialising hardware QEI on GP%d/GP%d\n", chAPin, chBPin);

    // Load the quadrature encoder PIO program
    // (quadrature_encoder.pio from pico-examples, compiled to a header)
    pioInst  = pio0;
    pioOffset = pio_add_program(pioInst, &quadrature_encoder_program);
    smNum    = pio_claim_unused_sm(pioInst, true);

    // Configure and start the state machine
    quadrature_encoder_program_init(pioInst, smNum, pioOffset,
                                    chAPin,   // base pin (A pin; B pin = chAPin + 1
                                               // if consecutive, or configure manually)
                                    0);        // max step rate (0 = unlimited)

    if (hasIndex && indexPin >= 0) {
        indexInterrupt = new ModuleInterrupt<Hardware_QEI>(
            indexPin,  // GPIO number used as IRQ slot
            this,
            &Hardware_QEI::handleIndexInterrupt
        );
        gpio_init(indexPin);
        gpio_set_dir(indexPin, GPIO_IN);
        if (modifier == GPIO_PULLUP)        gpio_pull_up(indexPin);
        else if (modifier == GPIO_PULLDOWN) gpio_pull_down(indexPin);
        else                                gpio_disable_pulls(indexPin);
        gpio_set_irq_enabled(indexPin, GPIO_IRQ_EDGE_RISE, true);
    }
}

uint32_t Hardware_QEI::get() {
    // Read the current count from the PIO FIFO
    // The PIO program pushes the count on demand or continuously
    return quadrature_encoder_get_count(pioInst, smNum);
}

void Hardware_QEI::handleIndexInterrupt() {
    indexDetected = true;
    indexCount    = get();
}
```

**Note:** `quadrature_encoder_program_init()` and `quadrature_encoder_get_count()` are helper functions defined alongside the `.pio` program in the pico-examples code. They are included verbatim or adapted as needed. The PIO program expects the A pin and B pin to be consecutive GPIO numbers for simplicity; if non-consecutive pins are needed, a modified PIO program is required.

### 12.5 Alternative: SoftEncoder

For applications where the encoder speed is below ~8 kHz and the base thread (40 kHz) has sufficient CPU budget, the existing `SoftEncoder` module requires zero new hardware code. It is already part of remora-core and uses `Pin::get()` for polling. This is the simplest path for initial bring-up.

---

## 13. Phase 9 — Analog Input (ADC)

**Files:** `Src/remora-hal/analogIn/analogIn.h` (interface unchanged), `analogIn.cpp` (rewrite)

### 13.1 RP2350 ADC architecture

The RP2350 has a single 12-bit ADC with up to 8 input channels. On a Pico 2 board, the accessible ADC-capable GPIOs are GP26 (channel 0) through GP29 (channel 3). The RP2350B variant adds channels 4–7 on GP40–GP47.

```cpp
AnalogIn::AnalogIn(const std::string& portAndPin) : portAndPin(portAndPin) {
    gpioNum = parseGpioNum(portAndPin);  // extract int from "GP26" etc.

    // Map GPIO to ADC channel:
    // GP26 → channel 0, GP27 → channel 1, GP28 → channel 2, GP29 → channel 3
    // GP40–GP47 → channels 4–7 (RP2350B only)
    if (gpioNum >= 26 && gpioNum <= 29) {
        channel = gpioNum - 26;
    } else if (gpioNum >= 40 && gpioNum <= 47) {
        channel = gpioNum - 36;  // 40-36=4 through 47-36=11
    } else {
        printf("AnalogIn: GP%d is not an ADC-capable pin on RP2350\n", gpioNum);
        channel = -1;
        return;
    }

    // Initialize ADC (idempotent — safe to call multiple times)
    static bool adc_initialized = false;
    if (!adc_initialized) {
        adc_init();
        adc_initialized = true;
    }

    adc_gpio_init(gpioNum);  // configures the GPIO for analog input (no pull resistors)
}

uint32_t AnalogIn::read() {
    if (channel < 0) return 0;
    adc_select_input(channel);
    return adc_read();  // returns 12-bit value 0–4095
}
```

### 13.2 Fix the thermistor scaling bug

The thermistor formula in `remora-core/sensors/thermistor/thermistor.cpp` uses `65536.0F` as the ADC full-scale value, but the RP2350 (like the STM32) has a 12-bit ADC with a maximum value of 4095. This is a remora-core bug that must be fixed in this PR:

```cpp
// thermistor.cpp — change:
float r = this->r2 / ((65536.0F / adcValue) - 1.0F);
// To:
float r = this->r2 / ((4096.0F / adcValue) - 1.0F);
```

### 13.3 Future enhancement: DMA round-robin sampling

The RP2350 ADC supports DMA-based continuous sampling using `adc_fifo_setup()` with `DREQ_ADC`. For a production implementation, this replaces the blocking `adc_read()` call with a continuously-running background DMA that feeds a small circular buffer. `AnalogIn::read()` then returns the latest value without blocking. This is an enhancement for a future revision, not a requirement for the initial port.

---

## 14. Phase 10 — Flash Storage & Platform Configuration

**Files:** `Src/remora-hal/platform_configuration.h`, `Src/remora-hal/hal_utils.h`, `Src/remora-hal/hal_utils.cpp`

### 14.1 Flash memory architecture differences

The STM32 has internal flash divided into named sectors of varying sizes, accessed directly by the CPU at known addresses. Writes use `HAL_FLASH_Program()`. Erasing uses `HAL_FLASHEx_Erase()` by sector number.

The RP2350 uses external QSPI flash (4 MB on Pico 2), accessed via XIP (Execute In Place). Flash at runtime is memory-mapped starting at `0x10000000`. Writing requires:
1. Disabling the XIP cache (`flash_range_erase` and `flash_range_program` handle this)
2. Erasing in 4 KB pages
3. Programming in 256-byte pages
4. Functions performing these operations **must run from RAM** (not flash)

The pico-sdk provides `flash_range_erase(offset, count)` and `flash_range_program(offset, data, count)` which handle all of this internally. `offset` is the byte offset from the start of flash (not the XIP mapped address).

### 14.2 platform_configuration.h

Replace the STM32 linker-symbol approach with fixed compile-time constants. On a 4 MB flash device, reserve the top 32 KB for config (two 16 KB regions), leaving the first 4,062 KB for the program — far more than needed:

```cpp
// Src/remora-hal/platform_configuration.h
#pragma once
#include <cstdint>

namespace Platform_Config {
    // RP2350 external flash: mapped at 0x10000000, total 4 MB
    constexpr uint32_t FLASH_XIP_BASE    = 0x10000000u;
    constexpr uint32_t FLASH_TOTAL_BYTES = 4u * 1024u * 1024u;  // 4 MB

    // Reserve last 32 KB: 16 KB upload staging + 16 KB persistent storage
    constexpr uint32_t JSON_UPLOAD_OFFSET  = FLASH_TOTAL_BYTES - 32768u;  // offset from flash start
    constexpr uint32_t JSON_STORAGE_OFFSET = FLASH_TOTAL_BYTES - 16384u;  // offset from flash start

    // XIP-mapped addresses (for reading via pointer dereference)
    constexpr uintptr_t JSON_upload_start_address  = FLASH_XIP_BASE + JSON_UPLOAD_OFFSET;
    constexpr uintptr_t JSON_upload_end_address    = JSON_upload_start_address  + 16384u;
    constexpr uintptr_t JSON_storage_start_address = FLASH_XIP_BASE + JSON_STORAGE_OFFSET;
    constexpr uintptr_t JSON_storage_end_address   = JSON_storage_start_address + 16384u;

    // Sector numbers (used by remora-core API for mass_erase — on RP2350 these
    // are 4 KB page indices, not STM32 sector numbers)
    constexpr uint32_t JSON_Config_Upload_Sector  = JSON_UPLOAD_OFFSET  / 4096u;
    constexpr uint32_t JSON_Config_Storage_Sector = JSON_STORAGE_OFFSET / 4096u;
}
```

### 14.3 hal_utils.h — flash macro replacements

The STM32 `hal_utils.h` defines macros that remora-core calls for flash operations. Replace with RP2350 equivalents:

```cpp
// Src/remora-hal/hal_utils.h
#pragma once
#include "pico/stdlib.h"
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"
#include "platform_configuration.h"
#include "pin/pin.h"

// System reset — pico-sdk watchdog reboot
#define pru_reboot()    watchdog_reboot(0, 0, 0)

// Flash operations — see hal_utils.cpp for implementations
// (lock/unlock are no-ops; pico-sdk handles XIP suspension internally)
inline void lock_flash()   {}
inline void unlock_flash() {}

// Flash erase/write wrappers (defined in hal_utils.cpp, must run from RAM)
void mass_erase_config_storage();
void mass_erase_upload_storage();
void mass_erase_flash_sector(uint32_t page_index);  // erases one 4 KB page
uint8_t write_to_flash_byte(uint32_t xip_addr, uint8_t data);
uint8_t write_to_flash_halfword(uint32_t xip_addr, uint16_t data);

// Utility
void delay_ms(uint32_t ms);
```

### 14.4 hal_utils.cpp — flash implementation

Flash writes on RP2350 must be done in 256-byte pages. The `jsonConfigHandler.cpp` calls `write_to_flash_byte()` and `write_to_flash_halfword()` in a loop. To avoid an erase+write cycle on every single byte, buffer the writes into a 256-byte staging buffer and flush on page boundaries:

```cpp
// hal_utils.cpp

// Must run from RAM — annotate with __not_in_flash_func
static __not_in_flash_func(void) rp2350_flash_write_page(
    uint32_t flash_offset, const uint8_t* data)
{
    // flash_offset must be aligned to 256 bytes
    // Interrupts must be disabled during flash programming
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_program(flash_offset, data, FLASH_PAGE_SIZE);  // 256 bytes
    restore_interrupts(irq_state);
}

static __not_in_flash_func(void) rp2350_flash_erase_sector(uint32_t flash_offset)
{
    // flash_offset must be aligned to 4096 bytes
    uint32_t irq_state = save_and_disable_interrupts();
    flash_range_erase(flash_offset, FLASH_SECTOR_SIZE);  // 4096 bytes
    restore_interrupts(irq_state);
}

// --- Buffered write system ---
static uint8_t  write_page_buf[FLASH_PAGE_SIZE];   // 256-byte page buffer
static uint32_t write_buf_offset = UINT32_MAX;     // current page offset (UINT32_MAX = no page open)
static uint32_t write_buf_pos    = 0;              // position within page

static void flush_write_buffer() {
    if (write_buf_offset != UINT32_MAX) {
        rp2350_flash_write_page(write_buf_offset, write_page_buf);
        write_buf_offset = UINT32_MAX;
        write_buf_pos    = 0;
    }
}

uint8_t write_to_flash_byte(uint32_t xip_addr, uint8_t data) {
    uint32_t flash_offset = xip_addr - Platform_Config::FLASH_XIP_BASE;
    uint32_t page_offset  = flash_offset & ~(FLASH_PAGE_SIZE - 1);  // align to 256

    if (page_offset != write_buf_offset) {
        flush_write_buffer();   // flush previous page if different
        memset(write_page_buf, 0xFF, FLASH_PAGE_SIZE);  // init to erased state
        write_buf_offset = page_offset;
    }

    write_page_buf[flash_offset & (FLASH_PAGE_SIZE - 1)] = data;
    write_buf_pos++;

    if (write_buf_pos >= FLASH_PAGE_SIZE) {
        flush_write_buffer();   // auto-flush when page is full
    }
    return 0;  // 0 = HAL_OK equivalent
}

uint8_t write_to_flash_halfword(uint32_t xip_addr, uint16_t data) {
    uint8_t result = write_to_flash_byte(xip_addr,     (uint8_t)(data & 0xFF));
    result        |= write_to_flash_byte(xip_addr + 1, (uint8_t)(data >> 8));
    return result;
}

void mass_erase_flash_sector(uint32_t page_index) {
    uint32_t flash_offset = page_index * FLASH_SECTOR_SIZE;
    rp2350_flash_erase_sector(flash_offset);
}

void mass_erase_config_storage() {
    mass_erase_flash_sector(Platform_Config::JSON_Config_Storage_Sector);
}

void mass_erase_upload_storage() {
    mass_erase_flash_sector(Platform_Config::JSON_Config_Upload_Sector);
}

void delay_ms(uint32_t ms) {
    sleep_ms(ms);
}
```

**Important:** After all byte writes in `store_json_in_flash()` are complete, `flush_write_buffer()` must be called once to commit any partial final page. Add this call at the end of the `store_json_in_flash()` function in `jsonConfigHandler.cpp`.

### 14.5 TFTP flash write interlock

The TFTP `IAP_wrq_recv_callback()` in `W5500_Networking.cpp` writes data block-by-block to the JSON_UPLOAD flash area. On RP2350, these writes go through `write_to_flash_halfword()`, which is now buffered. The TFTP code path already calls `mass_erase_upload_storage()` before the first write and the page-buffered writes will batch correctly. No changes to `W5500_Networking.cpp` are needed.

---

## 15. Phase 11 — Supporting Components

### 15.1 board_led_status.cpp

The `flash_led_error()` function uses `HAL_Delay()`. Replace with `sleep_ms()`. No other changes:

```cpp
// Replace all HAL_Delay(ms) calls with:
sleep_ms(ms);
```

The LED pin is changed in `main.cpp` from `"PD_10"` to `"GP25"` (the onboard LED on the Pico 2):

```cpp
init_board_status_led("GP25");
```

### 15.2 shared_handlers.cpp

The STM32 `shared_handlers.cpp` pre-allocated TIM and ADC handle structs to prevent double-initialization. On RP2350, the pico-sdk manages peripheral state internally and peripheral functions are idempotent. The shared handler system is not needed. Provide an empty stub:

```cpp
// shared_handlers.cpp — stub for RP2350
// Peripheral handle management is not needed on RP2350.
// pico-sdk hardware functions are idempotent.
```

Keep `shared_handlers.h` with minimal declarations so that any remora-core include chains that reference it still compile.

### 15.3 main.cpp

```cpp
// Src/main.cpp
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include <memory>
#include <cstdio>

#include "remora-core/remora.h"
#include "remora-hal/board_led_status.h"
#include "remora-hal/RP2350_timer.h"

#ifdef ETH_CTRL
    #include "remora-hal/RP2350_EthComms.h"
#else
    #include "remora-hal/RP2350_SPIComms.h"
#endif

int main() {
    // Enable USB serial printf (configured in CMakeLists.txt)
    stdio_init_all();

    // Register the shared GPIO interrupt dispatcher — must be done before
    // any ModuleInterrupt<> objects are created for GPIO pins
    gpio_set_irq_callback(gpio_irq_dispatcher);
    irq_set_enabled(IO_IRQ_BANK0, true);

    init_board_status_led("GP25");
    sleep_ms(2000);

    printf("Initialising Remora for RP2350\n");
    printf("CPU Clock: %lu Hz\n", (unsigned long)clock_get_hz(clk_sys));

    #ifdef SPI_CTRL
        // SD card init for SPI builds (SPI-based FatFs)
        // sd_init_driver();   // from no-OS-FatFS-SD-SPI-RPi-Pico
        // MX_FATFS_Init() equivalent — handled by FatFs library
    #endif

    // Construct the communications interface
    std::unique_ptr<CommsInterface> commsInterface;

    #ifdef ETH_CTRL
        commsInterface = std::make_unique<RP2350_EthComms>(
            &rxData, &txData,
            SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_CLK_GPIO,
            SPI_CS_GPIO, WIZ_RST_GPIO);
    #else
        commsInterface = std::make_unique<RP2350_SPIComms>(
            &rxData, &txData,
            SPI_MOSI_GPIO, SPI_MISO_GPIO, SPI_CLK_GPIO, SPI_CS_GPIO);
    #endif

    auto commsHandler = std::make_shared<CommsHandler>();
    commsHandler->setInterface(std::move(commsInterface));

    // Construct timers
    // Base thread:   TIMER0 alarm 0, 40 kHz, highest priority
    // Servo thread:  TIMER0 alarm 1, 1 kHz
    // Serial thread: TIMER1 alarm 0, 57.6 kHz (for TMC UART bit clock)
    auto baseTimer   = std::make_unique<RP2350_timer>(0, false, Config::pruBaseFreq,
                                                       nullptr, 0);
    auto servoTimer  = std::make_unique<RP2350_timer>(1, false, Config::pruServoFreq,
                                                       nullptr, 64);
    auto serialTimer = std::make_unique<RP2350_timer>(0, true,  Config::pruSerialFreq,
                                                       nullptr, 128);

    // Construct Remora engine and start
    Remora* remora = new Remora(
        commsHandler,
        std::move(baseTimer),
        std::move(servoTimer),
        std::move(serialTimer)   // serial thread now properly enabled
    );

    remora->run();  // never returns
}
```

### 15.4 SD card for SPI builds

The STM32 used its built-in SDIO peripheral. The RP2350 has no SDIO — SD cards must be accessed over SPI. The recommended library is **`no-OS-FatFS-SD-SPI-RPi-Pico`** by Carl Pottle, which provides a drop-in SPI-based FatFs for RP2040/RP2350. It exposes the same FatFs API (`f_mount`, `f_open`, `f_read`, `f_close`) used by `jsonConfigHandler.cpp`, so `readConfigFromSD()` requires no changes.

Add as a Git submodule:
```bash
git submodule add https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git lib/no-OS-FatFS-SD-SPI-RPi-Pico
```

Add to `CMakeLists.txt`:
```cmake
add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/src)
target_link_libraries(remora_rp2350_spi FatFs_SPI)
```

---

## 16. Phase 12 — Serial Thread & TMC Stepper Drivers

The STM32 HAL build passed `nullptr` for the serial timer, effectively disabling TMC driver support. The RP2350 port properly enables the serial thread using TIMER1.

### 16.1 Serial thread enabling

The serial timer is now constructed and passed to `Remora()` as shown in `main.cpp` above. The `pruSerialFreq = 57600 Hz` (19200 baud × 3 oversampling) gives the SoftwareSerial bit-clock.

The `TMC2209::configure()` and `TMC2208::configure()` methods call `instance->getSerialThread()->startThread()` — this will now succeed instead of dereferencing a null pointer.

### 16.2 SoftwareSerial pin mapping

The `SoftwareSerial` driver in remora-core uses the `Pin` class for TX/RX. The JSON config must specify `"RX pin"` using the new `"GPn"` format, e.g. `"GP4"`. No driver code changes are needed — the `Pin` class abstraction handles the rest.

### 16.3 TMC5160 SPI

The `TMC5160` uses `SoftwareSPI` for its dedicated SPI bus. The `SoftwareSPI` driver also uses the `Pin` class. JSON config keys `"pinCS"`, `"pinMOSI"`, `"pinMISO"`, `"pinSCK"` must use `"GPn"` format. No driver changes needed.

---

## 17. Phase 13 — Dual-Core Architecture (Enhancement)

This phase is **optional** for the initial port but highly recommended as a follow-up. The RP2350's dual Cortex-M33 cores allow distributing work that the single-core STM32 could only simulate through interrupt preemption.

### 17.1 Proposed core assignment

**Core 0 — Real-time motion control:**
- Base thread ISR (40 kHz step pulse generation)
- Servo thread ISR (1 kHz feedback and I/O)
- SPI slave DMA interrupt handlers
- GPIO interrupt handlers (CS pin, QEI index)

**Core 1 — Background and network:**
- `remora->run()` main loop (state machine)
- Ethernet `comms->tasks()` (W5500 polling loop)
- TFTP/JSON flash operations
- TMC driver `configure()` (one-shot at startup)

### 17.2 Implementation sketch

```cpp
// In main.cpp:

// Core 1 entry: runs the Remora state machine and network polling
void core1_entry() {
    remora->run();  // never returns
}

int main() {
    // ... initialization (same as Phase 11) ...

    // Launch Core 1 with the state machine AFTER Core 0 has initialized hardware
    multicore_launch_core1(core1_entry);

    // Core 0: spin forever servicing interrupts
    // The base and servo thread callbacks fire as hardware timer alarms
    // on Core 0 naturally (alarms fire on the core that armed them)
    while (true) {
        tight_loop_contents();
    }
}
```

The inter-core data sharing (rxData, txData) is already safe because the existing code uses `volatile` qualifiers and the copy is done under a brief IRQ-disabled critical section. No additional locking is needed for the initial dual-core port.

---

## 18. remora-core Bug Fixes to Apply During Port

These are all bugs in `remora-core` (platform-agnostic) that affect all targets. They should be submitted as separate pull requests to the remora-core repository but should be applied locally for the RP2350 port:

| Bug | File | Fix | Status |
|---|---|---|---|
| Thermistor 16-bit ADC divisor | `sensors/thermistor/thermistor.cpp` | Change `65536.0F` to `4096.0F` in the resistance formula | **FIXED** |
| O(n²) SD config string construction | `json/jsonConfigHandler.cpp` | Replace `jsonContent = jsonContent + rtext[i]` loop with `jsonContent.assign(rtext.data(), length)` | **FIXED** |
| VLA on stack in SD loader | `json/jsonConfigHandler.cpp` | Replace `char rtext[length]` with `std::vector<char> rtext(length)` | **FIXED** |
| Thread frequency setter broken | `remora.h/.cpp` | Added `baseTimer->setFrequency(baseFreq)` / `servoTimer->setFrequency(servoFreq)` in `Remora` constructor after JSON loads, before threads created. Startup frequency from JSON now correct. Runtime frequency change (while threads running) still requires `pruThread::setFrequency()` — not yet implemented. | **FIXED (startup)** |
| QEI pins hardcoded | `modules/qei/qei.cpp` | Read `"ChA Pin"`, `"ChB Pin"`, `"Index Pin"` from JSON config and pass to `Hardware_QEI` constructor | **FIXED** |

---

## 19. File Change Summary

### New files to create (the entire porting effort)

| New File | Replaces / Notes |
|---|---|
| `CMakeLists.txt` | Replaces `platformio.ini` |
| `pico_sdk_import.cmake` | New — standard pico-sdk bootstrap |
| `Src/hardware.h` | Update `PERIPH_COUNT_IRQn` from 149 to 52 |
| `Src/main.cpp` | Complete rewrite — pico-sdk init, RP2350 timer/comms construction |
| `Src/irqHandlers.h` | Complete rewrite — GPIO dispatcher, DMA ISR glue |
| `Src/remora-hal/platform_configuration.h` | Complete rewrite — fixed flash offsets, no linker symbols |
| `Src/remora-hal/hal_utils.h` | Partial rewrite — remove mbed pin-map helpers, add flash wrappers |
| `Src/remora-hal/hal_utils.cpp` | Complete rewrite — buffered flash write system, `sleep_ms` wrappers |
| `Src/remora-hal/shared_handlers.h/.cpp` | Replaced with empty stubs |
| `Src/remora-hal/board_led_status.cpp` | Minor change — `HAL_Delay` → `sleep_ms` |
| `Src/remora-hal/pin/pin.cpp` | Complete rewrite — parse `"GPn"` format, `gpio_init/put/get` |
| `Src/remora-hal/analogIn/analogIn.cpp` | Complete rewrite — pico-sdk `adc_init`/`adc_read` |
| `Src/remora-hal/hardware_pwm/hardware_pwm.h` | Updated — slice-based instead of TIM-based |
| `Src/remora-hal/hardware_pwm/hardware_pwm.cpp` | Complete rewrite — `pwm_gpio_to_slice_num`, `pwm_set_chan_level` |
| `Src/remora-hal/hardware_qei/hardware_qei.h` | Updated — configurable pins, PIO-based |
| `Src/remora-hal/hardware_qei/hardware_qei.cpp` | Complete rewrite — PIO quadrature encoder |
| `Src/remora-hal/RP2350_timer.h` | New — replaces `STM32F4_timer.h` |
| `Src/remora-hal/RP2350_timer.cpp` | New — replaces `STM32F4_timer.cpp` |
| `Src/remora-hal/RP2350_SPIComms.h` | New — replaces `STM32F4_SPIComms.h` |
| `Src/remora-hal/RP2350_SPIComms.cpp` | New — replaces `STM32F4_SPIComms.cpp` |
| `Src/remora-hal/RP2350_EthComms.h` | New — replaces `STM32F4_EthComms.h` |
| `Src/remora-hal/RP2350_EthComms.cpp` | New — replaces `STM32F4_EthComms.cpp` |

### Files that are deleted (STM32-specific, not needed on RP2350)

| Deleted File | Reason |
|---|---|
| `Src/remora-hal/peripheralPins.h/.c` | mbed-style pin-map tables — not needed; pico-sdk derives everything algorithmically |
| `Src/remora-hal/PinNamesTypes.h` | STM32-specific pin function bit-encoding macros |
| `Src/remora-hal/pinNames.h` | STM32 PinName enum — replaced by plain `int` GPIO numbers |
| `Src/remora-hal/STM32F4_timer.h/.cpp` | Replaced by `RP2350_timer.h/.cpp` |
| `Src/remora-hal/STM32F4_SPIComms.h/.cpp` | Replaced by `RP2350_SPIComms.h/.cpp` |
| `Src/remora-hal/STM32F4_EthComms.h/.cpp` | Replaced by `RP2350_EthComms.h/.cpp` |
| `Src/stm32f4xx_hal_msp.c` | STM32 HAL MSP callbacks — not applicable |
| `Src/stm32f4xx_it.c` | STM32 Cortex-M fault handlers — pico-sdk provides defaults |
| `Src/system_stm32f4xx.c` | STM32 system clock startup |
| `Src/syscalls.c` | Newlib retarget — `pico_stdio` handles this |
| `Src/sysmem.c` | Heap implementation — pico-sdk provides this |
| `Inc/` directory | STM32 HAL configuration headers (`stm32f4xx_hal_conf.h` etc.) |
| `FATFS/` directory | STM32Cube FATFS integration — replaced by `no-OS-FatFS-SD-SPI-RPi-Pico` |
| `Middlewares/` directory | STM32 FatFs middleware — replaced |
| `LinkerScripts/` | STM32 linker scripts — RP2350 uses pico-sdk default or minimal custom script |

### Minimal remora-core modifications for dual-core support

Two files in `Src/remora-core/` receive surgical changes to support the dual-core comms architecture (Phase 21). All other remora-core files remain untouched.

| File | Change |
|---|---|
| `remora.cpp` | Remove `comms->init()`, `comms->start()` from constructor; remove `comms->tasks()` from `run()` loop — comms lifecycle moves to Core 1 |
| `modules/comms/commsHandler.h` | `data` member becomes `volatile bool` — written by Core 1 comms ISR, read by Core 0 servo thread watchdog |

### Files completely unchanged (zero modifications to remora-core)

All other files in `Src/remora-core/` are untouched. This includes:
- `remora.h/.cpp`, `configuration.h`, `data.h`, `remoraStatus.h`
- All 13 module implementations (stepgen, softEncoder, digitalPin, analogPin, pwm, qei, sigmaDelta, temperature, blink, resetPin, tmc2208, tmc2209, tmc5160)
- `commsInterface.h/.cpp`, `commsHandler.h/.cpp`
- `jsonConfigHandler.h/.cpp`, `crc32.h`
- `pruThread.h/.cpp`, `pruTimer.h/.cpp`, `timerInterrupt.h/.cpp`
- `interrupt.h/.cpp`, `moduleInterrupt.h`, `module.h/.cpp`, `moduleFactory.h/.cpp`
- `W5500_Networking.h/.cpp` (delegates all SPI calls through `CommsInterface`)
- `TMCStepper` library, `SoftwareSPI`, `SoftwareSerial`
- `thermistor.h/.cpp` (except the ADC divisor bug fix)

---

## 21. Dual-Core Comms Isolation

### Motivation

The Base thread (40 kHz) and Servo thread (1 kHz) are hard real-time. On a single core, comms processing (SPI DMA completion ISR, memcpy, ETH polling) runs on the same core and can cause jitter. Offloading all comms to Core 1 gives the real-time threads uncontested access to Core 0.

### Core assignment

| Core | Responsibilities |
|---|---|
| **Core 0** | Base thread ISR (40 kHz), Servo thread ISR (1 kHz), Remora state machine, all module updates |
| **Core 1** | SPI or ETH comms init, DMA/GPIO IRQ handlers, `tasks()` tight loop |

### IRQ affinity

IRQ assignment follows which core calls the registration function:

| IRQ | Handler | Core | Registered by |
|---|---|---|---|
| `TIMER0_IRQ_0` | Base thread | Core 0 | `RP2350_timer` init, called from `Remora` constructor on Core 0 |
| `TIMER0_IRQ_1` | Servo thread | Core 0 | same |
| `DMA_IRQ_0` | SPI RX completion | Core 1 | `RP2350_SPIComms::init()` called from `core1_entry()` |
| `IO_IRQ_BANK0` (CS pin) | SPI CS edge | Core 1 | `gpio_set_irq_enabled(gpioCs, ...)` in `RP2350_SPIComms::init()` on Core 1 |
| `IO_IRQ_BANK0` (other pins) | QEI index, etc. | Core 0 | `gpio_set_irq_enabled(indexPin, ...)` called from module constructors on Core 0 |

On RP2350, each core has independent NVIC and independent `PROC0_INTE`/`PROC1_INTE` GPIO interrupt enable registers. The same `gpio_irq_dispatcher` callback is registered on both cores; both dispatch through the shared `Interrupt::ISRVectorTable`.

### Cross-core data access

`rxData` and `txData` are shared between cores. All struct members are 32-bit aligned, making per-field access atomic on Cortex-M33. The only field requiring explicit `volatile` is `CommsHandler::data` (a `bool`), which is written by the Core 1 comms interrupt and read by the Core 0 servo thread watchdog.

### Implementation

**`Src/remora-core/remora.cpp`** (2 changes):
- Constructor: remove `comms->init()` and `comms->start()` — comms lifecycle moves to `core1_entry()`
- `run()` loop: remove `comms->tasks()` calls — Core 1 runs `tasks()` in its own tight loop

**`Src/remora-core/modules/comms/commsHandler.h`** (1 change):
- `bool data` → `volatile bool data`

**`Src/main.cpp`** (additions):
- `static CommsHandler* g_commsHandler` — global raw pointer set before Core 1 launch
- `core1_entry()` — registers `gpio_irq_dispatcher` and enables `IO_IRQ_BANK0` on Core 1, calls `init()`/`start()`, then loops on `tasks()`
- `main()` calls `multicore_launch_core1(core1_entry)` after storing the pointer, then constructs and runs `Remora` as before

---

## 20. Implementation Sequence & Milestones

The phases are ordered to maximize early feedback and minimize blocked work. Each milestone should be verified before proceeding.

### Milestone 1 — Skeleton compiles and LED blinks

Complete Phase 1 (CMake), Phase 2 (Pin), and enough of Phase 3 (Timer) to start the servo thread. Verify: the project compiles, flashes to a Pico 2, and the `Blink` module running in the servo thread toggles GP25 at 4 Hz. This proves the entire build chain from remora-core JSON parsing through the thread system to GPIO output.

### Milestone 2 — Step pulses verified on logic analyzer

Complete Phase 3 (Timer) and Phase 4 (Interrupt dispatch) fully. Verify: a minimal JSON config with two Stepgen modules produces correct DDS step frequencies on two GPIO pins, confirmed with a logic analyzer. At 400 Hz frequency command, the oscilloscope should show exactly 400 Hz with no jitter beyond ±1 timer tick (±7 ns).

### Milestone 3 — SPI slave receives packets from Raspberry Pi (builtin config)

Use the `remora_rp2350_spi_builtin` build target (`SPI_CTRL=1`, `NO_FATFS_SPI=1`).  This target has full SPI slave DMA comms but loads the hardcoded default config from flash — no SD card is required.

**Build:** flash `remora_rp2350_spi_builtin.uf2`.

**Wiring:** connect the Raspberry Pi SPI master to the RP2350 SPI0 slave pins:

| Signal | RP2350 GPIO |
|--------|-------------|
| CLK    | GP2         |
| MOSI   | GP3         |
| MISO   | GP4         |
| CS     | GP5         |

**Verification steps:**

1. Open the USB CDC serial port before powering on.
2. Observe the startup sequence — config loads from the hardcoded default (no SD card mount attempted), two Stepgen modules and a Blink module are created, threads start.
3. Start the `remora-spi` LinuxCNC component on the Raspberry Pi.
4. Confirm `## Transitioning to Running state` appears on the serial port within the 100 ms comms watchdog window.
5. In the LinuxCNC HAL, verify `joint.0.pos-fb` and `joint.1.pos-fb` respond to step commands — confirms the full DMA → rxData → Stepgen → txData → DMA pipeline.
6. Verify the onboard LED continues blinking at 2 Hz while in `ST_RUNNING`, proving the servo thread is not starved by comms processing.

**Note:** The hardcoded default config uses GP7/GP10 for step output. Confirm these pins toggle when LinuxCNC issues motion commands.

### Milestone 3b — SPI slave with SD card config

Use the `remora_rp2350_spi` build target (`SPI_CTRL=1`, real FatFs).  This target loads `config.txt` from an SD card, enabling custom module configurations without recompiling.

**Prerequisites:**
- `no-OS-FatFS-SD-SPI-RPi-Pico` submodule present (`git submodule update --init`).
- FAT32-formatted SD card with a valid `config.txt` in the root directory.
- SD card wired to SPI1: SCK=GP10, MOSI=GP11, MISO=GP12, CS=GP13.

**Verification steps:**

1. Write a `config.txt` with three Stepgen joints plus any Digital Pin or Blink modules needed for the target machine.
2. Flash `remora_rp2350_spi.uf2`.
3. Confirm the serial output shows `JSON config file read SUCCESS!` and lists the modules being created from the SD card config.
4. If the SD card is absent, confirm the firmware falls back to the hardcoded default config (not a fatal halt) — serial output shows `SD card not detected — falling back to default config`.
5. Connect the RPi running `remora-spi` and verify `ST_RUNNING` is reached with the custom config active.
6. Verify all joints in the custom config appear in LinuxCNC HAL with correct feedback.

### Milestone 4 — Full SPI motion test

Add Digital Pin, Analog Pin, PWM, and QEI modules to the test config. Verify: digital outputs toggle in response to HAL write commands; ADC values appear in processVariable slots; PWM duty cycle changes track the setPoint; encoder counts track a driven encoder.

### Milestone 4b — Dual-core comms isolation

Complete Phase 21 (Dual-Core). Verify:
1. Serial output shows `Core 1: comms running` after boot.
2. Full SPI motion test still passes (LinuxCNC reaches `ST_RUNNING`, joints respond).
3. The Base and Servo threads are serviced exclusively by Core 0 (confirmed by checking that `DMA_IRQ_0` and the SPI CS GPIO interrupt are registered on Core 1's NVIC).
4. Step pulse timing is unchanged or improved (no jitter increase) — confirms Core 0 real-time threads are no longer interrupted by comms processing.

### Milestone 5 — Ethernet comms operational

Complete Phase 6 (ETH Comms). Verify: `upload_config.py` successfully uploads a config file over TFTP. After reboot, the firmware loads the new config from flash. LinuxCNC connects over UDP and reaches `ST_RUNNING`.

### Milestone 6 — Flash config storage

Complete Phase 10 (Flash). Verify: the CRC32 check passes after a TFTP upload. `store_json_in_flash()` writes to the correct flash region. After a hard power-cycle (not just reboot), the config is reloaded correctly from flash.

### Milestone 7 — TMC driver configuration

Complete Phase 12 (Serial Thread). Verify: a TMC2209 driver connects over single-wire UART, `test_connection()` returns OK, and the driver is configured with the expected current and microstep values readable via a TMC register dump.

### Milestone 8 — Full system acceptance test

Run a complete LinuxCNC milling machine cycle with all axes, spindle, home switches, and limit switches connected. Confirm no missed steps, no comms dropouts, and correct emergency-stop behavior.

---

*This porting plan is based on complete static analysis of the Remora STM32F4xx PIO codebase and remora-core. All architectural decisions are grounded in the existing code structure documented in `research.md`.*
