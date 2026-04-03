# Remora STM32F4xx PIO — Complete Codebase Analysis

> Prepared: March 2026  
> Sources: `Remora-STM32F4xx-PIO-main.zip` + `remora-core-8d3b6090.zip`  
> Repository (HAL): github.com/ben-jacobson/Remora-STM32F4xx-PIO  
> Repository (Core): github.com/ben-jacobson/remora-core (branch: PWM_module)

---

## Table of Contents

1. [Project Overview](#1-project-overview)
2. [Full Repository Structure](#2-full-repository-structure)
3. [Build System & Configuration](#3-build-system--configuration)
4. [Hardware Targets & Memory Map](#4-hardware-targets--memory-map)
5. [Complete Class Architecture & Hierarchy](#5-complete-class-architecture--hierarchy)
6. [remora-core Deep-Dive](#6-remora-core-deep-dive)
   - 6.1 The Remora State Machine
   - 6.2 Data Structures (data.h)
   - 6.3 Configuration Namespace (configuration.h)
   - 6.4 Status & Error System (remoraStatus.h)
   - 6.5 Thread System (pruThread, pruTimer, TimerInterrupt)
   - 6.6 Interrupt Dispatch System (Interrupt, ModuleInterrupt)
   - 6.7 Module System (Base Class, Factory, List)
   - 6.8 Communications Layer (CommsInterface, CommsHandler)
   - 6.9 JSON Configuration Handler
   - 6.10 CRC32 Engine
7. [remora-core Modules — All Implementations](#7-remora-core-modules--all-implementations)
   - 7.1 Stepgen (DDS Step Pulse Generator)
   - 7.2 SoftEncoder (Software Quadrature Decoder)
   - 7.3 DigitalPin (GPIO Input/Output)
   - 7.4 AnalogPin (ADC Input)
   - 7.5 PWM (Hardware Timer PWM Output)
   - 7.6 QEI (Hardware Quadrature Encoder Interface)
   - 7.7 SigmaDelta (1-bit DAC via Bit-Bang)
   - 7.8 Temperature (Thermistor NTC)
   - 7.9 Blink (Debug LED)
   - 7.10 ResetPin (Hardware Reset Input)
   - 7.11 TMC Stepper Drivers (TMC2208, TMC2209, TMC5160)
8. [remora-core Drivers](#8-remora-core-drivers)
   - 8.1 W5500 Networking (LwIP + WIZnet + TFTP)
   - 8.2 SoftwareSPI Driver
   - 8.3 SoftwareSerial Driver
   - 8.4 TMCStepper Library
9. [remora-hal HAL Abstraction Layer (STM32F4)](#9-remora-hal-hal-abstraction-layer-stm32f4)
   - 9.1 Startup & Main Entry Point
   - 9.2 Pin Abstraction
   - 9.3 Timer Implementation (STM32F4_timer)
   - 9.4 SPI Communications — Slave Mode (STM32F4_SPIComms)
   - 9.5 Ethernet Communications — W5500 Master (STM32F4_EthComms)
   - 9.6 Hardware PWM Driver
   - 9.7 Hardware QEI Driver
   - 9.8 Analog Input (ADC)
   - 9.9 Shared Peripheral Handle Management
   - 9.10 Board Status LED & Error Codes
   - 9.11 Platform Configuration & Flash Layout
10. [LinuxCNC Host-Side Components](#10-linuxcnc-host-side-components)
11. [Communication Protocol](#11-communication-protocol)
12. [Flash Memory & JSON Configuration System](#12-flash-memory--json-configuration-system)
13. [Real-Time Threading Model](#13-real-time-threading-model)
14. [Complete Data Flow: End-to-End](#14-complete-data-flow-end-to-end)
15. [Design Patterns & Engineering Decisions](#15-design-patterns--engineering-decisions)
16. [Known Limitations, Caveats & TODOs](#16-known-limitations-caveats--todos)
17. [Summary Findings Table](#17-summary-findings-table)

---

## 1. Project Overview

Remora is an open-source firmware project that turns an STM32F4-series microcontroller into a **real-time motion-control co-processor** for LinuxCNC. LinuxCNC runs on a general-purpose Linux PC, which lacks the real-time guarantees needed for precision stepper-motor pulse generation. Remora offloads this time-critical work to a dedicated MCU connected via either SPI (slave to Raspberry Pi) or Ethernet (W5500 UDP).

The system is split into two distinct repositories:

- **remora-core** — Platform-agnostic real-time engine: state machine, threads, module system, JSON config, comms protocol, and all hardware module logic. This is the "brain."
- **remora-hal** (STM32F4xx-PIO) — STM32F4-specific hardware drivers implementing the abstract interfaces defined by remora-core. This is the "hands."

Authors: Scott Alford ("scotta") as the original Remora architect; Ben Jacobson for the STM32F4 port; Cakeslob and Expatria Technologies for Ethernet communications. Version: 2.0.0 (as declared in `remora.h`).

---

## 2. Full Repository Structure

```
remora-core/
├── remora.h / remora.cpp           — Top-level state machine & orchestration
├── configuration.h                 — All compile-time constants (Config namespace)
├── data.h                          — Shared TX/RX data structures (64-byte packed unions)
├── remoraStatus.h                  — Structured error/status byte encoding
├── comms/
│   └── commsInterface.h/.cpp       — Abstract base class for all comms transports
├── thread/
│   ├── pruThread.h/.cpp            — RT thread: module list, start/stop, update loop
│   ├── pruTimer.h/.cpp             — Abstract timer base (frequency, owner, start/stop)
│   └── timerInterrupt.h/.cpp       — Connects timer IRQ to pruTimer::timerTick()
├── interrupt/
│   └── interrupt.h/.cpp            — Static ISR vector table (size: PERIPH_COUNT_IRQn)
├── modules/
│   ├── module.h/.cpp               — Module base class (update/slowUpdate/configure)
│   ├── moduleFactory.h/.cpp        — Singleton factory: JSON type string to Module
│   ├── moduleInterrupt.h           — Template: binds member function to IRQ
│   ├── moduleList.h                — Includes all module headers
│   ├── stepgen/                    — DDS stepper pulse generator
│   ├── softEncoder/                — Software quadrature decoder (state machine)
│   ├── digitalPin/                 — GPIO input/output
│   ├── analogPin/                  — ADC read to process variable
│   ├── pwm/                        — Hardware timer PWM (wraps HardwarePWM HAL)
│   ├── qei/                        — Hardware quadrature encoder (wraps Hardware_QEI HAL)
│   ├── sigmaDelta/                 — 1-bit sigma-delta modulator output
│   ├── temperature/                — Thermistor ADC to degrees C
│   ├── tmc/                        — TMC2208, TMC2209, TMC5160 stepper driver config
│   ├── blink/                      — Debug LED blinker
│   ├── resetPin/                   — Hardware reset input pin
│   └── comms/commsHandler.h/.cpp   — Servo-thread comms watchdog module
├── json/
│   ├── jsonConfigHandler.h/.cpp    — JSON load/parse/store/CRC verify
│   └── crc32.h                     — Software CRC32 (table-based, IEEE 802.3 poly)
├── sensors/
│   ├── tempSensor.h                — Abstract temperature sensor base class
│   └── thermistor/                 — Beta-equation NTC thermistor implementation
└── drivers/
    ├── W5500_Networking/           — LwIP + WIZnet W5500 + TFTP server
    ├── SoftwareSPI/                — Bit-banged SPI (used by TMC5160)
    ├── SoftwareSerial/             — Bit-banged UART (used by TMC2208/TMC2209)
    └── TMCStepper/                 — TMC stepper driver register interface library

remora-hal/ (STM32F4xx-PIO)
├── Src/main.cpp                    — Entry point, peripheral init, object graph construction
├── Src/irqHandlers.h               — All ISR definitions (timers, DMA, EXTI, SPI)
├── Src/hardware.h                  — PERIPH_COUNT_IRQn constant (149)
├── Src/remora-hal/
│   ├── STM32F4_timer.h/.cpp        — pruTimer to STM32 TIM peripheral
│   ├── STM32F4_SPIComms.h/.cpp     — CommsInterface: SPI slave + double-buffer DMA
│   ├── STM32F4_EthComms.h/.cpp     — CommsInterface: SPI master to W5500
│   ├── pin/pin.h/.cpp              — GPIO abstraction (string "PA_0" format)
│   ├── analogIn/analogIn.h/.cpp    — ADC single-channel blocking read
│   ├── hardware_pwm/               — TIM-based hardware PWM (linked list for shared timers)
│   ├── hardware_qei/               — TIM8 quadrature encoder + EXTI index
│   ├── shared_handlers.h/.cpp      — Global TIM/ADC handle pool
│   ├── hal_utils.h/.cpp            — SPI bus discovery, flash erase, pin map utils
│   ├── board_led_status.h/.cpp     — Error-code blink sequences
│   ├── platform_configuration.h   — Flash address constants from linker symbols
│   ├── pinNames.h                  — Full STM32F4 PinName enum
│   └── peripheralPins.h/.c         — mbed-style peripheral pin map tables
├── FATFS/                          — STM32Cube FatFs integration (SD card)
├── LinkerScripts/                  — Custom LD scripts (ETH, SPI, bootloader variants)
└── LinuxCNC/                       — Host-side HAL components and example configs
```

---

## 3. Build System & Configuration

The project uses PlatformIO (`platform = ststm32`, `framework = stm32cube`). Build environments are composed via inheritance to avoid duplication:

```
[STM32F4xx_global]          — FatFs, ArduinoJson, HSE override
  ├── [STMF4xx_eth]         — ETH_CTRL, W5500 library, SPI pins, WIZ_RST
  │     ├── nucleo_f446ze_eth   — USART3, ETH linker script
  │     ├── nucleo_f446re_eth   — USART2, ETH linker script
  └── [STMF4xx_spi]         — SPI_CTRL, SPI pins
        ├── nucleo_f446ze_spi
        ├── nucleo_f446re_spi
        └── Octopus_446_spi     — HAS_BOOTLOADER, shifted flash
```

**Key compile-time flags:**

| Flag | Effect |
|---|---|
| `ETH_CTRL=1` | Enables Ethernet comms path, W5500 library, TFTP, flash JSON storage |
| `SPI_CTRL=1` | Enables SPI slave path, SD card FatFs config loading |
| `HAS_BOOTLOADER=1` | Relocates VTOR, uses shifted linker script |
| `WIZ_RST`, `SPI_CS/CLK/MISO/MOSI` | Hardware pin assignments |
| `HSE_VALUE=8000000U` | Overrides PlatformIO's wrong HSE default for F4 |
| `SPI_CS_IRQ=EXTI4_IRQn` | NSS interrupt line for SPI slave |
| `UART_PORT=USART2/3` | Debug printf retarget |

**External dependencies:** ArduinoJson v7.4.1 (JSON parsing), FatFs (SD card), W5500_Networking ZIP library (ETH builds only).

---

## 4. Hardware Targets & Memory Map

### Clock Configuration (Nucleo F446)
- **HSE**: 8 MHz, **PLL**: M=4, N=168, P=2 — **SYSCLK = 168 MHz**
- **APB1 timers** (TIM2-TIM7): 84 MHz | **APB2 timers** (TIM1, TIM8-11): 168 MHz

### Flash Memory Layout (ETH build, no bootloader)

| Address | Region | Size | Sector | Purpose |
|---|---|---|---|---|
| `0x08000000` | FLASH_BOOT | 16 KB | 0 | ISR vectors, JTAG entry |
| `0x08004000` | JSON_UPLOAD | 16 KB | 1 | TFTP config staging area |
| `0x08008000` | JSON_STORAGE | 16 KB | 2 | Active persistent config |
| `0x0800C000` | FLASH_PGM | 464 KB | 3-7 | Application code |
| `0x20000000` | RAM | 128 KB | — | Data/BSS/Stack/Heap |

The Octopus board shifts the entire map up by 64 KB (4 sectors) to accommodate the BTT bootloader.

### JSON Flash Metadata Structure
At the very start of the JSON_UPLOAD region, a 512-byte packed struct `json_metadata_t` stores:
- `uint32_t crc32` — CRC32 of the JSON content (IEEE 802.3 polynomial, initial `0xFFFFFFFF`)
- `uint32_t length` — length in words for CRC verification
- `uint32_t jsonLength` — length of JSON in bytes
- `uint8_t padding[500]` — padded to exactly 512 bytes (one TFTP block)

---

## 5. Complete Class Architecture & Hierarchy

```
Interrupt  (static vector table, PERIPH_COUNT_IRQn=149 slots)
├── TimerInterrupt              — ISR_Handler() calls pruTimer::timerTick()
└── ModuleInterrupt<T>          — ISR_Handler() calls T::memberFn()  [template]

Module  (base class)
├── CommsInterface              — Abstract comms transport
│   ├── STM32F4_SPIComms        — SPI slave + DMA double-buffer
│   └── STM32F4_EthComms        — SPI master to W5500
├── CommsHandler                — Servo-thread watchdog, wraps CommsInterface
├── Stepgen                     — DDS step generator (Base thread)
├── SoftEncoder                 — Software QEI state machine (Base thread)
├── DigitalPin                  — GPIO input/output (Servo thread)
├── AnalogPin                   — ADC read (Servo thread)
├── PWM                         — Hardware PWM via HardwarePWM (Servo thread)
├── QEI                         — Hardware QEI via Hardware_QEI (Servo thread)
├── SigmaDelta                  — 1-bit sigma-delta DAC (Servo thread)
├── Temperature                 — Thermistor ADC (Servo thread, 1 Hz slow update)
├── Blink                       — Debug LED toggle (Servo thread)
├── ResetPin                    — Hardware reset input (Servo thread)
└── TMC  (base, enable_shared_from_this)
    ├── TMC2208                 — UART half-duplex (On load)
    ├── TMC2209                 — UART half-duplex + stall (On load)
    └── TMC5160                 — SPI full-duplex (On load)

pruTimer  (abstract)
└── STM32F4_timer               — Maps to TIM2/TIM3/TIM4 hardware

pruThread  (contains vector<shared_ptr<Module>>)
    owns a unique_ptr<pruTimer>

Remora  (top-level orchestrator)
    owns shared_ptr<CommsHandler>
    owns unique_ptr<pruThread> baseThread, servoThread, serialThread
    owns unique_ptr<JsonConfigHandler>
    manages onLoad vector<shared_ptr<Module>>

TempSensor  (abstract sensor base)
└── Thermistor                  — Beta-equation NTC using AnalogIn

HardwarePWM     — STM32 TIM PWM (intrusive linked list for shared TIMx)
Hardware_QEI    — STM32 TIM8 encoder + EXTI index interrupt
AnalogIn        — STM32 ADC polling read
Pin             — STM32 GPIO (string "PA_0" format)
```

---

## 6. remora-core Deep-Dive

### 6.1 The Remora State Machine

`Remora` is the top-level orchestrator. Its `run()` method is an infinite loop and never returns. The state machine has seven states:

```
ST_SETUP  →  ST_START  →  ST_IDLE  ⇄  ST_RUNNING
                                           ↓ comms lost
               ST_RESET  ←─────────────────
                    ↓
               ST_IDLE  (buffer cleared, wait for comms)
                                           ↓ reset pin asserted
               ST_SYSRESET (HAL_NVIC_SystemReset)
```

State behaviors:

**ST_SETUP**: Calls `loadModules()` — parses JSON, creates all module objects, registers them with their threads. Transitions immediately to ST_START.

**ST_START**: Runs `configure()` on all "On load" modules (e.g., TMC driver init). Starts servo and base threads. Transitions to ST_IDLE.

**ST_IDLE**: Polls `comms->getStatus()`. Once the `CommsHandler` confirms valid data received, transitions to ST_RUNNING.

**ST_RUNNING**: Monitors for comms loss (`!comms->getStatus()`) which triggers ST_RESET, or for `reset == true` (from ResetPin module) which triggers ST_SYSRESET.

**ST_RESET**: Calls `memset` on the rxBuffer to zero all commands, then transitions back to ST_IDLE.

**ST_SYSRESET**: Calls `HAL_NVIC_SystemReset()` — full MCU reboot.

**Fatal error bypass**: If `remoraStatus & 0x80` (the fatal bit), the state machine is bypassed entirely. The `run()` loop still calls `comms->tasks()` to maintain network keep-alive but processes no module updates. LinuxCNC sees the error in the TX header's status byte.

**ETH-specific JSON upload check**: In each main loop iteration for ETH builds, `run()` checks `JsonConfigHandler::new_flash_json`. If set (by the TFTP callback), it validates and commits the new config to flash and triggers a reboot.

**Constructor sequence**: Initializes threads, passes timers to them, creates a `JsonConfigHandler` (which immediately loads and parses config), calls `comms->init()` and `comms->start()`, and registers `comms` (the `CommsHandler`) as a module on the servo thread.

**Thread frequency setter limitation**: The `setBaseFreq()` and `setServoFreq()` methods update the frequency stored in `Remora` (used for module `frequencyScale` calculations) but the lines that would actually update the timer are commented out. JSON `"Threads"` frequency overrides cannot change the real hardware interrupt rate at runtime.

### 6.2 Data Structures (data.h)

The data structures are 64-byte packed unions (`#pragma pack(push, 1)`) declared `__attribute__((aligned(32)))`, shared between the MCU and LinuxCNC.

**`rxData_t` (STM32 receives from LinuxCNC):**

| Bytes | Type | Field | Purpose |
|---|---|---|---|
| 0-3 | int32_t | header | PRU_READ or PRU_WRITE magic word |
| 4-35 | int32_t[8] | jointFreqCmd | Base thread step frequency commands |
| 36-59 | float[6] | setPoint | Servo thread commands (PWM, PID SP) |
| 60 | uint8_t | jointEnable | Bit N enables joint N |
| 61-62 | uint16_t | outputs | Digital output bitmask |
| 63 | uint8_t | spare0 | Reserved |

**`txData_t` (STM32 sends to LinuxCNC):**

| Bytes | Type | Field | Purpose |
|---|---|---|---|
| 0-3 | int32_t | header | PRU_DATA or PRU_ACKNOWLEDGE + remoraStatus |
| 4-35 | int32_t[8] | jointFeedback | Step counts for position feedback |
| 36-59 | float[6] | processVariable | Temp, encoder counts, ADC values |
| 60-61 | uint16_t | inputs | Digital input bitmask |
| 62-63 | — | padding | — |

Both unions have **constructors** that zero all fields on creation. This is unusual for embedded code and ensures no stale motion commands can appear at startup.

**`DMA_RxBuffer_t`**: A struct holding two `rxData_t` buffers for double-buffered DMA in the SPI slave path.

**Important discrepancy**: The `rxData_t.outputs` field is `uint16_t` (16 bits), but the LinuxCNC `remora-eth` component declares `DIGITAL_OUTPUTS = 16` and `outputs` as `uint32_t` in its data union. The firmware can only represent 16 outputs.

### 6.3 Configuration Namespace (configuration.h)

All compile-time constants live in the `Config` namespace:

| Constant | Value | Meaning |
|---|---|---|
| `pruBaseFreq` | 40000 | Base thread frequency Hz |
| `pruServoFreq` | 1000 | Servo thread frequency Hz |
| `oversample` | 3 | Software serial oversampling ratio |
| `swBaudRate` | 19200 | Software serial baud rate |
| `pruSerialFreq` | 57600 | Serial thread rate (19200 x 3) |
| `stepBit` | 22 | DDS accumulator step detection bit |
| `joints` | 8 | Maximum joint count |
| `variables` | 6 | Number of setpoint/PV slots |
| `dataErrMax` | 100 | Servo cycles without data before comms lost |
| `dataBuffSize` | 64 | SPI/UDP packet size in bytes |
| `ip_address` | `{10,10,10,10}` | Hardcoded MCU IP (ETH only) |
| `subnet_mask` | `{255,255,255,0}` | Hardcoded subnet |
| `gateway` | `{10,10,10,1}` | Hardcoded gateway |

The `defaultConfig` is a hardcoded byte array (the hex representation of a minimal JSON running a 4 Hz LED blink on PB_0). This is the safe fallback loaded when flash is blank or unreadable.

IRQ priorities (lower = higher priority):

| IRQ | Priority |
|---|---|
| Base thread (TIM3) | 1 (highest) |
| Servo thread (TIM2) | 2 |
| Serial thread (TIM4) | 3 |
| SPI DMA TX | 4 |
| SPI DMA RX | 5 |
| SPI NSS (chip select) | 6 |
| QEI index pulse | 7 |

### 6.4 Status & Error System (remoraStatus.h)

The `remoraStatus` byte (sent in every TX packet header LSB) encodes errors in a structured 8-bit field:

```
Bit 7   : FATAL flag (1 = fatal, state machine halted)
Bits 6-4: Error source (3 bits)
Bits 3-0: Error code   (4 bits)
```

| Source | Value | Codes |
|---|---|---|
| `NO_ERROR` | 0x00 | `NO_ERROR` |
| `CORE` | 0x10 | `REMORA_CORE_ERROR` |
| `JSON_CONFIG` | 0x20 | `SD_MOUNT_FAILED`, `CONFIG_FILE_OPEN_FAILED`, `CONFIG_FILE_READ_FAILED`, `CONFIG_INVALID_INPUT`, `CONFIG_NO_MEMORY`, `CONFIG_PARSE_FAILED`, `CONFIG_LOADED_DEFAULT` |
| `MODULE_LOADER` | 0x30 | `MODULE_CREATE_FAILED` |
| `TMC_DRIVER` | 0x40 | `TMC_DRIVER_ERROR` |

`makeRemoraStatus(source, code, fatal)` is an inline helper combining these into the single byte. A fatal status (`bit 7 = 1`) halts the state machine but keeps comms running. `CONFIG_LOADED_DEFAULT` (0x27) is non-fatal — the system runs the blink program while waiting for a real config.

### 6.5 Thread System (pruThread, pruTimer, TimerInterrupt)

**`pruTimer`** is the abstract base for hardware timers. It holds a `unique_ptr<TimerInterrupt>`, a frequency, an owner thread pointer, and a running flag. `setFrequency()` respects running state: stops, reconfigures, and restarts atomically.

**`TimerInterrupt`** inherits from `Interrupt` and calls `interruptOwnerPtr->timerTick()` in its `ISR_Handler()`. The interrupt chain is: hardware ISR → `Interrupt::InvokeHandler()` → `TimerInterrupt::ISR_Handler()` → `pruTimer::timerTick()` → `pruThread::update()` → module execution.

**`pruThread`** owns:
- `unique_ptr<pruTimer> timerPtr`
- `vector<shared_ptr<Module>> modules` — called every tick via `runModule()`
- `vector<shared_ptr<Module>> modulesPost` — called after all modules via `runModulePost()`
- `atomic<bool> threadRunning`, `atomic<bool> threadPaused` — thread-safe state with explicit `memory_order_release`/`memory_order_acquire`

The `update()` method checks both `isRunning()` and `isPaused()` before calling `executeModules()`.

The **post-module system** exists specifically for `Stepgen`: `update()` calls `makePulses()` to raise the step pin HIGH; `updatePost()` calls `stopPulses()` to lower it. This guarantees step pulses are exactly one thread period (25 µs at 40 kHz) wide regardless of other module activity.

### 6.6 Interrupt Dispatch System

**`Interrupt`** maintains a static vector table `ISRVectorTable[PERIPH_COUNT_IRQn]` (149 slots for STM32F4). `Register(irqN, ptr)` stores the handler; `InvokeHandler(irqN)` calls it. All hardware ISRs in `irqHandlers.h` funnel into `InvokeHandler()`.

**`ModuleInterrupt<T>`** is a template class inheriting `Interrupt`. It stores a `T*` instance pointer and a `void (T::*)()` member function pointer. Its `ISR_Handler()` calls the member function on the instance. This is the cleanest solution to the C ISR / C++ member function problem without global instance variables.

### 6.7 Module System (Base Class, Factory, List)

**`Module`** base class has two constructors:
1. `Module()` — runs at full thread frequency (`updateCount = 1`).
2. `Module(threadFreq, slowUpdateFreq)` — computes `updateCount = threadFreq / slowUpdateFreq`. `runModule()` calls `slowUpdate()` every `updateCount` ticks, then always calls `update()`.

Virtual methods: `update()`, `updatePost()`, `slowUpdate()`, `configure()`. All default to no-ops.

**`ModuleFactory`** is a singleton (static instance). Its `createModule()` method maps `(threadName, moduleType)` string pairs to concrete `Module::create()` static factory methods. Supported types by thread:

| Thread | Module Types |
|---|---|
| `Base` | `Stepgen`, `Encoder` (SoftEncoder) |
| `Servo` | `Blink`, `Reset Pin`, `Digital Pin`, `Sigma Delta`, `Temperature`, `PWM`, `Analog Pin`, `QEI` |
| `On load` | `TMC2208`, `TMC2209`, `TMC5160` |

Adding a new module type requires editing `moduleFactory.cpp` — there is no registration mechanism.

### 6.8 Communications Layer (CommsInterface, CommsHandler)

**`CommsInterface`** inherits from `Module`. It holds `volatile rxData_t*` and `volatile txData_t*` pointers plus a `std::function<void(bool)> dataCallback`. All virtual transport methods (`init`, `start`, `tasks`, `read_byte`, `write_byte`, `DMA_write`, `DMA_read`, `flag_new_data`) default to no-ops.

**`CommsHandler`** wraps a `unique_ptr<CommsInterface>`. In `init()`, it sets the `dataCallback` lambda on the interface. In `update()` (called every servo tick): if `data` is true, resets `noDataCount` and sets `status = true`; otherwise increments `noDataCount`. If `noDataCount > 100`, sets `status = false` (comms lost). In `tasks()` (main loop): delegates to `interface->tasks()` for polling.

The watchdog: 100 consecutive servo cycles (100 ms) without valid data signals comms loss, driving ST_RUNNING → ST_RESET.

### 6.9 JSON Configuration Handler

`JsonConfigHandler` uses **ArduinoJson v7** (`JsonDocument`) to parse machine configuration.

**Load path selection:** ETH builds use `readConfigFromFlash()`; SPI builds use `readConfigFromSD()`.

**Flash load**: Reads 4-byte `jsonLength` at `JSON_storage_start_address`. If `0xFFFFFFFF` (erased), loads the hardcoded default blink config and returns `CONFIG_LOADED_DEFAULT` (non-fatal). Otherwise reads `jsonLength` bytes from flash into `std::string jsonContent`.

**SD load**: Mounts SDIO FatFs, opens `config.txt`, allocates a VLA `char rtext[length]` on the stack, reads all bytes, then copies character-by-character into `jsonContent` (O(n²) — see limitations section).

**Thread frequency override**: An optional `"Threads"` JSON array can specify `{"Thread": "Base", "Frequency": 40000}` to customize thread rates. These call `setBaseFreq()`/`setServoFreq()` — but as noted in section 6.1, this only affects the DDS frequency scale used by modules, not the actual timer interrupt rate.

**`json_check_length_and_CRC()`**: Called from `Remora::run()` after TFTP upload. Reads `json_metadata_t` from JSON_UPLOAD flash, generates the CRC32 table, computes CRC over `jsonLength + padding` bytes. Padding aligns to 4-byte boundary for STM32 hardware CRC32 compatibility. Returns `1` on success, `-1` on failure.

**`store_json_in_flash()`** (ETH only): Erases JSON_STORAGE sector, writes `jsonLength` as a word to the first 4 bytes, then copies JSON content byte-by-byte from JSON_UPLOAD to JSON_STORAGE flash using `write_to_flash_byte()`.

### 6.10 CRC32 Engine (crc32.h)

A header-only software CRC32 using the **IEEE 802.3 polynomial** (`0xEDB88320` reflected). API:
- `crc32::generate_table(table[256])` — pre-computes the 256-entry lookup table on the stack (1 KB).
- `crc32::update(table, initial, buf, len)` — processes `len` bytes, XORs result with `0xFFFFFFFF`.

Called from `json_check_length_and_CRC()` with `initial = 0xFFFFFFFF`. Note: the table is allocated on the stack each time `generate_table` is called — 1024 bytes of stack consumed during CRC verification.

---

## 7. remora-core Modules — All Implementations

### 7.1 Stepgen (DDS Step Pulse Generator)

**Thread**: Base (40 kHz) | **JSON keys**: `"Joint Number"`, `"Enable Pin"`, `"Step Pin"`, `"Direction Pin"`

The most critical module. Generates stepper motor pulses using **Direct Digital Synthesis (DDS)**:

```
frequencyScale = (1 << 22) / 40000 = 104.857...

Each 40 kHz tick:
  DDSaddValue = jointFreqCmd[N] * frequencyScale
  prev = DDSaccumulator
  DDSaccumulator += DDSaddValue
  changed = prev XOR DDSaccumulator
  step = changed & (1 << 22)
  if step: raise stepPin, rawCount += direction
```

The XOR trick (`stepNow ^= DDSaccumulator; stepNow &= stepMask`) detects when bit 22 transitions. Because the DDS accumulates continuously, the step rate is extremely precise — a 400 Hz command at 40 kHz base produces exactly one step per 100 ticks with zero jitter from integer rounding.

**Enable pin polarity**: `enablePin.set(true)` **disables** the driver (active-low enable is standard for stepper drivers). `enablePin.set(false)` enables it. This counterintuitive naming is consistent throughout.

**Post-update pulse width**: Step pin is raised in `update()` (first pass via `modules`) and lowered in `updatePost()` (second pass via `modulesPost`). At 40 kHz, step pulse width is exactly 25 µs.

**Direction**: Set from the sign of `DDSaddValue`. No direction setup time is enforced before the step edge.

**Feedback**: `rawCount` is written to `txData.jointFeedback[N]` after every step — LinuxCNC uses this for closed-loop position tracking.

### 7.2 SoftEncoder (Software Quadrature Decoder)

**Thread**: Base (40 kHz) | **JSON keys**: `"ChA Pin"`, `"ChB Pin"`, `"Index Pin"` (optional), `"PV[i]"`, `"Data Bit"`, `"Modifier"`

Implements a 4× quadrature decoder using the **state machine approach** (credited to Paul Stoffregen's Encoder library). The 4-bit state variable (2 bits previous + 2 bits current) maps to a 16-entry lookup table giving +1, -1, +2, -2, or 0 delta:

```cpp
uint8_t s = state & 3;       // previous A, B
if (pinA->get()) s |= 4;     // current A
if (pinB->get()) s |= 8;     // current B
// switch(s) → count delta
state = (s >> 2);
```

**Index handling**: When the index pin goes high and `pulseCount == 0`, the current count is captured and the index output bit is set. `pulseCount` is set to `(40000/1000) * 3 = 120` ticks — holding the index signal high for 3 servo thread periods so LinuxCNC reliably detects it.

**Maximum frequency**: At 40 kHz polling, the SoftEncoder reliably captures up to ~10 kHz quadrature signals. For faster spindles, the hardware QEI module is necessary.

### 7.3 DigitalPin (GPIO Input/Output)

**Thread**: Servo (1 kHz) | **JSON keys**: `"Pin"`, `"Mode"` (Input/Output), `"Data Bit"`, `"Invert"`, `"Modifier"`

For output mode: reads bit `bitNumber` from `rxData.outputs`, applies optional invert, sets the GPIO. For input mode: reads the GPIO, applies optional invert, sets bit `bitNumber` in `txData.inputs`. The `ptrData` pointer is selected at construction time. Unique among modules: uses `unique_ptr<Pin>` for RAII GPIO management.

### 7.4 AnalogPin (ADC Input)

**Thread**: Servo (1 kHz) | **JSON keys**: `"Pin"`, `"PV[i]"`

Constructs an `AnalogIn` HAL object and calls `adc->read()` every servo tick, writing the 12-bit ADC value (0-4095) as a `float` into `txData.processVariable[pv]`. Raw ADC counts are sent without scaling — LinuxCNC must apply scaling on the host side.

### 7.5 PWM (Hardware Timer PWM Output)

**Thread**: Servo (1 kHz) | **JSON keys**: `"PWM Pin"`, `"SP[i]"`, `"Period SP[i]"`, `"Period us"`, `"Variable Freq"`, `"PWM Max"`, `"Hardware PWM"`

Wraps `HardwarePWM`. Reads `rxData.setPoint[period_sp]` (if variable frequency) and `rxData.setPoint[sp]` each servo tick. Calls `hardware_PWM->change_period()` or `change_pulsewidth()` only when values change. `PWMMAX = 256` is used for the duty ceiling check: if `(duty / 100) * 256 > pwmMax`, duty is clamped. Software PWM is explicitly unsupported.

### 7.6 QEI (Hardware Quadrature Encoder Interface)

**Thread**: Servo (1 kHz) | **JSON keys**: `"PV[i]"`, `"Modifier"`, `"Enable Index"` (True/False), `"Data Bit"`

Wraps `Hardware_QEI`. Reads TIM8 counter right-shifted by 2 (4× correction). Implements the same 100-tick index hold as SoftEncoder. `"Open Drain"` modifier maps to `GPIO_PULLUP` (not yet implemented as true open drain — documented limitation in the code).

### 7.7 SigmaDelta (1-bit DAC via Bit-Bang)

**Thread**: Servo (1 kHz) | **JSON keys**: `"SD Pin"`, `"SP[i]"`, `"SD Max"` (optional)

Implements first-order sigma-delta modulation on a GPIO pin to approximate a slow analog output. The accumulator-based algorithm:
- Reads setpoint (0-100%) from `rxData.setPoint[i]`, scales to 0-SDmax.
- Not at boundaries: if accumulating up, add `setPoint`; when accumulator passes `SDmax/2`, reverse direction and subtract `(SDmax - setPoint)`.
- Pin is driven from the `SDdirection` flag.

At 1 kHz servo rate this produces a 1 kHz bit stream suitable for heating element control or similar slow actuators.

### 7.8 Temperature (Thermistor NTC)

**Thread**: Servo (1 kHz), **slow update at 1 Hz** | **JSON keys**: `"PV[i]"`, `"Sensor"`, `"Thermistor"` subobject with `"Pin"`, `"beta"`, `"r0"`, `"t0"`

Uses `Module(threadFreq, 1)` constructor so `slowUpdate()` fires once per second. The `Thermistor` class uses the simplified **beta equation**:

```
j = 1/beta
k = 1/(t0 + 273.15)
r = 4700 / (65536.0 / adcValue - 1)    ← NOTE: 65536, not 4096
T = 1/(k + j*ln(r/r0)) - 273.15        ← in Celsius
```

The formula uses `65536.0F` in the resistance calculation but the STM32 ADC is 12-bit (max 4095). This is a calibration error — the correct divisor is `4096.0F`. See limitations for full impact.

If temperature is ≤ 0, reports `999.0` (disconnected sensor heuristic).

### 7.9 Blink (Debug LED)

**Thread**: Servo (1 kHz) | **JSON keys**: `"Pin"`, `"Frequency"` (Hz)

`periodCount = threadFreq / frequency`. Toggles the pin every `periodCount / 2` ticks. This is also the default configuration loaded from hardcoded flash when no config file exists — a "heartbeat" to confirm the firmware is running.

### 7.10 ResetPin (Hardware Reset Input)

**Thread**: Servo (1 kHz) | **JSON keys**: `"Pin"`

Reads the pin state every servo tick and writes it to `Remora::reset`. When true, the state machine transitions to ST_SYSRESET, calling `HAL_NVIC_SystemReset()`. Provides a physical reset button for the MCU without power-cycling.

### 7.11 TMC Stepper Drivers (TMC2208, TMC2209, TMC5160)

**Thread**: "On load" (runs `configure()` once at startup)

All TMC modules use `std::enable_shared_from_this<TMC>` to obtain a `shared_ptr` to themselves for registration with the serial thread from within a virtual function call context.

**TMC2208 / TMC2209**: Use single-wire half-duplex UART via `SoftwareSerial`. The `configure()` sequence:
1. Starts the serial thread, registers self in it
2. Calls `driver->begin()` and `driver->test_connection()`
3. On failure: sets fatal status `TMC_DRIVER_ERROR`, stops serial thread
4. On success: configures TOFF(4), blank_time(24), rms_current, microsteps, TCOOLTHRS(0xFFFFF), CoolStep thresholds (`semin=5`, `semax=2`, `sedn=0b01`), SpreadCycle/StealthChop, pwm_autoscale, StallGuard threshold (TMC2209 only: `SGTHRS`), iholddelay(10), TPOWERDOWN(128)
5. Stops and unregisters from serial thread

After configuration, `update()` calls `driver->SWSerial->tickerHandler()` to service the software UART bit clock. The serial thread must remain running for ongoing UART communication.

**TMC5160**: Uses `SoftwareSPI` (bit-banged, separate CS/MOSI/MISO/SCK pins configured from JSON: `"pinCS"`, `"pinMOSI"`, `"pinMISO"`, `"pinSCK"`). Configuration follows a similar pattern to TMC2209.

---

## 8. remora-core Drivers

### 8.1 W5500 Networking (LwIP + WIZnet + TFTP)

The W5500 networking driver operates across three C++ namespaces:

**`namespace network`** — Top-level interface.

`EthernetInit()`: Configures WIZnet chip (reset, initialize, check version), initializes LwIP in `NO_SYS` mode, opens a MACRAW socket (socket 0, all 8 KB TX + 8 KB RX allocated to this one socket), adds the `netif`, starts UDP data server on port **27181** and TFTP server on port **69**.

`EthernetTasks()`: Polls MACRAW socket for received Ethernet frames, wraps in `pbuf`, inputs to LwIP via `netif.input()`. Calls `sys_check_timeouts()` for LwIP timer events.

`udp_data_callback()`: Called by LwIP when a UDP packet arrives on port 27181. Implements a **ring buffer detection mechanism** with `head`, `tail`, `dropped_packets` — but the actual ring buffer storage is commented out. Behavior: `memcpy` latest packet to `ptrRxData`, check header (`pruRead` or `pruWrite`), send immediate UDP response with `ptrTxData` contents, call `flag_new_data()`. Dropped packets are counted and printed when the ring buffer is "full."

**`namespace lwip`** — LwIP adaptation layer.

`netif_output()`: Transmits Ethernet frames through the W5500. Handles pbuf chains (concatenates segments into `tx_frame[1542]`), pads short frames to 60 bytes, computes FCS CRC32.

MAC address is hardcoded: `{0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}` (WIZnet OUI prefix). Static IP `10.10.10.10`, subnet `255.255.255.0`, gateway `10.10.10.1` — all from `Config` namespace constants.

**`namespace wiznet`** — W5500 SPI abstraction.

`wizchip_initialize()`: Registers SPI callbacks with the WIZnet driver library. The four callbacks (`SPI_read_byte`, `SPI_write_byte`, `SPI_DMA_read`, `SPI_DMA_write`) delegate to `CommsInterface*` methods. This is the glue between the WIZnet C library and the platform-specific SPI implementation.

`wizchip_check()`: Reads the version register, verifies it equals `0x04`. Hangs in `while(1)` if wrong — a hard startup assertion.

`wizchip_critical_section_lock()`: Uses busy-wait spin-lock (`while(spin_lock)`). Works on single-core MCU but prevents lower-priority ISRs during the spin.

**`namespace tftp`** — TFTP server (write-only, RFC 1350).

Listens on port 69. Only accepts WRQ (write request) opcodes. `IAP_tftp_process_write()`: On WRQ, erases JSON_UPLOAD sector, sets flash write address. `IAP_wrq_recv_callback()`: Receives 512-byte data blocks, writes each as 256 half-words to flash. After final block (less than 512 bytes), sets `JsonConfigHandler::new_flash_json = true`. Upload is complete when `Remora::run()` detects this flag and processes it.

### 8.2 SoftwareSPI Driver

Bit-banged SPI master used by `TMC5160`. Provides byte-level `transfer()`. Separate from the W5500 SPI bus, allowing the TMC5160 to have its own dedicated SPI pins.

### 8.3 SoftwareSerial Driver

Bit-banged UART used by `TMC2208`/`TMC2209` for half-duplex single-wire communication. The `tickerHandler()` method is called from the serial thread (TIM4 at `swBaudRate * oversample = 57600 Hz`) to advance the bit-timing state machine. This provides 3× oversampling for reliable reception at 19200 baud.

### 8.4 TMCStepper Library

A Mercurial-tracked fork of the TMCStepper Arduino library. Supports TMC2130, TMC2160, TMC2208, TMC2209, TMC5130, TMC5160 via strongly-typed register accessor classes. Notable registers: `GCONF`, `IHOLD_IRUN`, `TPOWERDOWN`, `CHOPCONF`, `COOLCONF`, `PWMCONF`, `DRV_STATUS`, `SGTHRS`, `TCOOLTHRS`. The library provides full configurability of current control, microstepping, CoolStep, StealthChop, and StallGuard features.

---

## 9. remora-hal HAL Abstraction Layer (STM32F4)

### 9.1 Startup & Main Entry Point

`main()` sequence:
1. Bootloader VTOR relocation (if `HAS_BOOTLOADER`)
2. `HAL_Init()` + `SystemClock_Config()` → 168 MHz SYSCLK (HSE 8 MHz, PLL M=4 N=168 P=2)
3. `init_board_status_led("PD_10")` — LED active immediately
4. `MX_UART_Init()` — printf retarget to USART2 or USART3 at 115200 baud
5. SPI builds only: `MX_SDIO_SD_Init()` + `MX_FATFS_Init()`
6. `HAL_Delay(2000)` — stabilization
7. Construct `STM32F4_EthComms` or `STM32F4_SPIComms` (compile-time selection via `#ifdef ETH_CTRL`)
8. Wrap in `CommsHandler`
9. Construct `STM32F4_timer` for TIM3 (base, 40 kHz) and TIM2 (servo, 1 kHz)
10. Construct `Remora(commsHandler, baseTimer, servoTimer, nullptr)`
11. `remora->run()` — never returns

The `printf` retarget overrides the weak `_write()` in `syscalls.c` with a strong function calling `HAL_UART_Transmit()`.

### 9.2 Pin Abstraction

`Pin` wraps STM32 HAL GPIO. Three constructors: simple mode+modifier, full alternate-function (gpio_mode, gpio_pull, gpio_speed, gpio_alt), and mode+modifier. Pin names are `"PA_0"` style strings parsed to extract port index (0=A to 7=H) and pin number. `enableClock()` activates the GPIO peripheral clock. `get()` and `set()` call `HAL_GPIO_ReadPin`/`HAL_GPIO_WritePin`.

`createPinFromPinMap()` uses mbed-style pin map tables to look up the alternate function number for SPI/PWM/encoder pins.

### 9.3 Timer Implementation (STM32F4_timer)

Inherits `pruTimer`. Maps TIM2 (servo, 1 kHz), TIM3 (base, 40 kHz), TIM4 (serial, 57.6 kHz — disabled).

Period calculation at 40 kHz: `ARR = (84,000,000 / 1 / 40000) - 1 = 2099`. At 1 kHz: `ARR = 83999`.

`configTimer()` powers on the timer clock, sets PSC=0, calculates ARR, enables update interrupts. `startTimer()` sets `TIM_CR1_CEN` and enables the NVIC IRQ. Timer ISRs in `irqHandlers.h` manually clear `TIM_SR_UIF` before dispatching — avoids `HAL_TIM_IRQHandler()` overhead.

### 9.4 SPI Communications — Slave Mode (STM32F4_SPIComms)

SPI slave to Raspberry Pi. The most architecturally complex component.

**Double-buffered circular DMA**: `HAL_DMAEx_MultiBufferStart_IT()` configures RX DMA in double-buffer mode between `rxDMABuffer.buffer[0]` and `buffer[1]`. TX DMA is circular single-buffer (always transmitting `txData`).

**Header check on half-complete**: `CheckHeader()` is called from DMA half-complete callbacks (at 32 of 64 bytes received). Since the header is the first 4 bytes, it is valid at the halfway point. `PRU_WRITE` sets `newWriteData = true` and saves `RXbufferIdx`.

**NSS interrupt (EXTI4)**: Rising edge on CS signals end of SPI transaction. If `newWriteData` is true, sets `copyRXbuffer = true`.

**Memory-to-memory DMA copy** (`tasks()`): When `copyRXbuffer` is true, a mem-to-mem DMA transfer (DMA2_Stream1) copies the received buffer to `ptrRxData` with IRQs disabled. `HAL_DMA_PollForTransfer()` makes this synchronous, ensuring `rxData` is updated atomically.

### 9.5 Ethernet Communications — W5500 Master (STM32F4_EthComms)

SPI master to W5500. CPOL=0, CPHA=0, BRP=2 (42 MHz SPI clock). DMA configured for both TX and RX (non-circular, normal mode). Byte-level `read_byte()`/`write_byte()` use direct register access for minimal latency. Bulk `DMA_write()`/`DMA_read()` use HAL DMA with blocking spin-wait.

`flag_new_data()` (called from the LwIP UDP callback) sets `newDataFlagged = true`. On the next `tasks()` call, the header is inspected and `dataCallback(true/false)` is invoked.

### 9.6 Hardware PWM Driver

`HardwarePWM` provides timer-based PWM on any available STM32 timer channel. Identifies timer instance and channel from `PinMap_PWM` via `STM_PIN_CHAN_SHIFT`/`STM_PIN_CHAN_MASK`. Detects complementary (TIMx_CHyN) channels and uses `HAL_TIMEx_PWMN_Start()`. Prescaler = `(timer_clk_hz / 1,000,000) - 1` giving 1 µs per tick.

**Shared-timer linked list**: Static `head` + instance `prev`. `change_period()` and `change_pulsewidth()` iterate all instances sharing the same TIMx to keep all channels in sync when the period changes. Guards against TIM2/TIM3 with a printf warning.

### 9.7 Hardware QEI Driver

`Hardware_QEI` uses TIM8 in encoder mode. Fixed pins: PC_6 (ChA), PC_7 (ChB), PA_8 (index). `TIM_ENCODERMODE_TI12` (4× decode). 16-bit counter. `get()` returns `counter >> 2` (divides by 4). Input capture filter = 10 clock cycles. Index interrupt on EXTI9_5 (EXTI8 specifically).

### 9.8 Analog Input (ADC)

`AnalogIn` uses shared ADC handles. Singleton guard: only initializes the ADC if `ptr_adc_handle->Instance == 0`. 12-bit, right-aligned, software-triggered, polling-based, 3-cycle sampling. `HAL_ADC_ConfigChannel()` is called on every read (necessary for channel mux re-selection on a shared ADC). Blocking read with 10 ms timeout.

### 9.9 Shared Peripheral Handle Management

Pre-allocated global `TIM_HandleTypeDef` for TIM1, TIM4, TIM5, TIM8-TIM14 (TIM2/TIM3 excluded — reserved for Remora threads). Pre-allocated `ADC_HandleTypeDef` for ADC1, ADC2, ADC3. `get_timer_clk_freq(TIMx)` correctly handles the APB prescaler multiplier (×2 when APBx divider > 1).

### 9.10 Board Status LED & Error Codes

LED blink-code system on a configurable GPIO. Five patterns of 6 pulses (FAST=250ms, SLOW=1000ms) with 125ms gaps, repeating with 1500ms pause. Covers: CRITICAL_HAL_ERROR (6×fast), SD_CARD_HW_ERROR, SD_CARD_MOUNT_ERROR, SD_CARD_FILE_ERROR, SPI_PERIPH_ERROR. 64 unique codes possible (2^6).

### 9.11 Platform Configuration & Flash Layout

Linker script symbols exported to C++ via `extern "C"` pointer declarations. The `Platform_Config` namespace exposes typed `uintptr_t` constants. Sector numbers are stored as linker-defined addresses and retrieved as pointer-cast integers — a clever trick for injecting numeric values through the linker.

---

## 10. LinuxCNC Host-Side Components

### remora-eth-3.0.c (Ethernet)

A LinuxCNC RTAPI/HAL component communicating over UDP (port 27181). HAL pins exposed for up to 8 joints, 16 digital outputs, 32×2 digital input slots, 6 setpoints and process variables, plus enable/reset/status. The stepgen algorithm uses the same DDS approach (`STEPBIT = 22`, `PRU_BASEFREQ = 40000`). Step frequency command: `freq = (vel_cmd * pos_scale) / PRU_BASEFREQ * (1 << STEPBIT)`. Position feedback: `pos_fb = count / pos_scale`.

### remora-spi.c (SPI via RPi)

Uses BCM2835 (RPi 3/4) or RP1 (RPi 5) SPI driver libraries. Supports multiple boards via `dtcboards.h`.

### HAL Configuration (remora-NucleoHat.hal)

Wires Remora pins to LinuxCNC motion joints 0-2. Configures per-joint scale, acceleration, P-gain, FF1-gain, deadband. Sets up analog potentiometer-based feed and spindle speed overrides reading from `remora.processVariable.0`/`.1`.

---

## 11. Communication Protocol

### Packet Layout (64 bytes, packed)

**RX (MCU receives):**
```
[0-3]   int32_t  header         "read" or "writ"
[4-35]  int32_t  jointFreqCmd[8] step frequency commands in DDS units
[36-59] float    setPoint[6]    analog commands (PWM duty/period, PID SP)
[60]    uint8_t  jointEnable    joint enable bitmask (bit N = joint N)
[61-62] uint16_t outputs        digital output bitmask (16 bits)
[63]    uint8_t  spare0
```

**TX (MCU sends):**
```
[0-3]   int32_t  header         0x64617461 | remoraStatus ("data" + status)
[4-35]  int32_t  jointFeedback[8] step counts (position feedback)
[36-59] float    processVariable[6] analog feedback (temp, encoder, ADC)
[60-61] uint16_t inputs         digital input bitmask (16 bits)
[62-63] padding
```

### Protocol Headers (human-readable ASCII)

| Header | Hex | Direction | Meaning |
|---|---|---|---|
| "read" | `0x72656164` | Host→MCU | Read request only |
| "writ" | `0x77726974` | Host→MCU | Write commands + read |
| "data" | `0x64617461` | MCU→Host | Normal response (ETH) |
| "dat\0" | `0x64617400` | MCU→Host | Normal response (SPI) |
| "ackn" | `0x61636b6e` | MCU→Host | Write acknowledged |
| "erro" | `0x6572726f` | MCU→Host | Error state |
| "estp" | `0x65737470` | Both | Emergency stop |

The difference between `"data"` and `"dat\0"`: ETH builds use the full 4-byte ASCII; SPI builds use 3-byte ASCII + null. The SPI `remora-spi` component checks only `HEADER_MASK = 0xFFFFFF00` for backward compatibility.

---

## 12. Flash Memory & JSON Configuration System

### Configuration Upload Flow (ETH builds)

```
python3 upload_config.py config.txt
  → tftpy client sends WRQ to MCU port 69
    → TFTP server:
      1. Receives WRQ → erases JSON_UPLOAD sector
      2. Receives 512-byte DATA blocks → writes to flash as halfwords
      3. Final partial block → JsonConfigHandler::new_flash_json = true

Next main loop iteration:
  → json_check_length_and_CRC()
    → reads json_metadata_t from JSON_UPLOAD
    → computes CRC32 (padded to 4-byte boundary)
    → if CRC OK: store_json_in_flash()
      → erases JSON_STORAGE sector
      → writes jsonLength (4 bytes) then JSON content
    → pru_reboot() → HAL_NVIC_SystemReset()

After reboot:
  → readConfigFromFlash()
    → reads jsonLength from JSON_STORAGE[0:3]
    → reads JSON bytes byte-by-byte
    → ArduinoJson parse → loadModules()
```

### Configuration Upload Flow (SPI builds)

SD card with `config.txt` at root → FatFs mount on boot → read entire file → ArduinoJson parse. No CRC verification.

### JSON Configuration Format

```json
{
  "Board": "NUCLEOF446xx Hat",
  "Threads": [
    {"Thread": "Base", "Frequency": 40000},
    {"Thread": "Servo", "Frequency": 1000}
  ],
  "Modules": [
    {
      "Thread": "Base",
      "Type": "Stepgen",
      "Comment": "X axis",
      "Joint Number": 0,
      "Enable Pin": "PE_9",
      "Step Pin": "PG_4",
      "Direction Pin": "PG_7"
    },
    {
      "Thread": "Servo",
      "Type": "PWM",
      "Comment": "Spindle PWM",
      "PWM Pin": "PA_1",
      "SP[i]": 0,
      "Period SP[i]": 1,
      "Period us": 100,
      "Variable Freq": "True",
      "PWM Max": 256,
      "Hardware PWM": "True"
    },
    {
      "Thread": "On load",
      "Type": "TMC2209",
      "Comment": "X driver",
      "RX pin": "PD_2",
      "RSense": 0.11,
      "Address": 0,
      "Current": 800,
      "Microsteps": 16,
      "Stealth chop": "on",
      "Stall sensitivity": 100
    }
  ]
}
```

---

## 13. Real-Time Threading Model

Three interrupt-driven threads, each owning a hardware timer:

| Thread | Timer | Frequency | Priority | Purpose |
|---|---|---|---|---|
| Base | TIM3 | 40 kHz | 1 (highest) | Step pulse generation, quadrature decoding |
| Servo | TIM2 | 1 kHz | 2 | Position feedback, comms, analog/digital I/O, PWM |
| Serial | TIM4 | 57.6 kHz | 3 | TMC UART bit-clock (disabled if no TMC) |

`CommsHandler` runs only in the servo thread, monitoring packet flow and providing the `status` signal driving the Remora state machine. The main loop runs at background priority, handling `comms->tasks()` and ETH JSON upload checks. No OS or RTOS — pure bare-metal interrupt-driven execution.

---

## 14. Complete Data Flow: End-to-End

### SPI Path

```
LinuxCNC servo thread (1 kHz)
  → RPi writes 64-byte packet via hardware SPI
    → STM32 SPI slave DMA receives into rxDMABuffer[N]
      → DMA half-complete: CheckHeader()
        → PRU_WRITE: newWriteData=true, save RXbufferIdx
      → NSS rising edge (EXTI4): set copyRXbuffer=true
      → DMA TX: continuously clocks out txData to RPi
    → Servo thread tasks(): DMA mem-to-mem copy rxDMABuffer to rxData
    → CommsHandler sees data flag → status=true
    → State: ST_IDLE → ST_RUNNING

Base thread (25 µs, 40 kHz):
  → Stepgen::update() [for each joint]:
    DDSaccumulator += jointFreqCmd[N] * frequencyScale
    if bit 22 changed: raise stepPin, update jointFeedback
  → SoftEncoder::update() [if configured]:
    state machine decode → processVariable[N]
  → Stepgen::updatePost(): lower all stepPins (25 µs pulse ends)

Servo thread (1 ms, 1 kHz):
  → DigitalPin: rxData.outputs → GPIO pins, GPIO pins → txData.inputs
  → AnalogPin: ADC read → txData.processVariable[N]
  → PWM: rxData.setPoint changes → HardwarePWM period/duty
  → QEI: TIM8 counter → txData.processVariable[N]
  → Temperature (1 Hz): thermistor ADC → txData.processVariable[N]
  → txData.header = Config::pruData | remoraStatus
  → CommsHandler::update(): packet watchdog

Main loop (background):
  → Remora::run(): state machine
  → comms->tasks(): no-op for SPI path
```

### Ethernet Path

```
LinuxCNC servo thread (1 kHz)
  → UDP packet sent to 10.10.10.10:27181

Main loop (background):
  → comms->tasks() → network::EthernetTasks()
    → poll MACRAW socket → recv_lwip() → pbuf → netif.input()
    → LwIP: UDP demux → udp_data_callback()
      → memcpy payload to rxData
      → check header → build txData response
      → send UDP response to LinuxCNC
      → flag_new_data()
    → EthComms::tasks(): newDataFlagged → dataCallback(true)
    → CommsHandler sees data flag

[All RT thread behavior identical to SPI path]

ETH JSON upload (background, when new config detected):
  → json_check_length_and_CRC() → store_json_in_flash()
  → pru_reboot()
```

---

## 15. Design Patterns & Engineering Decisions

**Smart-pointer ownership graph**: `Remora` uses `unique_ptr` for exclusive ownership (threads, timers, config handler) and `shared_ptr` for shared ownership (modules registered in multiple lists, CommsHandler). Ownership semantics are explicit and correct throughout.

**Factory pattern with static factory methods**: Each module class has a `static create(JsonObject, Remora*)` method called by `ModuleFactory`. This cleanly separates module instantiation (which needs JSON data and Remora pointers) from object construction. The tradeoff is that the factory requires hardcoded if-else chains.

**Singleton factory**: `ModuleFactory::getInstance()` returns a heap-allocated static instance. Never deallocated (leak-by-design for embedded singletons). Thread-safety is not a concern since modules are only created during ST_SETUP before threads start.

**`enable_shared_from_this` for TMC modules**: TMC drivers need to register themselves with the serial thread during `configure()`, but `configure()` is invoked via virtual dispatch with a raw `this` pointer. `enable_shared_from_this` provides the necessary `shared_ptr` without creating a duplicate ownership count.

**Post-module vector for pulse width control**: `modulesPost` / `runModulePost()` is a general-purpose mechanism enabling a "second pass" over modules. Currently only used by Stepgen to guarantee exactly one thread period of step pulse width.

**`std::atomic<bool>` for thread state**: `pruThread::threadRunning` and `threadPaused` use explicit `memory_order_release`/`memory_order_acquire`. Technically correct for ARM Cortex-M (weakly-ordered memory model); conservative but safe.

**Human-readable protocol magic numbers**: All communication headers are valid ASCII strings. Logic analyzer traces are immediately interpretable without a lookup table.

**Linker script symbol injection**: Flash sector addresses/numbers are injected from linker scripts via `extern "C"` pointer declarations. Different boards just use different linker scripts with no source changes.

**Default blink config in ROM**: The hardcoded `Config::defaultConfig` ensures safe fallback behavior (4 Hz LED blink) when flash is blank. Non-fatal `CONFIG_LOADED_DEFAULT` status reports this condition to LinuxCNC.

**W5500 MACRAW socket with LwIP**: The W5500 is used in raw Ethernet mode (MACRAW), with LwIP handling all IP/UDP protocol processing. This gives full TCP/IP stack capabilities through the WIZnet chip while maintaining portability of the network application code.

**CRC32 on TFTP upload**: The 512-byte metadata block (padded to exactly one TFTP packet) at the start of the upload area stores CRC32, length, and JSON length. The CRC is verified before committing to storage flash — preventing corrupt configs from being applied and requiring a reboot to recover.

---

## 16. Known Limitations, Caveats & TODOs

**Thread frequency setter broken at runtime**: `setBaseFreq()`/`setServoFreq()` update the frequency stored in `Remora` (used for module DDS scale calculations) but the lines that would update the actual timer are commented out with a note "could use a refactor." JSON `"Threads"` frequency overrides work only partially.

**Thermistor ADC 16-bit vs 12-bit mismatch**: The thermistor resistance formula uses `65536.0F / adcValue` (designed for a 16-bit ADC) but the STM32 ADC is 12-bit (max 4095). The correct divisor is `4096.0F`. At the midpoint (ADC=2047), this computes `r = 4700 / (32.02 - 1) = 151Ω` instead of the correct `r = 4700 / (2.0 - 1) = 4700Ω`. Temperature readings will be systematically wrong by a large margin.

**SD card config: O(n²) string construction**: `readConfigFromSD()` builds `jsonContent` with `jsonContent = jsonContent + rtext[i]` in a character loop. This is O(n²) in memory copies. For a 10 KB config file: ~50 million byte copies. Should use `jsonContent.assign(rtext, length)`.

**SD card config: VLA on stack**: `char rtext[length]` is a C99 VLA allocated on the stack. Large config files can overflow the 1 KB minimum stack. Should use heap allocation or a fixed maximum-size buffer.

**Timer collision warning is advisory**: `HardwarePWM` prints a warning if TIM2/TIM3 are selected but allows configuration to proceed, which would interfere with Remora threads. Should be a hard error or assertion.

**QEI pins are hardcoded**: `Hardware_QEI` has `chAPortAndPin = "PC_6"`, `chBPortAndPin = "PC_7"`, `indexPortAndPin = "PA_8"` as hardcoded member initializers. Not JSON-configurable. Single QEI instance per system.

**ADC reads are blocking and slow**: `AnalogIn::read()` uses `HAL_ADC_PollForConversion()` with 10 ms timeout. Multiple analog channels create serial latency in the servo thread.

**W5500 MAC address is hardcoded**: `{0x00, 0x08, 0xDC, 0x12, 0x34, 0x56}`. ARP conflicts if multiple boards exist on the same network.

**W5500 IP is hardcoded**: Static IP `10.10.10.10` with no DHCP support.

**W5500 ring buffer is vestigial**: Ring buffer detection code exists but actual ring buffer storage is commented out. Lost packets are counted but not recovered.

**W5500 spin-lock is not ISR-safe**: `while(spin_lock)` polling in `wizchip_critical_section_lock`. Could starve lower-priority interrupts during W5500 SPI transactions.

**`PULSE_DIVIDER = 2` naming is misleading**: Used as a right-shift amount, dividing by 4, not 2. The comment says "divide by 4." Should be `PULSE_DIVIDER_SHIFT` or the value should be 4.

**Software PWM not implemented**: `PWM::create()` checks `if (!strcmp(hardware, "False"))` and prints "Software PWM not yet supported" with no fallback.

**No direction setup time in Stepgen**: Direction pin is set in the same cycle as the step pulse. Some stepper drivers require 100ns–1µs setup. This could be violated at 168 MHz GPIO toggle rates during direction reversals.

**Serial thread disabled in STM32 HAL build**: TIM4 is passed as `nullptr` to `Remora`. `TMC2208`/`TMC2209` `configure()` calls `instance->getSerialThread()->startThread()` — this would dereference `nullptr` if TMC modules are used without a serial thread. The STM32 build currently cannot use TMC UART drivers.

**`ModuleFactory` uses leaked singleton**: `static ModuleFactory* instance = new ModuleFactory()` is never freed. Acceptable for embedded but worth noting.

**`onLoad` thread name case sensitivity**: The factory checks `strcmp(_tname, "On load")` (capital O, lowercase l). Any other capitalization would fail silently with "Unknown thread type" error.

**CRC table allocated on stack**: `crc32::generate_table(table[256])` requires 1024 bytes of stack space every time it is called during config upload verification.

---

## 17. Summary Findings Table

| Aspect | Detail |
|---|---|
| **Firmware version** | 2.0.0 |
| **Target MCU** | STM32F446RE/ZE @ 168 MHz |
| **Framework** | STM32Cube HAL via PlatformIO |
| **Core architecture** | Platform-agnostic engine (remora-core) + STM32F4 HAL port (remora-hal) |
| **Comms modes** | SPI slave (to RPi) or Ethernet UDP (W5500, hardcoded 10.10.10.10:27181) |
| **Config upload** | TFTP over Ethernet or SD card → flash → CRC verify → store → reboot |
| **RT thread freq** | Base 40 kHz, Servo 1 kHz, Serial 57.6 kHz (currently disabled in HAL build) |
| **Max step rate** | 40 kHz per axis |
| **Max joints** | 8 |
| **Digital outputs** | 16 (uint16_t bitmask) |
| **Digital inputs** | 16 (firmware) / 32 (LinuxCNC ETH component) — mismatch |
| **Analog variables** | 6 setpoints + 6 process variables (float) |
| **Module types** | Stepgen, SoftEncoder, DigitalPin, AnalogPin, PWM, QEI, SigmaDelta, Temperature, Blink, ResetPin, TMC2208, TMC2209, TMC5160 |
| **PWM channels** | TIM1, TIM4, TIM5, TIM8-14 (TIM2/3 reserved for threads) |
| **QEI channels** | 1 hardware (TIM8, fixed pins), unlimited software (Base thread CPU budget) |
| **ADC inputs** | 3 ADC peripherals, multiple channels, polling only |
| **TMC drivers** | 2208/2209 (UART half-duplex), 5160 (SPI) — serial thread disabled in current HAL build |
| **Error system** | 8-bit structured status byte + LED blink codes (64 unique codes) |
| **Key design patterns** | State machine, Singleton factory, DDS stepgen, ModuleInterrupt<T> template, enable_shared_from_this, post-module vector |
| **Config format** | JSON (ArduinoJson v7) in STM32 internal flash |
| **Config integrity** | CRC32 (IEEE 802.3, 256-entry lookup table) |
| **Networking stack** | LwIP NO_SYS over WIZnet W5500 MACRAW socket, TFTP write server |
| **Hardcoded items** | IP address, MAC address, QEI pins, flash sector sizes, thermistor divider resistor |
| **Notable bugs** | Thermistor 16-bit/12-bit ADC mismatch; O(n²) SD config string build; thread freq setter broken at runtime |
| **Board support** | Nucleo F446RE, Nucleo F446ZE, Octopus 1.1 (with bootloader) |
| **LinuxCNC integration** | HAL components for ETH and SPI; example .hal/.ini configs provided |

---

*Analysis based on complete static reading of both `Remora-STM32F4xx-PIO-main.zip` and `remora-core-8d3b6090.zip`. All findings are the result of source code analysis.*
