# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD waveshare_rp2040_plus_4mb CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(remora_rp2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# ---- remora-core sources (platform-agnostic, never modified) ----
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

# ---- Common HAL sources (shared by both SPI and ETH targets) ----
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
    # Default Pico 2 SPI0 slave pins — override with -D flags as needed
    SPI_MOSI_GPIO=3
    SPI_MISO_GPIO=4
    SPI_CLK_GPIO=2
    SPI_CS_GPIO=5
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
)
pico_enable_stdio_usb(remora_rp2350_spi 1)
pico_enable_stdio_uart(remora_rp2350_spi 0)
pico_add_extra_outputs(remora_rp2350_spi)  # generates .uf2, .bin, .hex

# ---- ETH build target ----
add_executable(remora_rp2350_eth
    ${REMORA_CORE_SOURCES}
    ${REMORA_HAL_SOURCES}
    Src/remora-hal/RP2350_EthComms.cpp
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
