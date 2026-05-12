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
set(PICO_BOARD waveshare_rp2350_plus_4mb CACHE STRING "Board type")

cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(remora_rp2350 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Enable C++ exception handling globally.
# Must be set before pico_sdk_init() — the sdk otherwise forces -fno-exceptions
# on all targets through its interface compile options.
# The RP2350's 4 MB flash makes the binary size overhead acceptable.
set(PICO_CXX_ENABLE_EXCEPTIONS 1)

pico_sdk_init()

# ---- W5500 chip variant ----
# Common clone W5500 silicon (non-WIZnet third-party) differs from the genuine
# WIZnet W5500 in two ways that require build-time workarounds:
#   1. VERSIONR register returns 0x03 instead of 0x04.
#   2. Issuing a software PHY reset (ctlwizchip CW_RESET_PHY) immediately before
#      the MACRAW socket OPEN command causes the socket to latch at an undefined
#      status (0x0c) instead of SOCK_MACRAW (0x42), silently disabling all frame
#      reception.  The hardware RST-pin reset in wizchip_initialize() already
#      handles PHY init, so the software reset is simply skipped on clone chips.
#
# Set W5500_CLONE_CHIP=OFF on the cmake command line (or in CMakePresets.json)
# when building for a board fitted with genuine WIZnet W5500 silicon.
option(W5500_CLONE_CHIP
    "Target a W5500 clone chip (3rd-party silicon: VERSIONR=0x03, breaks on SW PHY reset before MACRAW open). ON=clone, OFF=genuine WIZnet."
    ON)

if(W5500_CLONE_CHIP)
    message(STATUS "W5500 chip variant: clone (VERSIONR normalisation + SW PHY reset suppressed)")
else()
    message(STATUS "W5500 chip variant: genuine WIZnet W5500")
endif()

# ---- no-OS-FatFS-SD-SPI-RPi-Pico (SPI build only — SD card config loader) ----
# Submodule must be present at lib/no-OS-FatFS-SD-SPI-RPi-Pico.
# To add it: git submodule add https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git lib/no-OS-FatFS-SD-SPI-RPi-Pico
#            git submodule update --init
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI/CMakeLists.txt)
    # hardware_rtc is RP2040-only and does not exist in the RP2350 pico-sdk.
    # Define an empty stub target so the FatFs_SPI CMakeLists.txt link dependency
    # resolves without error.  Our hardware/rtc.h stub handles the compile side.
    if(NOT TARGET hardware_rtc)
        add_library(hardware_rtc INTERFACE)
    endif()
    add_subdirectory(lib/no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI FatFs_SPI_build)
    set(FATFS_SPI_AVAILABLE TRUE)
    message(STATUS "no-OS-FatFS-SD-SPI-RPi-Pico found — SPI build will use real FatFs")
    # The RP2350 has no dedicated RTC peripheral so hardware/rtc.h does not exist
    # in the pico-sdk.  Prepend Src/ so our stub hardware/rtc.h is found first,
    # satisfying the include in the library's rtc.c.
    target_include_directories(FatFs_SPI INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/Src)
else()
    set(FATFS_SPI_AVAILABLE FALSE)
    message(WARNING
        "no-OS-FatFS-SD-SPI-RPi-Pico submodule not found. "
        "SPI build will fall back to the hardcoded default config (no SD card). "
        "To enable real FatFs, run:\n"
        "  git submodule add https://github.com/carlk3/no-OS-FatFS-SD-SPI-RPi-Pico.git lib/no-OS-FatFS-SD-SPI-RPi-Pico\n"
        "  git submodule update --init\n"
        "then re-run cmake.")
endif()

# ---- ArduinoJson (header-only, used by remora-core) ----
# TODO: ArduinoJson should be provided outside of CMake (e.g. as a git submodule
# or a manually downloaded copy placed under lib/ArduinoJson), rather than
# fetched at configure time via FetchContent.  Update this section accordingly
# once the preferred delivery method is decided.
include(FetchContent)
FetchContent_Declare(
    ArduinoJson
    GIT_REPOSITORY https://github.com/bblanchon/ArduinoJson.git
    GIT_TAG        v7.4.1
)
FetchContent_MakeAvailable(ArduinoJson)

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
    Src/remora-core/drivers/TMCStepper/TMC2130Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC2160Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC2208Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC2209Stepper.cpp
    Src/remora-core/drivers/TMCStepper/TMC5130Stepper.cpp
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
    Src/remora-hal/sd_globals.cpp   # FatFs workspace objects (SDFatFS, SDFile, SDPath)
    Src/remora-hal/hw_config.c      # SD card SPI1 hardware config (GP10-GP13)
)
target_include_directories(remora_rp2350_spi PRIVATE ${REMORA_INCLUDE_DIRS})
target_compile_options(remora_rp2350_spi PRIVATE -fexceptions)
target_compile_definitions(remora_rp2350_spi PRIVATE
    SPI_CTRL=1
    # SPI0 slave pins (from RPi master perspective):
    #   MOSI = GP4 (SPI0_RX — slave receives from master)
    #   MISO = GP3 (SPI0_TX — slave transmits to master)
    SPI_MOSI_GPIO=4
    SPI_MISO_GPIO=3
    SPI_CLK_GPIO=2
    SPI_CS_GPIO=5
    # NO_FATFS_SPI is defined when the submodule is absent — fatfs.h falls back to stub
    $<$<NOT:$<BOOL:${FATFS_SPI_AVAILABLE}>>:NO_FATFS_SPI=1>
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
    ArduinoJson
    $<$<BOOL:${FATFS_SPI_AVAILABLE}>:FatFs_SPI>
)
pico_generate_pio_header(remora_rp2350_spi
    ${CMAKE_CURRENT_LIST_DIR}/Src/remora-hal/spi_slave.pio)
pico_generate_pio_header(remora_rp2350_spi
    ${CMAKE_CURRENT_LIST_DIR}/Src/remora-hal/hardware_qei/quadrature_encoder.pio)
target_compile_definitions(remora_rp2350_spi PRIVATE HAS_QUADRATURE_ENCODER_PIO_H=1)
pico_enable_stdio_usb(remora_rp2350_spi 1)
pico_enable_stdio_uart(remora_rp2350_spi 0)
pico_add_extra_outputs(remora_rp2350_spi)  # generates .uf2, .bin, .hex

# ---- SPI builtin-config build target ----
# SPI comms to the Raspberry Pi host, but config is the hardcoded default
# (no SD card or FatFs required).  Useful for verifying SPI connectivity
# before the SD card is wired up.  Reaches ST_RUNNING as soon as the RPi
# sends the first valid packet; jointFreqCmd values come from the host.
add_executable(remora_rp2350_spi_builtin
    ${REMORA_CORE_SOURCES}
    ${REMORA_HAL_SOURCES}
    Src/remora-hal/RP2350_SPIComms.cpp
)
target_include_directories(remora_rp2350_spi_builtin PRIVATE ${REMORA_INCLUDE_DIRS})
target_compile_options(remora_rp2350_spi_builtin PRIVATE -fexceptions)
target_compile_definitions(remora_rp2350_spi_builtin PRIVATE
    SPI_CTRL=1
    NO_FATFS_SPI=1      # skip SD card — use hardcoded default config
    # SPI0 slave pins (from RPi master perspective):
    #   MOSI = GP4 (SPI0_RX — slave receives from master)
    #   MISO = GP3 (SPI0_TX — slave transmits to master)
    SPI_MOSI_GPIO=4
    SPI_MISO_GPIO=3
    SPI_CLK_GPIO=2
    SPI_CS_GPIO=5
)
target_link_libraries(remora_rp2350_spi_builtin
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
    ArduinoJson
)
pico_generate_pio_header(remora_rp2350_spi_builtin
    ${CMAKE_CURRENT_LIST_DIR}/Src/remora-hal/spi_slave.pio)
pico_generate_pio_header(remora_rp2350_spi_builtin
    ${CMAKE_CURRENT_LIST_DIR}/Src/remora-hal/hardware_qei/quadrature_encoder.pio)
target_compile_definitions(remora_rp2350_spi_builtin PRIVATE HAS_QUADRATURE_ENCODER_PIO_H=1)
pico_enable_stdio_usb(remora_rp2350_spi_builtin 1)
pico_enable_stdio_uart(remora_rp2350_spi_builtin 0)
pico_add_extra_outputs(remora_rp2350_spi_builtin)

# ---- W5500 / LwIP sources (ETH build only) ----
set(LWIP_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/W5500_Networking/lwip)
set(W5500_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/lib/W5500_Networking)

set(LWIP_SOURCES
    # LwIP core (minimum set from Filelists.cmake lwipcore_SRCS)
    ${LWIP_DIR}/src/core/init.c
    ${LWIP_DIR}/src/core/def.c
    ${LWIP_DIR}/src/core/dns.c
    ${LWIP_DIR}/src/core/inet_chksum.c
    ${LWIP_DIR}/src/core/ip.c
    ${LWIP_DIR}/src/core/mem.c
    ${LWIP_DIR}/src/core/memp.c
    ${LWIP_DIR}/src/core/netif.c
    ${LWIP_DIR}/src/core/pbuf.c
    ${LWIP_DIR}/src/core/raw.c
    ${LWIP_DIR}/src/core/stats.c
    ${LWIP_DIR}/src/core/sys.c
    ${LWIP_DIR}/src/core/altcp.c
    ${LWIP_DIR}/src/core/altcp_alloc.c
    ${LWIP_DIR}/src/core/altcp_tcp.c
    ${LWIP_DIR}/src/core/tcp.c
    ${LWIP_DIR}/src/core/tcp_in.c
    ${LWIP_DIR}/src/core/tcp_out.c
    ${LWIP_DIR}/src/core/timeouts.c
    ${LWIP_DIR}/src/core/udp.c
    # LwIP IPv4 (lwipcore4_SRCS)
    ${LWIP_DIR}/src/core/ipv4/autoip.c
    ${LWIP_DIR}/src/core/ipv4/dhcp.c
    ${LWIP_DIR}/src/core/ipv4/etharp.c
    ${LWIP_DIR}/src/core/ipv4/icmp.c
    ${LWIP_DIR}/src/core/ipv4/igmp.c
    ${LWIP_DIR}/src/core/ipv4/ip4_frag.c
    ${LWIP_DIR}/src/core/ipv4/ip4.c
    ${LWIP_DIR}/src/core/ipv4/ip4_addr.c
    # LwIP netif — ethernet.c needed for MACRAW / ethernet_input
    ${LWIP_DIR}/src/netif/ethernet.c
    # WIZnet ioLibrary driver
    ${W5500_LIB_DIR}/ioLibrary_Driver/Ethernet/socket.c
    ${W5500_LIB_DIR}/ioLibrary_Driver/Ethernet/wizchip_conf.c
    ${W5500_LIB_DIR}/ioLibrary_Driver/Ethernet/W5500/w5500.c
    # Platform arch: sys_now() + Thumb-2 memcpy/checksum (required by arch/cc.h)
    ${W5500_LIB_DIR}/arch/sys_arch.c
    ${W5500_LIB_DIR}/arch/checksum.c
    ${W5500_LIB_DIR}/arch/memcpy.c
    # Remora W5500 networking glue
    Src/remora-core/drivers/W5500_Networking/W5500_Networking.cpp
)

set(W5500_INCLUDE_DIRS
    lib/W5500_Networking/lwip/src/include           # lwip/*.h
    lib/W5500_Networking/ioLibrary_Driver/Ethernet  # socket.h, wizchip_conf.h, W5500/w5500.h
    lib/W5500_Networking                            # arch/cc.h, lwipopts.h
)

# ---- ETH build target ----
add_executable(remora_rp2350_eth
    ${REMORA_CORE_SOURCES}
    ${REMORA_HAL_SOURCES}
    Src/remora-hal/RP2350_EthComms.cpp
    ${LWIP_SOURCES}
)
target_include_directories(remora_rp2350_eth PRIVATE
    ${REMORA_INCLUDE_DIRS}
    ${W5500_INCLUDE_DIRS}
)
target_compile_options(remora_rp2350_eth PRIVATE -fexceptions)
target_compile_definitions(remora_rp2350_eth PRIVATE
    ETH_CTRL=1
    _WIZCHIP_=5500
    # W5500 wired to the same GPIO header as the SPI build's Pi comms connector.
    # RP2350 is SPI0 master here; pin roles are master-perspective (TX=MOSI, RX=MISO).
    SPI_CLK_GPIO=2
    SPI_MOSI_GPIO=3    # SPI0_TX — master transmits to W5500 MOSI
    SPI_MISO_GPIO=4    # SPI0_RX — master receives from W5500 MISO
    SPI_CS_GPIO=5
    WIZ_RST_GPIO=6     # W5500 reset — any free GPIO; change if GP6 is needed elsewhere
    # Clone chip workarounds — see option() block near the top of this file.
    # Evaluates to W5500_CLONE_CHIP when the option is ON, nothing when OFF.
    $<$<BOOL:${W5500_CLONE_CHIP}>:W5500_CLONE_CHIP>
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
    ArduinoJson
)
pico_generate_pio_header(remora_rp2350_eth
    ${CMAKE_CURRENT_LIST_DIR}/Src/remora-hal/hardware_qei/quadrature_encoder.pio)
target_compile_definitions(remora_rp2350_eth PRIVATE HAS_QUADRATURE_ENCODER_PIO_H=1)
pico_enable_stdio_usb(remora_rp2350_eth 1)
pico_enable_stdio_uart(remora_rp2350_eth 0)
pico_add_extra_outputs(remora_rp2350_eth)
