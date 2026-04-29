cmake_minimum_required(VERSION 3.13)

# Pull in Pico SDK (must be before project())
include(pico_sdk_import.cmake)

project(cd32_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialise the Pico SDK
pico_sdk_init()

# Gather all source files
set(SOURCES
    main.c
    core/player.c
    core/dispatcher.c
    core/cmd_hndl.c
    core/commo.c
    core/sts_q_id.c
    core/service.c
    core/play.c
    drivers/driver.c
    drivers/servo.c
    drivers/subcode.c
    drivers/strtstop.c
    drivers/shock.c
    utils/timer.c
    utils/maths.c
)


add_executable(cd32_pico ${SOURCES})

# Include directories
target_include_directories(cd32_pico PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link Pico SDK libraries
target_link_libraries(cd32_pico
    pico_stdlib
    hardware_gpio
    hardware_timer
    hardware_irq
    hardware_pio
    pico_multicore
)

# Enable USB serial output and disable UART (swap if needed)
pico_enable_stdio_usb(cd32_pico 1)
pico_enable_stdio_uart(cd32_pico 0)

# Generate UF2, bin, hex etc.
pico_add_extra_outputs(cd32_pico)
