cmake_minimum_required(VERSION 3.13)

# Raspberry Pi Pico 2 (RP2350). Must be set before the SDK is initialized.
set(PICO_BOARD pico2 CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(avionics C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(avionics
    src/main.c
    src/imu.c
    src/ms5611.c
    src/servos.c
    src/flashlog.c
    src/control.c
    src/state.c
    src/logbuf.c
    src/usb_cli.c
    src/shared.c
    src/tc.c
)

target_include_directories(avionics PRIVATE include)

target_link_libraries(avionics
    pico_stdlib
    hardware_spi
    hardware_pwm
    hardware_adc
    hardware_watchdog
    hardware_sync
    pico_multicore
)

pico_enable_stdio_usb(avionics 1)
pico_enable_stdio_uart(avionics 0)
pico_add_extra_outputs(avionics)
