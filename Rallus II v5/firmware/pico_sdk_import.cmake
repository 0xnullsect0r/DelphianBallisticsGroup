# Locate the pico-sdk from an explicit -DPICO_SDK_PATH=... or the PICO_SDK_PATH
# environment variable. No automatic git fetch: install the SDK yourself, e.g.
#   git clone --recursive https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk

if (NOT DEFINED PICO_SDK_PATH)
    if (DEFINED ENV{PICO_SDK_PATH})
        set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
        message(STATUS "Using PICO_SDK_PATH from environment: ${PICO_SDK_PATH}")
    elseif (EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../pico-sdk")
        set(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/../../../pico-sdk")
        message(STATUS "Using PICO_SDK_PATH next to home: ${PICO_SDK_PATH}")
    endif ()
endif ()

if (NOT DEFINED PICO_SDK_PATH)
    message(FATAL_ERROR
        "PICO_SDK_PATH not set. Clone the SDK with submodules and pass it:\n"
        "  git clone --recursive https://github.com/raspberrypi/pico-sdk.git ~/pico-sdk\n"
        "  cmake -B build -DPICO_SDK_PATH=~/pico-sdk")
endif ()

set(PICO_SDK_PATH "${PICO_SDK_PATH}" CACHE PATH "Path to the Raspberry Pi Pico SDK")

include(${PICO_SDK_PATH}/pico_sdk_init.cmake)
