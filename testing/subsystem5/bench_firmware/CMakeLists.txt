# Buddy 5 bench-test firmware (Pico SDK, NOT the micro T-Kernel image).
# Tests the HC-SR04 and scan servo on their own. The avoidance logic is built
# straight from app_program/subsystem5, so this always tests the team's copy.

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.3.1)
set(toolchainVersion 15_2_Rel1)
set(picotoolVersion 2.3.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(buddy5_bench C CXX ASM)
pico_sdk_init()

set(B5_CORE ${CMAKE_CURRENT_LIST_DIR}/../../../app_program/subsystem5)

add_library(buddy5_core STATIC
        ${B5_CORE}/range.c
        ${B5_CORE}/obstacle.c
        ${B5_CORE}/avoidance.c
        )
target_include_directories(buddy5_core PUBLIC ${B5_CORE})
target_compile_options(buddy5_core PRIVATE
        -Wall -Wextra -Werror -Wshadow -Wdouble-promotion -Wstrict-prototypes)

add_executable(buddy5_bench main.c ultrasonic.c servo.c)
target_compile_options(buddy5_bench PRIVATE -Wall -Wextra -Werror)
target_link_libraries(buddy5_bench
        buddy5_core
        pico_stdlib
        hardware_pwm
        hardware_clocks
        hardware_irq
        hardware_watchdog
        hardware_flash   # servo calibration saved in the last flash sector
        pico_flash
        )
pico_enable_stdio_usb(buddy5_bench 1)
pico_enable_stdio_uart(buddy5_bench 0)
pico_add_extra_outputs(buddy5_bench)
