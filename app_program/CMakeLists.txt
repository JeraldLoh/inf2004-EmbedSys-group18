# Group 18 team host build: hardware-independent code and unit tests on a PC.
#
# This does NOT build the firmware. The .uf2 image is built by the course GNU
# Make in build_make/, which compiles the kernel, drivers and every .c file
# one level under app_program/. Each subsystem adds only its portable sources
# here (no tk_* calls, no GPIO/I2C/ADC access, no app.h).
#
# Build from the repository root, with the build folder OUTSIDE app_program:
#   cmake -S app_program -B build/app-host
#   cmake --build build/app-host
#   ctest --test-dir build/app-host --output-on-failure
# One subsystem only:  ctest --test-dir build/app-host -L subsystem4
#
# Never put a test .c file directly in app_program/<folder>/: the course Make
# would compile it into the firmware (a second main()). Keep tests in
# testing/host/ or in a tests/ subfolder one level deeper.
cmake_minimum_required(VERSION 3.20)
project(group18_app_host C)

if(CMAKE_CROSSCOMPILING)
    message(FATAL_ERROR "app_program/CMakeLists.txt is a host (PC) build only; "
                        "use build_make/ for the firmware.")
endif()

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

set(GROUP18_REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/..")

include(CTest)   # BUILD_TESTING, ON by default

# Shared headers (contracts/system_types.h, control_messages.h).
add_library(group18_contracts INTERFACE)
target_include_directories(group18_contracts INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}")

# Same strictness as testing/host: warnings are errors, asserts always on.
function(group18_host_warnings target)
    if(MSVC)
        target_compile_options(${target} PRIVATE /W4 /WX)
    else()
        target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -Wpedantic)
    endif()
endfunction()

# Register one test executable for a subsystem, labelled so it can be run
# alone with `ctest -L subsystemN`.
function(group18_add_host_test name label)
    add_executable(${name} ${ARGN})
    group18_host_warnings(${name})
    if(MSVC)
        target_compile_options(${name} PRIVATE /UNDEBUG)
    else()
        target_compile_options(${name} PRIVATE -UNDEBUG)
    endif()
    add_test(NAME ${name} COMMAND ${name})
    set_tests_properties(${name} PROPERTIES LABELS ${label})
endfunction()

add_subdirectory(subsystem1)
add_subdirectory(subsystem2)
add_subdirectory(subsystem3)
add_subdirectory(subsystem4)
add_subdirectory(subsystem5)
