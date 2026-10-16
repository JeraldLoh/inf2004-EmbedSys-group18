cmake_minimum_required(VERSION 3.20)
project(vehicle_controller_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(vehicle_controller_host_tests
    test_vehicle_controller.c
    ../../app/safety.c
    ../../app/vehicle_controller.c
)

if(MSVC)
    target_compile_options(vehicle_controller_host_tests PRIVATE /W4 /WX)
else()
    target_compile_options(vehicle_controller_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic)
endif()

enable_testing()
add_test(NAME vehicle_controller_host_tests
         COMMAND vehicle_controller_host_tests)
