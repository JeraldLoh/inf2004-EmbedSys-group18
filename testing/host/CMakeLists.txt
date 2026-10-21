cmake_minimum_required(VERSION 3.20)
project(vehicle_controller_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(vehicle_controller_host_tests
    test_vehicle_controller.c
    ../../app_program/vehicle_controller/safety.c
    ../../app_program/vehicle_controller/vehicle_controller.c
)

if(MSVC)
    target_compile_options(vehicle_controller_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(vehicle_controller_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

enable_testing()
add_test(NAME vehicle_controller_host_tests
         COMMAND vehicle_controller_host_tests)

add_executable(motion_math_host_tests
    test_motion_math.c
    ../../app_program/subsystem2/motion_math.c
)

if(MSVC)
    target_compile_options(motion_math_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(motion_math_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME motion_math_host_tests
         COMMAND motion_math_host_tests)

add_executable(perception_math_host_tests
    test_perception_math.c
    ../../app_program/subsystem3/barcode_decoder.c
    ../../app_program/subsystem4/lsm303_decode.c
)

if(MSVC)
    target_compile_options(perception_math_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(perception_math_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME perception_math_host_tests
         COMMAND perception_math_host_tests)

add_executable(terrain_estimator_host_tests
    test_terrain_estimator.c
    ../../app_program/subsystem4/imu_math.c
    ../../app_program/subsystem4/terrain_estimator.c
    ../../app_program/subsystem4/s4_shared.c
)

if(MSVC)
    target_compile_options(terrain_estimator_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(terrain_estimator_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    target_link_libraries(terrain_estimator_host_tests PRIVATE m)
endif()

add_test(NAME terrain_estimator_host_tests
         COMMAND terrain_estimator_host_tests)
