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

add_executable(obstacle_avoidance_host_tests
    subsystem5/test_main.c
    subsystem5/test_range.c
    subsystem5/test_obstacle.c
    subsystem5/test_avoidance.c
    ../../app_program/subsystem5/range.c
    ../../app_program/subsystem5/obstacle.c
    ../../app_program/subsystem5/avoidance.c
)

target_include_directories(obstacle_avoidance_host_tests PRIVATE
    ../../app_program/subsystem5
)

if(MSVC)
    target_compile_options(obstacle_avoidance_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(obstacle_avoidance_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    target_link_libraries(obstacle_avoidance_host_tests PRIVATE m)
endif()

add_test(NAME obstacle_avoidance_host_tests
         COMMAND obstacle_avoidance_host_tests)

add_executable(subsystem3_sensor_host_tests
    test_subsystem3_sensors.c
    ../../app_program/subsystem3/line_follow.c
    ../../app_program/subsystem3/barcode_sensor.c
    ../../app_program/subsystem3/barcode_decoder.c
    ../../app_program/subsystem3/barcode_edges.c
    ../../app_program/vehicle_controller/vehicle_controller.c
    ../../app_program/vehicle_controller/safety.c
)

if(MSVC)
    target_compile_options(subsystem3_sensor_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(subsystem3_sensor_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME subsystem3_sensor_host_tests
         COMMAND subsystem3_sensor_host_tests)
