cmake_minimum_required(VERSION 3.20)
project(vehicle_controller_host_tests C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)

add_executable(vehicle_controller_host_tests
    test_vehicle_controller.c
    ../../app_program/contracts/hardware_faults.c
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
    ../../app_program/subsystem3/barcode_map.c
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

# Controller task core: events in, STOP out through a fake motion output.
add_executable(controller_service_host_tests
    test_controller_service.c
    ../../app_program/contracts/hardware_faults.c
    ../../app_program/vehicle_controller/safety.c
    ../../app_program/vehicle_controller/vehicle_controller.c
    ../../app_program/vehicle_controller/controller_service.c
)

if(MSVC)
    target_compile_options(controller_service_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(controller_service_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME controller_service_host_tests
         COMMAND controller_service_host_tests)

# Buddy 5 avoidance states driving the real controller's obstacle missions.
add_executable(obstacle_mission_bridge_host_tests
    subsystem5/test_bridge.c
    ../../app_program/subsystem5/range.c
    ../../app_program/subsystem5/obstacle.c
    ../../app_program/subsystem5/avoidance.c
    ../../app_program/subsystem5/b5_mission_bridge.c
    ../../app_program/contracts/hardware_faults.c
    ../../app_program/vehicle_controller/safety.c
    ../../app_program/vehicle_controller/vehicle_controller.c
    ../../app_program/vehicle_controller/controller_service.c
)

target_include_directories(obstacle_mission_bridge_host_tests PRIVATE
    ../../app_program/subsystem5
)

if(MSVC)
    target_compile_options(obstacle_mission_bridge_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(obstacle_mission_bridge_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    target_link_libraries(obstacle_mission_bridge_host_tests PRIVATE m)
endif()

add_test(NAME obstacle_mission_bridge_host_tests
         COMMAND obstacle_mission_bridge_host_tests)

# Draft Buddy 2 motion executor core (continuous requests + finite goals).
add_executable(motion_executor_host_tests
    test_motion_executor.c
    ../../app_program/subsystem2/motion_executor.c
    ../../app_program/subsystem2/motion_math.c
)

if(MSVC)
    target_compile_options(motion_executor_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(motion_executor_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME motion_executor_host_tests
         COMMAND motion_executor_host_tests)

# Buddy 5 moves as authority-stamped Buddy 2 goals (CAR_RUN wiring).
add_executable(b5_motion_host_tests
    subsystem5/test_b5_motion.c
    ../../app_program/subsystem5/b5_motion.c
)

target_include_directories(b5_motion_host_tests PRIVATE
    ../../app_program/subsystem5
)

if(MSVC)
    target_compile_options(b5_motion_host_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(b5_motion_host_tests PRIVATE
        -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()

add_test(NAME b5_motion_host_tests
         COMMAND b5_motion_host_tests)

# Buddy 1 portable protocol/retry and mocked lifecycle tests. No hardware claims.
add_executable(buddy1_protocol_tests
    ../../host_tests/protocol_test.c
    ../../app_program/subsystem1/buddy1_protocol.c)
target_include_directories(buddy1_protocol_tests PRIVATE ../../app_program/subsystem1)
add_executable(buddy1_retry_tests ../../host_tests/retry_test.c)
target_include_directories(buddy1_retry_tests PRIVATE ../../lib/libwifi/sysdepend/pico_rp2040)
set(BUDDY1_TEST_TARGETS buddy1_protocol_tests buddy1_retry_tests)
foreach(mode 0 1)
    add_executable(buddy1_lifecycle_${mode}
        test_buddy1_lifecycle.c
        ../../app_program/subsystem1/subsystem1.c
        ../../app_program/app_main.c)
    target_include_directories(buddy1_lifecycle_${mode} PRIVATE buddy1_stubs)
    target_compile_definitions(buddy1_lifecycle_${mode} PRIVATE B1_STANDALONE_DEMO=${mode})
    list(APPEND BUDDY1_TEST_TARGETS buddy1_lifecycle_${mode})
endforeach()
foreach(test_target IN LISTS BUDDY1_TEST_TARGETS)
    if(MSVC)
        target_compile_options(${test_target} PRIVATE /W4 /WX /UNDEBUG)
    else()
        target_compile_options(${test_target} PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    endif()
    add_test(NAME ${test_target} COMMAND ${test_target})
endforeach()

add_executable(buddy1_heartbeat_tests
    test_buddy1_heartbeat.c
    ../../app_program/subsystem1/buddy1_bridge.c
    ../../app_program/subsystem1/buddy1_protocol.c
    ../../app_program/subsystem1/buddy1_diagnostics.c
    ../../app_program/subsystem1/buddy1_binary.c)
target_include_directories(buddy1_heartbeat_tests PRIVATE buddy1_stubs)
if(MSVC)
    target_compile_options(buddy1_heartbeat_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(buddy1_heartbeat_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()
add_test(NAME buddy1_heartbeat_regular COMMAND buddy1_heartbeat_tests 0)
add_test(NAME buddy1_heartbeat_busy_once COMMAND buddy1_heartbeat_tests 5020)
add_test(NAME buddy1_heartbeat_busy_100ms COMMAND buddy1_heartbeat_tests 5100)

add_executable(buddy1_binary_tests ../../host_tests/binary_test.c
    ../../app_program/subsystem1/buddy1_binary.c)
if(MSVC)
    target_compile_options(buddy1_binary_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(buddy1_binary_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()
add_test(NAME buddy1_binary_tests COMMAND buddy1_binary_tests)
add_executable(buddy1_diagnostics_tests ../../host_tests/diagnostics_test.c
    ../../app_program/subsystem1/buddy1_binary.c ../../app_program/subsystem1/buddy1_diagnostics.c)
if(MSVC)
    target_compile_options(buddy1_diagnostics_tests PRIVATE /W4 /WX /UNDEBUG)
else()
    target_compile_options(buddy1_diagnostics_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()
add_test(NAME buddy1_diagnostics_tests COMMAND buddy1_diagnostics_tests)

# Live Buddy 1 integration: sections merge without stealing producer events.
add_executable(buddy1_car_tests test_buddy1_car.c
    ../../app_program/subsystem1/buddy1_bridge.c
    ../../app_program/subsystem1/buddy1_binary.c
    ../../app_program/subsystem1/buddy1_diagnostics.c
    ../../app_program/subsystem1/buddy1_protocol.c)
target_include_directories(buddy1_car_tests PRIVATE buddy1_stubs)
target_compile_definitions(buddy1_car_tests PRIVATE B1_DEMO=0)
if(MSVC)
 target_compile_options(buddy1_car_tests PRIVATE /W4 /WX /UNDEBUG)
else()
 target_compile_options(buddy1_car_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()
add_test(NAME buddy1_car_tests COMMAND buddy1_car_tests)

add_executable(remote_start_tests test_remote_start.c
    ../../app_program/contracts/hardware_faults.c
    ../../app_program/vehicle_controller/safety.c
    ../../app_program/vehicle_controller/vehicle_controller.c
    ../../app_program/vehicle_controller/controller_service.c)
if(MSVC)
 target_compile_options(remote_start_tests PRIVATE /W4 /WX /UNDEBUG)
else()
 target_compile_options(remote_start_tests PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
endif()
add_test(NAME remote_start_tests COMMAND remote_start_tests)
