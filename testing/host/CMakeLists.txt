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
