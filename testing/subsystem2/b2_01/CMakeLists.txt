cmake_minimum_required(VERSION 3.20)
project(buddy2_driver_foundation C)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(B2_SOURCE "${CMAKE_CURRENT_LIST_DIR}/../../../app_program/subsystem2")
option(B2_COVERAGE "Instrument native driver tests with GCC coverage" OFF)

add_library(b2_foundation STATIC
    ${B2_SOURCE}/motion_hal.c
    ${B2_SOURCE}/drv_motor.c
    ${B2_SOURCE}/drv_encoder.c
    ${B2_SOURCE}/wheel_measurement.c
    ${B2_SOURCE}/wheel_pid.c
    ${B2_SOURCE}/measured_motion_executor.c
    ${B2_SOURCE}/car_motion.c
    ${B2_SOURCE}/subsystem2.c
)
target_include_directories(b2_foundation PUBLIC ${B2_SOURCE} stubs)
target_compile_definitions(b2_foundation PRIVATE APP_B2_BENCH=1 APP_B2_MOVE_TEST=1)
add_library(b2_fake STATIC fake_motion_hal.c)
target_link_libraries(b2_fake PUBLIC b2_foundation)

if(MSVC)
    add_compile_options(/W4 /WX /UNDEBUG)
    target_compile_options(b2_foundation PRIVATE /W4 /WX /UNDEBUG)
    target_compile_options(b2_fake PRIVATE /W4 /WX /UNDEBUG)
else()
    add_compile_options(-Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    target_compile_options(b2_foundation PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    target_compile_options(b2_fake PRIVATE -Wall -Wextra -Werror -Wpedantic -UNDEBUG)
    if(B2_COVERAGE)
        target_compile_options(b2_foundation PRIVATE --coverage -O0)
        add_link_options(--coverage)
    endif()
endif()

enable_testing()
foreach(test_name IN ITEMS driver_foundation stop_interleavings encoder_snapshots lifecycle)
    add_executable(test_${test_name} test_${test_name}.c)
    target_link_libraries(test_${test_name} PRIVATE b2_fake)
    add_test(NAME b2_${test_name} COMMAND test_${test_name})
endforeach()

add_executable(test_bench_start test_bench_start.c ${B2_SOURCE}/b2_bench_target.c
    platform_stubs.c bsp_stubs.c ${B2_SOURCE}/b2_platform_rp2040.c
    ${B2_SOURCE}/motion_hal_rp2040.c)
target_compile_definitions(test_bench_start PRIVATE APP_B2_BENCH=1)
target_link_libraries(test_bench_start PRIVATE b2_runtime_policy)
add_test(NAME b2_bench_task COMMAND test_bench_start)
add_test(NAME b2_bench_create_failure COMMAND test_bench_start create)
add_test(NAME b2_bench_start_failure COMMAND test_bench_start start)
foreach(failure_step RANGE 1 20)
    add_test(NAME b2_lifecycle_failure_${failure_step}
        COMMAND test_lifecycle ${failure_step})
endforeach()
add_test(NAME b2_lifecycle_detach_failure COMMAND test_lifecycle detach)
add_test(NAME b2_lifecycle_start_clock_failure COMMAND test_lifecycle start_clock)
add_test(NAME b2_lifecycle_start_output_failure COMMAND test_lifecycle start_output)

add_executable(test_rp2040_adapter test_rp2040_adapter.c bsp_stubs.c
    ${B2_SOURCE}/motion_hal_rp2040.c)
target_link_libraries(test_rp2040_adapter PRIVATE b2_fake)
add_test(NAME b2_rp2040_adapter COMMAND test_rp2040_adapter)

add_library(b2_runtime_policy STATIC
    ${B2_SOURCE}/b2_trial.c ${B2_SOURCE}/b2_gesture.c
    ${B2_SOURCE}/../project_config/gpio_dispatcher.c
    ${B2_SOURCE}/../vehicle_controller/vehicle_controller.c
    ${B2_SOURCE}/../vehicle_controller/controller_service.c
    ${B2_SOURCE}/../vehicle_controller/controller_motion.c
    ${B2_SOURCE}/../vehicle_controller/safety.c
    ${B2_SOURCE}/../contracts/hardware_faults.c)
target_link_libraries(b2_runtime_policy PUBLIC b2_fake)
target_compile_definitions(b2_runtime_policy PRIVATE APP_B2_MOVE_TEST=1)
add_executable(test_runtime_policy test_runtime_policy.c)
target_link_libraries(test_runtime_policy PRIVATE b2_runtime_policy)
add_test(NAME b2_runtime_policy COMMAND test_runtime_policy)
add_executable(test_platform_binding test_platform_binding.c platform_stubs.c bsp_stubs.c
    ${B2_SOURCE}/b2_platform_rp2040.c ${B2_SOURCE}/motion_hal_rp2040.c)
target_compile_definitions(test_platform_binding PRIVATE APP_B2_BENCH=1)
target_link_libraries(test_platform_binding PRIVATE b2_runtime_policy)
add_test(NAME b2_platform_binding COMMAND test_platform_binding)
foreach(case IN ITEMS timer_owner gpio_owner irq_failure output_failure clock_failure)
    add_test(NAME b2_platform_${case} COMMAND test_platform_binding ${case})
endforeach()

add_executable(test_motion_control test_motion_control.c)
target_link_libraries(test_motion_control PRIVATE b2_runtime_policy)
foreach(case IN ITEMS measurement pid admission forward reverse left right both_wheels stall overshoot stop_race ttl controller result_adapter)
    add_test(NAME b2_motion_${case} COMMAND test_motion_control ${case})
endforeach()
add_executable(test_controller_transport test_controller_transport.c
    ${B2_SOURCE}/../vehicle_controller/controller_task.c)
target_link_libraries(test_controller_transport PRIVATE b2_runtime_policy)
foreach(case IN ITEMS stop latest result)
    add_test(NAME b2_transport_${case} COMMAND test_controller_transport ${case})
endforeach()
add_executable(test_car_motion test_car_motion.c ${B2_SOURCE}/../subsystem5/b5_motion.c)
target_link_libraries(test_car_motion PRIVATE b2_runtime_policy)
foreach(case IN ITEMS unqualified steering obstacle b5_goal junction stop_ids deadline)
    add_test(NAME b2_car_${case} COMMAND test_car_motion ${case})
endforeach()
add_executable(test_car_startup test_car_startup.c
    ${B2_SOURCE}/../vehicle_controller/car_run_target.c)
target_include_directories(test_car_startup PRIVATE ${B2_SOURCE} stubs)
target_compile_definitions(test_car_startup PRIVATE APP_CAR_RUN=1 APP_B2_BENCH=0)
foreach(case IN ITEMS unqualified qualified init_failure stop_ack)
    add_test(NAME b2_car_startup_${case} COMMAND test_car_startup ${case})
endforeach()
add_executable(test_circuit_test test_circuit_test.c bsp_stubs.c
    ${B2_SOURCE}/../diagnostics/circuit_test.c ${B2_SOURCE}/../diagnostics/circuit_controls.c
    ${B2_SOURCE}/../subsystem5/drv_servo.c)
target_compile_definitions(test_circuit_test PRIVATE APP_CIRCUIT_TEST=1 APP_B2_BENCH=1)
target_link_libraries(test_circuit_test PRIVATE b2_runtime_policy)
foreach(case IN ITEMS boot motor left_reverse right right_reverse both both_reverse motor_stop barcode sonar imu servo servo_stop late_stop sensor_failure critical_failure create_failure start_failure power20 power25 power30 power30_stop power_no_run power_wrap imu_stale)
    add_test(NAME circuit_${case} COMMAND test_circuit_test ${case})
endforeach()
add_executable(test_trial_limits test_trial_limits.c)
target_compile_definitions(test_trial_limits PRIVATE APP_B2_MOVE_TEST=1)
target_link_libraries(test_trial_limits PRIVATE b2_runtime_policy)
add_test(NAME b2_trial_diagnostic_limits COMMAND test_trial_limits)
add_executable(test_move_calibration test_move_calibration.c ${B2_SOURCE}/../diagnostics/b2_move_profile.c)
target_compile_definitions(test_move_calibration PRIVATE APP_B2_MOVE_TEST=1)
target_link_libraries(test_move_calibration PRIVATE b2_fake)
foreach(case IN ITEMS bounds deadline wrap stop_race forward_1m reverse left right)
    add_test(NAME b2_move_calibration_${case} COMMAND test_move_calibration ${case})
endforeach()
add_executable(test_move_task test_move_task.c ${B2_SOURCE}/../diagnostics/b2_move_profile.c
    ${B2_SOURCE}/../diagnostics/b2_move_test.c)
target_compile_definitions(test_move_task PRIVATE APP_B2_MOVE_TEST=1 APP_B2_BENCH=1)
target_link_libraries(test_move_task PRIVATE b2_runtime_policy)
foreach(case IN ITEMS boot locked missing_feedback flow_stop flow_stall flow_time critical create start long_locked watchdog)
    add_test(NAME b2_move_task_${case} COMMAND test_move_task ${case})
endforeach()
get_property(b2_tests DIRECTORY PROPERTY TESTS)
set_tests_properties(${b2_tests} PROPERTIES LABELS subsystem2)
