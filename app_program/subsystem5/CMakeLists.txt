# Buddy 5 host build: ultrasonic reading filter, obstacle geometry and the
# detect / scan / decide / bypass / reacquire state machine.
#
# Not built here (they need the micro T-Kernel or the sonar/servo hardware,
# so the course Make builds them): subsystem5.c, subsystem5_test.c,
# drv_sonar.c, drv_servo.c, b5_run_target.c. Add new portable Buddy 5 sources
# (no tk_* calls, no GPIO/PWM/IRQ, no app.h) to subsystem5_host below.
add_library(subsystem5_host STATIC
    range.c
    obstacle.c
    avoidance.c
)

# The mission bridge talks to the vehicle controller, which has no host
# library of its own yet, so its sources are compiled in (as subsystem 3 does).
set(B5_CONTROLLER_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/b5_mission_bridge.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_controller/controller_service.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_controller/vehicle_controller.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_controller/safety.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../contracts/hardware_faults.c")
target_include_directories(subsystem5_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(subsystem5_host PUBLIC group18_contracts)
group18_host_warnings(subsystem5_host)

if(BUILD_TESTING)
    set(B5_TESTS "${GROUP18_REPO_ROOT}/testing/host/subsystem5")
    group18_add_host_test(subsystem5_host_tests subsystem5
        "${B5_TESTS}/test_main.c"
        "${B5_TESTS}/test_range.c"
        "${B5_TESTS}/test_obstacle.c"
        "${B5_TESTS}/test_avoidance.c")
    target_link_libraries(subsystem5_host_tests PRIVATE subsystem5_host)
    if(NOT MSVC)
        # The tests compare against the host libm; the firmware code uses none.
        target_link_libraries(subsystem5_host_tests PRIVATE m)
    endif()

    group18_add_host_test(subsystem5_bridge_host_tests subsystem5
        "${B5_TESTS}/test_bridge.c" ${B5_CONTROLLER_SOURCES})
    target_link_libraries(subsystem5_bridge_host_tests PRIVATE subsystem5_host)
    if(NOT MSVC)
        target_link_libraries(subsystem5_bridge_host_tests PRIVATE m)
    endif()
endif()
