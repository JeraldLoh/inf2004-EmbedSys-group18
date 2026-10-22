# Buddy 4 host build: IMU, terrain and hump monitoring (portable C, no kernel).
#
#   imu_math.c           fixed-point decode, axis mapping, atan2, sine, heading
#   terrain_estimator.c  calibration, hump state machine and height, motion,
#                        collision, turn detection, range log, packed outputs
#   s4_shared.c          seqlock snapshot, odometry input word, 17-byte MQTT
#                        frame, command decoder, terrain_report_t adapter
#
# Not built here (they need the micro T-Kernel, so the course Make builds
# them): drv_lsm303.c, task_imu.c, s4_selftest.c, ../contracts/fault_mask.c.
add_library(subsystem4_host STATIC
    imu_math.c
    terrain_estimator.c
    s4_shared.c
)
target_include_directories(subsystem4_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(subsystem4_host PUBLIC group18_contracts)
group18_host_warnings(subsystem4_host)

if(BUILD_TESTING)
    # Simulated car: humps, collisions, turns, faults, packing, payloads.
    group18_add_host_test(subsystem4_host_tests subsystem4
        "${GROUP18_REPO_ROOT}/testing/host/test_terrain_estimator.c")
    target_link_libraries(subsystem4_host_tests PRIVATE subsystem4_host)
    if(NOT MSVC)
        target_link_libraries(subsystem4_host_tests PRIVATE m)   # libm for the test
    endif()
endif()
