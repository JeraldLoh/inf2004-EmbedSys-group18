# Buddy 3 host build: line following, barcode sampling, edge timing and
# Code 39 decoding.
#
# Not built here (they need the micro T-Kernel or the IR/ADC hardware, so the
# course Make builds them): subsystem3.c, subsystem3_test.c, drv_ir.c. Add new
# portable Buddy 3 sources (no tk_* calls, no GPIO/ADC, no app.h) to
# subsystem3_host below.
add_library(subsystem3_host STATIC
    line_follow.c
    barcode_sensor.c
    barcode_edges.c
    barcode_decoder.c
    barcode_map.c
)
target_include_directories(subsystem3_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(subsystem3_host PUBLIC group18_contracts)
group18_host_warnings(subsystem3_host)

if(BUILD_TESTING)
    # The junction tests drive the real vehicle controller, which has no host
    # library of its own yet, so its sources are compiled into the test.
    group18_add_host_test(subsystem3_host_tests subsystem3
        "${GROUP18_REPO_ROOT}/testing/host/test_subsystem3_sensors.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_controller/vehicle_controller.c"
        "${CMAKE_CURRENT_SOURCE_DIR}/../vehicle_controller/safety.c")
    target_sources(subsystem3_host_tests PRIVATE
        "${CMAKE_CURRENT_SOURCE_DIR}/../contracts/hardware_faults.c")
    target_link_libraries(subsystem3_host_tests PRIVATE subsystem3_host)
endif()
