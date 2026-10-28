# Buddy 4 host build: portable LSM303DLHC raw-sample decoding.
#
# Not built here (they need the micro T-Kernel, so the course Make builds
# them): subsystem4.c, subsystem4_test.c. Add new portable Buddy 4 sources
# (no tk_* calls, no I2C, no app.h) to subsystem4_host below.
add_library(subsystem4_host STATIC
    lsm303_decode.c
)
target_include_directories(subsystem4_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(subsystem4_host PUBLIC group18_contracts)
group18_host_warnings(subsystem4_host)

if(BUILD_TESTING)
    group18_add_host_test(subsystem4_host_tests subsystem4
        "${GROUP18_REPO_ROOT}/testing/host/test_lsm303_decode.c")
    target_link_libraries(subsystem4_host_tests PRIVATE subsystem4_host)
endif()
