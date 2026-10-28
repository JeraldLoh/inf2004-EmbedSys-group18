# Buddy 5 host build: PLACEHOLDER (nothing is built yet).
#
# Add this subsystem's hardware-independent sources here: no tk_* calls, no
# GPIO/PWM/IRQ, no app.h. For example, obstacle profiling or bypass planning:
#
#   add_library(subsystem5_host STATIC my_module.c)
#   target_include_directories(subsystem5_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
#   target_link_libraries(subsystem5_host PUBLIC group18_contracts)
#   group18_host_warnings(subsystem5_host)
#
#   if(BUILD_TESTING)
#       group18_add_host_test(subsystem5_host_tests subsystem5
#           "${GROUP18_REPO_ROOT}/testing/host/test_subsystem5.c")
#       target_link_libraries(subsystem5_host_tests PRIVATE subsystem5_host)
#   endif()
#
# Keep test files in testing/host/ (or a tests/ subfolder), never directly in
# this folder: the course Make would compile them into the firmware.
# See app_program/subsystem4/CMakeLists.txt for a working example.
