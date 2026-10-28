# Buddy 2 host build: PLACEHOLDER (nothing is built yet).
#
# Ready to add: motion_math.c is already hardware-independent and is tested
# by testing/host/test_motion_math.c (currently built by testing/host/).
# To build it here instead:
#
#   add_library(subsystem2_host STATIC motion_math.c)
#   target_include_directories(subsystem2_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
#   target_link_libraries(subsystem2_host PUBLIC group18_contracts)
#   group18_host_warnings(subsystem2_host)
#
#   if(BUILD_TESTING)
#       group18_add_host_test(subsystem2_host_tests subsystem2
#           "${GROUP18_REPO_ROOT}/testing/host/test_motion_math.c")
#       target_link_libraries(subsystem2_host_tests PRIVATE subsystem2_host)
#   endif()
#
# Only portable code belongs here: no tk_* calls, no GPIO/PWM, no app.h.
# Keep test files in testing/host/ (or a tests/ subfolder), never directly in
# this folder: the course Make would compile them into the firmware.
# See app_program/subsystem4/CMakeLists.txt for a working example.
