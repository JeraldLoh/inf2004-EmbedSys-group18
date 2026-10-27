# Buddy 3 host build: PLACEHOLDER (nothing is built yet).
#
# Ready to add: barcode_decoder.c is already hardware-independent. Its test,
# testing/host/test_perception_math.c, also checks Buddy 4's lsm303_decode.c,
# so it needs both libraries (or split it into a barcode-only test).
# To build it here:
#
#   add_library(subsystem3_host STATIC barcode_decoder.c)
#   target_include_directories(subsystem3_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
#   target_link_libraries(subsystem3_host PUBLIC group18_contracts)
#   group18_host_warnings(subsystem3_host)
#
#   if(BUILD_TESTING)
#       group18_add_host_test(subsystem3_host_tests subsystem3
#           "${GROUP18_REPO_ROOT}/testing/host/test_perception_math.c")
#       target_link_libraries(subsystem3_host_tests
#           PRIVATE subsystem3_host subsystem4_host)
#   endif()
#
# subsystem4_host is defined after this folder, which CMake allows: targets
# are resolved when the build is generated.
# Only portable code belongs here: no tk_* calls, no GPIO/ADC, no app.h.
# Keep test files in testing/host/ (or a tests/ subfolder), never directly in
# this folder: the course Make would compile them into the firmware.
# See app_program/subsystem4/CMakeLists.txt for a working example.
