# Buddy 1 host build: PLACEHOLDER (nothing is built yet).
#
# Add this subsystem's hardware-independent sources here: no tk_* calls, no
# network stack, no app.h. For example, message encode/decode or validation:
#
#   add_library(subsystem1_host STATIC my_module.c)
#   target_include_directories(subsystem1_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
#   target_link_libraries(subsystem1_host PUBLIC group18_contracts)
#   group18_host_warnings(subsystem1_host)
#
#   if(BUILD_TESTING)
#       group18_add_host_test(subsystem1_host_tests subsystem1
#           "${GROUP18_REPO_ROOT}/testing/host/test_subsystem1.c")
#       target_link_libraries(subsystem1_host_tests PRIVATE subsystem1_host)
#   endif()
#
# Keep test files in testing/host/ (or a tests/ subfolder), never directly in
# this folder: the course Make would compile them into the firmware.
# See app_program/subsystem4/CMakeLists.txt for a working example.
