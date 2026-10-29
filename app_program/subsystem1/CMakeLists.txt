# Buddy 1 portable host codecs and tests. Firmware stays in build_make/.
# The bridge uses tk_* and is tested by the mocked suite in testing/host/.
add_library(subsystem1_host STATIC
    buddy1_protocol.c
    buddy1_binary.c
    buddy1_diagnostics.c
)
target_include_directories(subsystem1_host PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")
target_link_libraries(subsystem1_host PUBLIC group18_contracts)
group18_host_warnings(subsystem1_host)

if(BUILD_TESTING)
    group18_add_host_test(subsystem1_protocol_tests subsystem1
        "${GROUP18_REPO_ROOT}/host_tests/protocol_test.c")
    target_link_libraries(subsystem1_protocol_tests PRIVATE subsystem1_host)

    group18_add_host_test(subsystem1_binary_tests subsystem1
        "${GROUP18_REPO_ROOT}/host_tests/binary_test.c")
    target_link_libraries(subsystem1_binary_tests PRIVATE subsystem1_host)

    group18_add_host_test(subsystem1_diagnostics_tests subsystem1
        "${GROUP18_REPO_ROOT}/host_tests/diagnostics_test.c")
    target_link_libraries(subsystem1_diagnostics_tests PRIVATE subsystem1_host)

    group18_add_host_test(subsystem1_retry_tests subsystem1
        "${GROUP18_REPO_ROOT}/host_tests/retry_test.c")
    target_include_directories(subsystem1_retry_tests PRIVATE
        "${GROUP18_REPO_ROOT}/lib/libwifi/sysdepend/pico_rp2040")
endif()
