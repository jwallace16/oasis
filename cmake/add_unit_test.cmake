# Function: add_unit_test
# Usage:
#   add_unit_test(NAME MyTest SOURCES test_main.cpp test_math.cpp LIBS mylib)
# or, without libraries
#   add_unit_test(NAME MyTest SOURCES testmain.cpp)
function(add_unit_test TEST_NAME)
    # Parse arguments
    set(options "")
    set(oneValueArgs "")
    set(multiValueArgs SOURCES LIBS)
    cmake_parse_arguments(ARG "${options}" "${oneValueARgs}" "${multiValueArgs}" ${ARGN})

    # Fetch google test
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )

    # Make gtest available
    FetchContent_MakeAvailable(googletest)

    # Enable testing
    enable_testing()

    # Create the test executable
    add_executable(${TEST_NAME} ${ARG_SOURCES})

    # Link in dependencies
    target_link_libraries(${TEST_NAME}
        gtest_main
        ${ARG_LIBS}
    )

    # Discover tests automatically
    # include(GoogleTest)
    # gtest_discover_tests(${TEST_NAME})
endfunction()