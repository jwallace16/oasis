# CMakeLists.txt for all OASIS tests
add_subdirectory(cpp)

# Only build tests if BUILD_TESTING is enabled
# if(BUILD_TESTING_OBE)
#     message(STATUS "Building OASIS tests")
    
#     # Enable testing framework
#     enable_testing()
#     include(CTest)
    
#     # Global test settings
#     set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/tests/bin)
    
#     # Add C++ tests
#     add_subdirectory(cpp)
    
#     # Add Python tests (when implemented)
#     # add_subdirectory(python)
    
#     # Add integration tests (when implemented)  
#     # add_subdirectory(integration)
    
#     # Create convenience targets for different test categories
#     add_custom_target(all_tests
#         COMMENT "Building all OASIS tests"
#     )
    
#     add_dependencies(all_tests cpp_tests)
    
#     # Custom test targets for different categories
#     add_custom_target(unit_tests
#         COMMENT "Running unit tests"
#         COMMAND ${CMAKE_CTEST_COMMAND} -L unit --output-on-failure
#         DEPENDS cpp_tests
#     )
    
#     add_custom_target(math_tests
#         COMMENT "Running math tests"
#         COMMAND ${CMAKE_CTEST_COMMAND} -L math --output-on-failure
#         DEPENDS cpp_tests
#     )
    
#     # Print test summary
#     add_custom_target(test_summary
#         COMMAND ${CMAKE_CTEST_COMMAND} --print-labels
#         COMMENT "Available test labels"
#     )
    
# else()
#     message(STATUS "Testing disabled. Use -DBUILD_TESTING=ON to enable tests.")
# endif()