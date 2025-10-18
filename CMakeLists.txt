cmake_minimum_required(VERSION 3.16)
project(OASIS
        VERSION 1.0.0
        DESCRIPTION "Objective Aerospace System Integration Simulation"
        LANGUAGES CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Build options
option(BUILD_TESTING "Build unit tests" ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings" OFF)
option(BUILD_EXAMPLES "Build examples" OFF)
option(BUILD_DOCS "Build documentation" OFF)

# Set default build type
# if(NOT CMAKE_BUILD_TYPE)
#     set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose build type" FORCE)
#     set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS Debug Release RelWithDebInfo MinSizeRel)
# endif()

# Include cmake modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
include(cmake/add_unit_test.cmake)

# Find required packages
# find_package(Threads REQUIRED)

# Include directories
include_directories(${CMAKE_SOURCE_DIR}/include)

# Compiler-specific settings
add_compile_options(
    -Wall
    -Wextra
    -Wpedantic
    $<$<CONFIG:Release>:-O3>
    $<$<CONFIG:Release>:-march=native>
    $<$<CONFIG:Debug>:-O0>
    $<$<CONFIG:Debug>:-g>
)


# Enable position independent code
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Add subdirectories
# add_subdirectory(src)

# if(BUILD_TESTING)
#     add_subdirectory(tests)
# endif()

add_subdirectory(tests)

# if(BUILD_PYTHON_BINDINGS)
#     add_subdirectory(bindings)
# endif()

# if(BUILD_EXAMPLES)
#     add_subdirectory(examples)
# endif()

# Print build configuration
message(STATUS "OASIS Build Configuration:")
message(STATUS "  Build Type: ${CMAKE_BUILD_TYPE}")
message(STATUS "  C++ Standard: ${CMAKE_CXX_STANDARD}")
message(STATUS "  Testing: ${BUILD_TESTING}")
message(STATUS "  Python Bindings: ${BUILD_PYTHON_BINDINGS}")
message(STATUS "  Examples: ${BUILD_EXAMPLES}")
message(STATUS "  Documentation: ${BUILD_DOCS}")

# Installation (future)
# include(GNUInstallDirs)
# install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})