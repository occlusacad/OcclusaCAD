cmake_minimum_required(VERSION 3.21)

# std::format / std::to_chars(double) in libc++ need macOS 13.3+.
set(CMAKE_OSX_DEPLOYMENT_TARGET "13.3" CACHE STRING "Minimum macOS version")

project(OcclusaCAD
    VERSION 0.1.0
    DESCRIPTION "OcclusaCAD dental CAD suite"
    LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()

option(OCCLUSACAD_BUILD_TESTS "Build unit tests" ON)
option(OCCLUSACAD_BUILD_TOOLS "Build developer tools (synthetic data generator)" ON)
option(OCCLUSACAD_WARNINGS_AS_ERRORS "Treat warnings as errors in first-party code" OFF)

# Put all executables side by side so OcclusaCAD DB can find OcclusaCAD.
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(OcclusaCompileOptions)
include(EmbedResource)

add_subdirectory(third_party)
add_subdirectory(src)

if(OCCLUSACAD_BUILD_TOOLS)
    add_subdirectory(tools)
endif()

if(OCCLUSACAD_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
