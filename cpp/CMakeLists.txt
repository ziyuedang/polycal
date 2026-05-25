cmake_minimum_required(VERSION 3.20)

project(polycal_cpp LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(APPLE)
    if(CMAKE_HOST_SYSTEM_PROCESSOR STREQUAL "arm64")
        list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
    else()
        list(APPEND CMAKE_PREFIX_PATH "/usr/local")
    endif()
endif()

find_package(Eigen3 REQUIRED)

find_path(SOPHUS_INCLUDE_DIR sophus/se3.hpp
    HINTS ${CMAKE_PREFIX_PATH}/include)
if(NOT SOPHUS_INCLUDE_DIR)
    message(FATAL_ERROR "Sophus not found")
endif()

add_library(polycal_ekf STATIC
    src/ekf.cpp)

target_include_directories(polycal_ekf
    PUBLIC
        include
        ${SOPHUS_INCLUDE_DIR})

target_link_libraries(polycal_ekf
    PUBLIC
        Eigen3::Eigen)

include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(test_ekf
    tests/test_ekf.cpp)

target_link_libraries(test_ekf
    PRIVATE
        polycal_ekf
        GTest::gtest_main)

include(GoogleTest)
gtest_discover_tests(test_ekf)
