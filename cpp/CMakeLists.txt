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

include(FetchContent)

find_path(SOPHUS_INCLUDE_DIR sophus/se3.hpp
    HINTS ${CMAKE_PREFIX_PATH}/include)
set(POLYCAL_FETCHED_SOPHUS OFF)
if(NOT SOPHUS_INCLUDE_DIR)
    set(POLYCAL_FETCHED_SOPHUS ON)
    FetchContent_Declare(
        fmt
        GIT_REPOSITORY https://github.com/fmtlib/fmt.git
        GIT_TAG        10.2.1)
    FetchContent_Declare(
        sophus
        URL https://github.com/strasdat/Sophus/archive/refs/tags/1.22.10.zip)
    FetchContent_MakeAvailable(fmt sophus)
    set(SOPHUS_INCLUDE_DIR ${sophus_SOURCE_DIR})
endif()

add_library(polycal_ekf STATIC
    src/ekf.cpp
    src/cusum.cpp)

target_include_directories(polycal_ekf
    PUBLIC
        include
        ${SOPHUS_INCLUDE_DIR})

if(POLYCAL_FETCHED_SOPHUS)
    target_link_libraries(polycal_ekf
        PUBLIC
            Eigen3::Eigen
            fmt::fmt)
else()
    target_link_libraries(polycal_ekf
        PUBLIC
            Eigen3::Eigen)
endif()

FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

FetchContent_Declare(
    pybind11
    GIT_REPOSITORY https://github.com/pybind/pybind11.git
    GIT_TAG        v2.12.0)
FetchContent_MakeAvailable(pybind11)

enable_testing()

add_executable(test_ekf
    tests/test_ekf.cpp)

if(POLYCAL_FETCHED_SOPHUS)
    target_link_libraries(test_ekf
        PRIVATE
            polycal_ekf
            GTest::gtest_main
            fmt::fmt)
else()
    target_link_libraries(test_ekf
        PRIVATE
            polycal_ekf
            GTest::gtest_main)
endif()

include(GoogleTest)
gtest_discover_tests(test_ekf)

pybind11_add_module(_polycal_cpp
    src/bindings.cpp)

target_include_directories(_polycal_cpp
    PRIVATE
        include
        ${SOPHUS_INCLUDE_DIR})

target_link_libraries(_polycal_cpp
    PRIVATE
        polycal_ekf
        Eigen3::Eigen)

if(TARGET fmt::fmt)
    target_link_libraries(_polycal_cpp
        PRIVATE
            fmt::fmt)
endif()

set_target_properties(_polycal_cpp PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY
        ${CMAKE_SOURCE_DIR}/../python/polycal)
