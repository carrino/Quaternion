cmake_minimum_required(VERSION 3.17)
project(Quaternion CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_library(Quaternion SHARED Quaternion.cpp Quaternion.h)
target_include_directories(Quaternion PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# enable_testing() must be called from the top-level directory, otherwise
# running ctest from the build root finds no tests.
enable_testing()
add_subdirectory(test)
