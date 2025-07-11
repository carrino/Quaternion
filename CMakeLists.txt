cmake_minimum_required(VERSION 3.17)
project(Quaternion)

set(CMAKE_CXX_STANDARD 14)

# Find Boost headers
find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

add_library(Quaternion SHARED Quaternion.cpp Quaternion.h)

enable_testing() # Moved from test/CMakeLists.txt
add_subdirectory(test)

