cmake_minimum_required(VERSION 3.17)
project(Quaternion)

set(CMAKE_CXX_STANDARD 14)

add_library(Quaternion SHARED Quaternion.cpp Quaternion.h)

enable_testing()
add_subdirectory(test)

