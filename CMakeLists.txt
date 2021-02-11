cmake_minimum_required(VERSION 3.17)
project(Quaternion)

set(CMAKE_CXX_STANDARD 14)

add_library(Quaternion SHARED Quaternion.cpp Quaternion.h)

add_subdirectory(test)

