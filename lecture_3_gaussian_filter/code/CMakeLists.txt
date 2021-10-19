cmake_minimum_required(VERSION 3.10)
project(code)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

add_executable(code kalman_filter_example.cpp)
