cmake_minimum_required(VERSION 3.10)
project(kalman_filter)

set(CMAKE_CXX_STANDARD 11)

find_package(Eigen3 REQUIRED)

find_package(Python2 COMPONENTS Development NumPy)

include_directories(
    ${EIGEN3_INCLUDE_DIRS}
    3rd
)

add_executable(kalman_filter kalman_filter_example.cpp)
target_include_directories(kalman_filter PRIVATE ${Python2_INCLUDE_DIRS} ${Python2_NumPy_INCLUDE_DIRS})
target_link_libraries(kalman_filter Python2::Python Python2::NumPy)
