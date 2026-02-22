cmake_minimum_required(VERSION 3.15)
project(PositionTracking)

find_package(Eigen3 REQUIRED)

add_library(map Map.cpp)
target_link_libraries(map Eigen3::Eigen)

add_executable(position_tracking main.cpp)
target_link_libraries(position_tracking map)

enable_testing()
add_executable(test_map map_tests.cpp)
target_link_libraries(test_map map)