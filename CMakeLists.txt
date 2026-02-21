cmake_minimum_required(VERSION 3.15)
project(PositionTracking)

find_package(Eigen3 REQUIRED)

add_library(map Map.cpp)
target_link_libraries(map Eigen3::Eigen)