cmake_minimum_required(VERSION 3.15)
project(PositionTracking)

find_package(Eigen3 REQUIRED)

add_library(map Map.cpp)
target_link_libraries(map Eigen3::Eigen)

add_library(tof ToF.cpp)
target_link_libraries(tof map Eigen3::Eigen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(position_tracking main.cpp)
target_link_libraries(position_tracking map tof)

enable_testing()
add_executable(test_map map_tests.cpp)
target_link_libraries(test_map map)
add_test(NAME MapTests COMMAND test_map)

add_executable(test_tof tof_tests.cpp)
target_link_libraries(test_tof tof map Eigen3::Eigen)
add_test(NAME ToFTests COMMAND test_tof)