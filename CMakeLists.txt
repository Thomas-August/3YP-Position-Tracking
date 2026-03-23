cmake_minimum_required(VERSION 3.15)
project(PositionTracking)

include(FetchContent)

message(STATUS "Fetching Eigen 3.4.0 via FetchContent...")
FetchContent_Declare(
	eigen
	DOWNLOAD_EXTRACT_TIMESTAMP TRUE
	URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
)
FetchContent_MakeAvailable(eigen)

if(NOT TARGET Eigen3::Eigen)
	if(TARGET eigen)
		add_library(Eigen3::Eigen ALIAS eigen)
	else()
		message(FATAL_ERROR "Fetched Eigen, but Eigen3::Eigen target is unavailable.")
	endif()
endif()

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