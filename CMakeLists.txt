cmake_minimum_required(VERSION 3.15)
project(PositionTracking)

include(FetchContent)

set(BUILD_TESTING OFF CACHE BOOL "Disable tests for fetched dependencies" FORCE)

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

include_directories(include)

add_library(map src/Map.cpp)
target_link_libraries(map Eigen3::Eigen)

add_library(tof src/ToF.cpp)
target_link_libraries(tof map Eigen3::Eigen)

add_library(drone src/Drone.cpp)
target_link_libraries(drone tof map Eigen3::Eigen)

add_library(particle src/Particle.cpp)
target_link_libraries(particle Eigen3::Eigen)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

add_executable(position_tracking src/main.cpp)
target_link_libraries(position_tracking map tof)

enable_testing()
add_executable(test_map test/map_tests.cpp)
target_link_libraries(test_map map)
add_test(NAME MapTests COMMAND test_map)

add_executable(test_tof test/tof_tests.cpp)
target_link_libraries(test_tof tof map Eigen3::Eigen)
add_test(NAME ToFTests COMMAND test_tof)

add_executable(test_drone test/drone_tests.cpp)
target_link_libraries(test_drone drone tof map Eigen3::Eigen)
add_test(NAME DroneTests COMMAND test_drone)

add_executable(test_particle test/particle_tests.cpp)
target_link_libraries(test_particle particle Eigen3::Eigen)
add_test(NAME ParticleTests COMMAND test_particle)