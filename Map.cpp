// Map.cpp
// Implementation of a class to store the map and perform raycasting

#include "Map.hpp"
#include <Eigen/Dense>

Map::Map(const Eigen::MatrixXi& map, float gridSize) : map_(map), gridSize_(gridSize) {}

float Map::raycast(Eigen::Vector3f position, Eigen::Vector3f direction) {
    // TODO: Implement raycasting algorithm here
    return 0.0f;
}