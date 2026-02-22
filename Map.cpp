// Map.cpp
// Implementation of a class to store the map and perform raycasting

#include "Map.hpp"
#include <Eigen/Dense>

Map::Map(const Eigen::MatrixXi& map, float mapGridSize, float mapHeight, float maxRayDistance) : map_(map), mapGridSize_(mapGridSize), mapHeight_(mapHeight), maxRayDistance_(maxRayDistance) {}
    // A constructor to initialize the map and grid size.   

Eigen::MatrixXi& Map::getMap() {
    // A function to return a reference to the map matrix.
    return const_cast<Eigen::MatrixXi&>(map_);
} 

float Map::raycast(Eigen::Vector3f position, Eigen::Vector3f direction) {
    // A function to perform raycasing on the map.
    // This fucntion projects a horizontal ray from the given position in the direction of the given direction projected into the xy plane,
    // checks for collisions with the map walls, then checks if the ray has exceeded the vertical bounds at that point, and returns the
    // distance to the collision point with the wall or the floor/ceiling or the max ray distance, whichever is closer.

    // Ensure that the direction vector is a unit vector
    direction.normalize();

    // Get the xy component of the direction vector without normalizing so the distance can bet applied to the z component later to 
    // find height at the collision point and check for floor and ceiling collisions.
    Eigen::Vector2f directionXY = direction.head<2>();

    return 0.0f;
}