// Map.cpp
// Implementation of a class to store the map and perform raycasting

#include "Map.hpp"
#include <Eigen/Dense>
#include <cmath>
#include <iostream>

Map::Map(const Eigen::MatrixXi& map, float mapGridSize, float mapHeight) : map_(map), mapGridSize_(mapGridSize), mapHeight_(mapHeight) {}
    // A constructor to initialize the map and grid size.   

Eigen::MatrixXi& Map::getMap() {
    // A function to return a reference to the map matrix.
    return const_cast<Eigen::MatrixXi&>(map_);
} 

float Map::raycast(Eigen::Vector3f pos, Eigen::Vector3f dir, float maxRayDist) {
    // A function to perform raycasing on the map.
    // This fucntion projects a horizontal ray from the given position in the direction of the given direction projected into the xy plane,
    // checks for collisions with the map walls, then checks if the ray has exceeded the vertical bounds at that point, and returns the
    // distance to the collision point with the wall or the floor/ceiling or the max ray distance, whichever is closer.
    // Inputs:
    // - pos: the position of the ray origin in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z)
    // - dir: the direction of the ray in 3D space (x, y, z)
    // Output: the distance to the collision point with the wall or the floor/ceiling or the max ray distance, whichever is closer.

    // Throw an error if the direction vector is the zero vector
    if (dir.norm() == 0.0f) {
        throw std::invalid_argument("Direction vector cannot be the zero vector.");
    }

    // Ensure that the direction vector is a unit vector
    dir.normalize();

    // Check edge case where there is no component of the direcition vector in the xy plane
    if (dir.head<2>().norm() == 0.0f) {
        // If there is no component of the direction vector in the xy plane, then the ray is vertical and can only collide with the floor or ceiling.
        // In this case, we can directly check for floor and ceiling collisions without performing raycasting in the xy plane.

        // Calculate the distance to the floor and ceiling from the current position
        float distToFloor = pos.z() / -dir.z(); // Distance to floor
        float distToCeiling = (mapHeight_ - pos.z()) / dir.z(); // Distance to ceiling

        // Check for floor collision
        if (distToFloor > 0 && distToFloor < maxRayDist) {
            return distToFloor; // Collision with floor
        }

        // Check for ceiling collision
        if (distToCeiling > 0 && distToCeiling < maxRayDist) {
            return distToCeiling; // Collision with ceiling
        }

        return maxRayDist; // No collision within max ray distance
    }

    // Otherwise, if there is an xy component of the direction vector
    // Create a variable to keep track of the total distance traveled by the ray
    float totalDist = 0.0f;

    // Get the position within the current grid cell
    float cellPosX = std::fmod(pos.x(), mapGridSize_);
    float cellPosY = std::fmod(pos.y(), mapGridSize_);

    // Get the indices of the current grid cell in the map
    int cellIndexX = static_cast<int>(pos.x() / mapGridSize_);
    int cellIndexY = static_cast<int>(-pos.y() / mapGridSize_);

    // Create a variable to store the distance travelled within the current grid cell
    float cellDist = 0.0f;

    // Create a variable to count the number of iterations of the loop to prevent infinite loops in case of errors in the map or raycasting algorithm
    int iterations = 0;
    // Set a maximum number of iterations to 10 times the diagonal length of the map in grid cells, which should be more than enough for any ray to traverse the entire map without getting stuck in an infinite loop.
    const int maxIterations = std::sqrt(map_.rows() * map_.rows() + map_.cols() * map_.cols()) * 10; 

    // Create a flag to track if the ray has left the map boundaries
    bool rayExitedMap = false;

    // Loop until the ray exceeds the maximum ray distance
    while (totalDist < maxRayDist && iterations < maxIterations) {

        // Check which edge of the cell the ray intersects with and check if there is a wall at that edge
        // If there is a wall at that edge then return the total distance traveled by the ray
        // otherwise add the distance travelled in that cell to the total distance and
        // move the ray to the point it intersects with that edge and continue checking for collisions in the next cell

        // Increment the iteration counter to prevent infinite loops in case of errors in the map or raycasting algorithm
        iterations++;

        // Reset the cell distance variable for the next iteration of the loop
        cellDist = 0.0f;

        // Check which direction the ray is moving in the xy plane
        if (dir.x() > 0) {
            // If the ray has a component in the positive x direction

            // Ray distance to collision with east edge of current grid cell
            cellDist = (mapGridSize_ - cellPosX) / dir.x(); 
            // Check if the y position of the ray at the point it intersects with the east edge is within the bounds of the grid cell
            float cellIntersectY = cellPosY + cellDist * dir.y();
            if (cellIntersectY <= 0 && cellIntersectY > -mapGridSize_) {
                // Increse the total distance traveled by the ray
                totalDist += cellDist;
                // Check if the map cell for collision checking is out of bounds (ray is out of bounds)
                if (cellIndexX + 1 < 0 || cellIndexX + 1 >= map_.cols() || cellIndexY < 0 || cellIndexY >= map_.rows()) {
                    rayExitedMap = true;
                    // No boundaries outside map bounds continue ray until it reaches max ray distance
                    // Move the ray to the point it intersects with the east edge of the current grid cell
                    cellPosX = 0;
                    cellPosY = cellIntersectY;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the east edge
                    cellIndexX += 1;
                    continue; // Restart the loop to check for collisions in the next cell
                } else if (map_(cellIndexY, cellIndexX + 1) == 2 || map_(cellIndexY, cellIndexX + 1) == 3) {
                    break; // Collision with east wall so exit the loop
                } else {
                    // Move the ray to the point it intersects with the east edge of the current grid cell
                    cellPosX = 0;
                    cellPosY = cellIntersectY;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the east edge
                    cellIndexX += 1;
                    continue; // Restart the loop to check for collisions in the next cell
                }
            }
        } else if (dir.x() < 0) {
            // If the ray has a component in the negative x direction

            // Ray distance to collision with west edge of current grid cell
            cellDist = -cellPosX / dir.x(); 
            // Check if the y position of the ray at the point it intersects with the west edge is within the bounds of the grid cell
            float cellIntersectY = cellPosY + cellDist * dir.y();
            if (cellIntersectY <= 0 && cellIntersectY > -mapGridSize_) {
                // Increse the total distance traveled by the ray
                totalDist += cellDist;
                // Check if the map cell for collision checking is out of bounds (ray is out of bounds)
                if (cellIndexX < 0 || cellIndexX >= map_.cols() || cellIndexY < 0 || cellIndexY >= map_.rows()) {
                    rayExitedMap = true;
                    // No boundaries outside map bounds continue ray until it reaches max ray distance
                    // Move the ray to the point it intersects with the west edge of the current grid cell
                    cellPosX = mapGridSize_;
                    cellPosY = cellIntersectY;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the west edge
                    cellIndexX -= 1;
                    continue; // Restart the loop to check for collisions in the next cell
                } else if (map_(cellIndexY, cellIndexX) == 2 || map_(cellIndexY, cellIndexX) == 3) {
                    break; // Collision with west wall so exit the loop
                } else {
                    // Move the ray to the point it intersects with the west edge of the current grid cell
                    cellPosX = mapGridSize_;
                    cellPosY = cellIntersectY;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the west edge
                    cellIndexX -= 1;
                    continue; // Restart the loop to check for collisions in the next cell
                }
            }
        } 
        if (dir.y() > 0) {
            // If the ray has a component in the positive y direction

            // Ray distance to collision with north edge of current grid cell
            cellDist = -cellPosY / dir.y(); 
            // Check if the x position of the ray at the point it intersects with the north edge is within the bounds of the grid cell
            float cellIntersectX = cellPosX + cellDist * dir.x();
            if (cellIntersectX >= 0 && cellIntersectX < mapGridSize_) {
                // Increse the total distance traveled by the ray
                totalDist += cellDist;
                // Check if the map cell for collision checking is out of bounds (ray is out of bounds)
                if (cellIndexX < 0 || cellIndexX >= map_.cols() || cellIndexY < 0 || cellIndexY >= map_.rows()) {
                    rayExitedMap = true;
                    // No boundaries outside map bounds continue ray until it reaches max ray distance
                    // Move the ray to the point it intersects with the north edge of the current grid cell
                    cellPosX = cellIntersectX;
                    cellPosY = -mapGridSize_;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the north edge
                    cellIndexY -= 1;
                    continue; // Restart the loop to check for collisions in the next cell
                } else if (map_(cellIndexY, cellIndexX) == 1 || map_(cellIndexY, cellIndexX) == 3) {
                    break; // Collision with north wall so exit the loop
                } else {
                    // Move the ray to the point it intersects with the north edge of the current grid cell
                    cellPosX = cellIntersectX;
                    cellPosY = -mapGridSize_;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the north edge
                    cellIndexY -= 1;
                    continue; // Restart the loop to check for collisions in the next cell
                }
            }
        } else if (dir.y() < 0) {
            // If the ray has a component in the negative y direction

            // Ray distance to collision with south edge of current grid cell
            cellDist = -(mapGridSize_ + cellPosY) / dir.y(); 
            // Check if the x position of the ray at the point it intersects with the south edge is within the bounds of the grid cell
            float cellIntersectX = cellPosX + cellDist * dir.x();
            if (cellIntersectX >= 0 && cellIntersectX < mapGridSize_) {
                // Increse the total distance traveled by the ray
                totalDist += cellDist;
                // Check if the map cell for collision checking is out of bounds (ray is out of bounds)
                if (cellIndexX < 0 || cellIndexX >= map_.cols() || cellIndexY + 1 < 0 || cellIndexY + 1 >= map_.rows()) {
                    rayExitedMap = true;
                    // No boundaries outside map bounds continue ray until it reaches max ray distance
                    // Move the ray to the point it intersects with the south edge of the current grid cell
                    cellPosX = cellIntersectX;
                    cellPosY = 0;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the south edge
                    cellIndexY += 1;
                    continue; // Restart the loop to check for collisions in the next cell
                } else if (map_(cellIndexY + 1, cellIndexX) == 1 || map_(cellIndexY + 1, cellIndexX) == 3) {
                    break; // Collision with south wall so exit the loop
                } else {
                    // Move the ray to the point it intersects with the south edge of the current grid cell
                    cellPosX = cellIntersectX;
                    cellPosY = 0;
                    // Update the cell indices to reflect the new grid cell the ray is in after moving through the south edge
                    cellIndexY += 1;
                    continue; // Restart the loop to check for collisions in the next cell
                }
            }
        }

        // The above checks are comprehensive so if the code get to this point throw an error as it means there is a bug in the raycasting algorithm
        throw std::runtime_error("Error in raycasting algorithm: failed to determine which edge the ray intersects with.");
    
    }

    // Print warning if ray exited the map
    if (rayExitedMap) {
        std::cerr << "Warning: Ray exited map boundaries. Ray continued in unmapped space until max distance reached." << std::endl;
    }

    // If there is a z component to the ray check if the ray hit the floor or ceiling before hitting a wall
    if (dir.z() != 0.0f) {
        // Calculate the vertical position of the ray at the point it intersects with the wall
        float rayIntersectZ = pos.z() + totalDist * dir.z();
        // Check for floor collision
        if (rayIntersectZ < 0) {
            // Update ray dist to collision with floor
            totalDist = pos.z() / -dir.z(); 
        } else if (rayIntersectZ > mapHeight_) {
            // Update ray dist to collision with ceiling
            totalDist = (mapHeight_ - pos.z()) / dir.z(); 
        }
    }

    // totalDist now contains the distance to the collision point with a wall floor or ceiling

    // Check if the total distance traveled by the ray exceeds the maximum ray distance and return the appropriate value
    if (totalDist >= maxRayDist) {
        return maxRayDist; // No collision within max ray distance
    } else {
        return totalDist;
    }
}