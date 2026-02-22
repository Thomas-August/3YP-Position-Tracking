// Map.hpp
// A class to store the map and perform raycasting

#include <Eigen/Dense>

class Map { 
    public: 
        Map(const Eigen::MatrixXi& map, float mapGridSize, float mapHeight, float maxRayDistance);
            // A constructor to initialize the max ray distance, map and its grid size and height.  
        Eigen::MatrixXi& getMap();
            // A function to return a reference to the map matrix.
        float raycast(Eigen::Vector3f position, Eigen::Vector3f direction);
            // A function to perform raycasing on the map.
            // This fucntion projects a horizontal ray from the given position in the direction of the given direction projected into the xy plane,
            // checks for collisions with the map walls, then checks if the ray has exceeded the vertical bounds at that point, and returns the
            // distance to the collision point with the wall or the floor/ceiling or the max ray distance, whichever is closer.

    private:
        const Eigen::MatrixXi& map_;
            // A reference to the map matrix, where each cell contains an integer representing the type of cell 
            // (0 for no walls, 1 for top wall, 2 for left wall, and 3 for both walls)
            // (right and bottom walls are not represented in the map as they will be checked for using the adjacent cells).
        float mapGridSize_;
            // The size of each grid cell in the map, used to calculate distances in the raycasting algorithm.
        float mapHeight_;
            // The height of the map, used to check for floor and ceiling collisions in the raycasting algorithm.
        float maxRayDistance_;
            // The maximum distance the ray can measure

};