// Map.hpp
// A class to store the map and perform raycasting

#include <Eigen/Dense>

class Map { 
    public: 
        Map(const Eigen::MatrixXi& map, float gridSize);
        float raycast(Eigen::Vector3f position, Eigen::Vector3f direction);
        Eigen::MatrixXi& getMap();

    private:
        const Eigen::MatrixXi& map_;
        float gridSize_;

};