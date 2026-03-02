// ToF.hpp
// A class to simulate the Time of Flight (ToF) sensor

#include <Eigen/Dense>

class ToF {
    public:
        ToF(Map &map, Eigen::Vector3f pos, Eigen::Vector3f dir, float maxRange, int arraySize = 4, float fov = 45);
        // A constructor to initialize the ToF sensor with a reference to the map, its position and direction.
        void setPose(Eigen::Vector3f pos, Eigen::Vector4f dir);
        // A function to set the position and direction of the ToF sensor.
        Eigen::Vector7f getPose();
        // A function to get the position and orientation (quaternion) of the ToF sensor as a 7D vector (x, y, z, qx, qy, qz, qw).
        Eigen::MatrixXf readDistances();
        // A function to read the distance measurements from the ToF sensor.

    private:
        Map &map_;
        // A reference to the map object, used to perform raycasting for distance measurement.
        Eigen::Vector3f pos_;
        // The position of the ToF sensor in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z).
        Eigen::Vector4f dir_;
        // The orientation of the ToF sensor in 3D space as a quaternion (x, y, z, w).

}