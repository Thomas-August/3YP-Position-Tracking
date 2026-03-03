// ToF.hpp
// A class to simulate the Time of Flight (ToF) sensor

#include <Eigen/Dense>

class ToF {
    public:
        ToF(Map &map, Eigen::Vector3f pos, Eigen::Vector4f ori, float maxRange, int arraySize = 4, float fov = 45);
        // A constructor to initialize the ToF sensor with a reference to the map, its position and orientation, and parameters for the sensor setup.
        // Inputs:
        // - map: a reference to the Map object, used to perform raycasting for distance measurement.
        // - pos: the initial position of the ToF sensor in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z).
        // - ori: the initial orientation of the ToF sensor in 3D space as a quaternion (qx, qy, qz, qw).
        // - maxRange: the maximum range of the ToF sensor for distance measurement.
        // - arraySize: the size of the distance measurement array, which determines how many rays are cast within the field of view (FOV) of the sensor. Default is 4.     
        // - fov: the field of view (FOV) of the ToF sensor in degrees, which determines the angular spread of the rays cast for distance measurement. Default is 45 degrees.

        void setPose(Eigen::Vector3f pos, Eigen::Vector4f ori);
        // A function to set the position and orientation of the ToF sensor.
        Eigen::Vector7f getPose();
        // A function to get the position and orientation of the ToF sensor as a 7D vector (x, y, z, qx, qy, qz, qw).
        Eigen::VectorXf readDistances();
        // A function to read the distance measurements from the ToF sensor. 
        // Returns a vector of size arraySize^2 containing the distance measurements for each raycast starting
        // from the top left ray and going across the rows and then down the columns of the raycast grid.

    private:
        Map &map_;
        // A reference to the map object, used to perform raycasting for distance measurement.
        Eigen::Vector3f pos_;
        // The position of the ToF sensor in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z).
        Eigen::Vector4f ori_;
        // The orientation of the ToF sensor in 3D space as a quaternion (qx, qy, qz, qw).
        float maxRange_;
        // The maximum range of the ToF sensor for distance measurement.
        int arraySize_;
        // The size of the distance measurement array, which determines how many rays are cast within the field of view (FOV) of the sensor.
        float fov_;
        // The field of view (FOV) of the ToF sensor in degrees, which determines the angular spread of the rays cast for distance measurement.
        Eigen::MatrixXf rayDir_;
        // A matrix of size arraySize^2 x 3 containing the direction vectors for each raycast in the sensor's field of view, relative to the sensor's orientation.

    }