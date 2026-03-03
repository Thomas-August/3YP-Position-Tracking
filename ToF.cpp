// ToF.cpp
// A class to simulate the Time of Flight (ToF) sensor

#include <Eigen/Dense>

ToF::ToF(Map &map, Eigen::Vector3f pos, Eigen::Vector4f ori, float maxRange, int arraySize, float fov) : map_(map), pos_(pos), maxRange_(maxRange), arraySize_(arraySize), fov_(fov) {
    // A constructor to initialize the ToF sensor with a reference to the map, its position and orientation, and parameters for the sensor setup.
        // Inputs:
        // - map: a reference to the Map object, used to perform raycasting for distance measurement.
        // - pos: the initial position of the ToF sensor in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z).
        // - ori: the initial orientation of the ToF sensor in 3D space as a quaternion (qx, qy, qz, qw).
        // - maxRange: the maximum range of the ToF sensor for distance measurement.
        // - arraySize: the size of the distance measurement array, which determines how many rays are cast within the field of view (FOV) of the sensor. Default is 4.     
        // - fov: the field of view (FOV) of the ToF sensor in degrees, which determines the angular spread of the rays cast for distance measurement. Default is 45 degrees.


    // Ensure that the quaternion is a unit vector
    if (ori.norm() == 0.0f) {
        throw std::invalid_argument("Quaternion orientation cannot be the zero vector.");
    }
    ori.normalize();
    ori_ = ori;

    // Generate the angles for the rays based on the field of view and array size
    rayDir_.resize(arraySize_*arraySize_, 3);
    float angleIncrementRad = fov_ / (arraySize_ - 1) * M_PI / 180.0f; // Calculate the angle increment in radians
    for (int i = 0; i < arraySize_; ++i) { // Loop through each row of rays
        for (int j = 0; j < arraySize_; ++j) { // Loop through each column of rays
            float angle_z = -fov_ / 2 + i * angleIncrementRad; // Calculate the vertical angle for the current ray
            float angle_y = -fov_ / 2 + j * angleIncrementRad; // Calculate the horizontal angle for the current ray
            Eigen::Vector3f dir = Eigen::Vector3f(1, tan(angle_y), tan(angle_z)); // assuming the sensor faces along the positive x-axis
            dir.normalize(); // Normalize the direction vector
            rayDir_.row(i * arraySize_ + j) = dir;
        }
    }

}

void ToF::setPose(Eigen::Vector3f pos, Eigen::Vector4f ori) {
    // A function to set the position and orientation of the ToF sensor.
    // Ensure that the quaternion is a unit vector
    if (ori.norm() == 0.0f) {
        throw std::invalid_argument("Quaternion orientation cannot be the zero vector.");
    }
    ori.normalize();
    pos_ = pos;
    ori_ = ori;
}

Eigen::Vector7f ToF::getPose() {
    // A function to get the position and orientation of the ToF sensor as a 7D vector (x, y, z, qx, qy, qz, qw).
    Eigen::Vector7f pose;
    pose << pos_, ori_;
    return pose;
}