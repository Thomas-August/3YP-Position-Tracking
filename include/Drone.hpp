// Drone.hpp
// A class to simulate a drone with ToF sensors (TODO: and an IMU) 

#include <Eigen/Dense>
#include <vector>

class Map;  // Forward declaration
class ToF;  // Forward declaration

struct ToFSensor {
    ToF* sensor; // Pointer to the ToF sensor object
    Eigen::Isometry3f relativePose; // The pose of the sensor relative to the drone's body frame (x, y, z, roll, pitch, yaw)
};

class Drone {
    public:
        Drone(Map &map, Eigen::Isometry3f pose, std::vector<ToFSensor> tofSensors);
        // A constructor to initialize the drone with a reference to the map, its pose, and its ToF sensors.
        Eigen::VectorXf readSensors();
        // A function to read the distance measurements from all the ToF sensors on the drone and return them as a single vector.
        Eigen::Isometry3f getPose();
        // A function to get the current pose of the drone in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z, roll, pitch, yaw).
        void setPose(Eigen::Isometry3f pose);
        // A function to set the current pose of the drone in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z, roll, pitch, yaw).

    private:
        Map &map_;
        // A reference to the map object, used to perform raycasting for distance measurement.
        Eigen::Isometry3f pose_;
        // The pose of the drone in 3D space relative to the northwest corner on the floor, so x and z are positive and y is negative within the map (x, y, z, roll, pitch, yaw).
        std::vector<ToFSensor> tofSensors_;
        // A vector of ToF sensors mounted on the drone, each with its own relative pose to the drone's body frame.

};