// Drone.cpp
// Implementation of a class to simulate a drone with ToF sensors

#include "Drone.hpp"
#include "ToF.hpp"

#include <stdexcept>
#include <utility>

Drone::Drone(Map &map, Eigen::Isometry3f pose, std::vector<ToFSensor> tofSensors)
    : map_(map), pose_(pose), tofSensors_(std::move(tofSensors)) {
    for (const auto &tofSensor : tofSensors_) {
        if (tofSensor.sensor == nullptr) {
            throw std::invalid_argument("ToF sensor pointer cannot be null.");
        }
    }
}

Eigen::VectorXf Drone::readSensors() {
    std::vector<Eigen::VectorXf> sensorReadings;
    sensorReadings.reserve(tofSensors_.size());

    std::size_t totalSize = 0;
    for (auto &tofSensor : tofSensors_) {
        if (tofSensor.sensor == nullptr) {
            throw std::invalid_argument("ToF sensor pointer cannot be null.");
        }

        const Eigen::Isometry3f sensorPose = pose_ * tofSensor.relativePose;
        tofSensor.sensor->setPose(sensorPose.translation(), Eigen::Quaternionf(sensorPose.rotation()));

        Eigen::VectorXf distances = tofSensor.sensor->readDistances();
        totalSize += static_cast<std::size_t>(distances.size());
        sensorReadings.push_back(std::move(distances));
    }

    Eigen::VectorXf readings(static_cast<int>(totalSize));
    int offset = 0;
    for (const auto &distances : sensorReadings) {
        readings.segment(offset, distances.size()) = distances;
        offset += distances.size();
    }

    return readings;
}

Eigen::Isometry3f Drone::getPose() {
    return pose_;
}

void Drone::setPose(Eigen::Isometry3f pose) {
    pose_ = pose;
}