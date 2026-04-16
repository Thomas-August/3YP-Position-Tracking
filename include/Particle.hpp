// Particle.hpp
// A lightweight particle class for static position + yaw estimation

#pragma once

#include <Eigen/Dense>

class Particle {
    public:
        Particle();
        Particle(const Eigen::Vector3f& position, float yaw, float weight = 1.0f);

        Eigen::Vector3f getPosition() const;
        void setPosition(const Eigen::Vector3f& position);

        float getYaw() const;
        void setYaw(float yaw);

        float getWeight() const;
        void setWeight(float weight);

    private:
        Eigen::Vector3f position_;
        float yaw_;
        float weight_;
};