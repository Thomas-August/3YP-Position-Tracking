// Particle.cpp
// Implementation of a lightweight particle class for static position + yaw estimation

#include "Particle.hpp"

#include <cmath>
#include <stdexcept>

Particle::Particle() : position_(Eigen::Vector3f::Zero()), yaw_(0.0f), weight_(1.0f) {}

Particle::Particle(const Eigen::Vector3f& position, float yaw, float weight)
    : position_(position), yaw_(yaw), weight_(1.0f) {
    setYaw(yaw);
    setWeight(weight);
}

Eigen::Vector3f Particle::getPosition() const {
    return position_;
}

void Particle::setPosition(const Eigen::Vector3f& position) {
    position_ = position;
}

float Particle::getYaw() const {
    return yaw_;
}

void Particle::setYaw(float yaw) {
    if (!std::isfinite(yaw)) {
        throw std::invalid_argument("Particle yaw must be finite.");
    }
    yaw_ = yaw;
}

float Particle::getWeight() const {
    return weight_;
}

void Particle::setWeight(float weight) {
    if (!std::isfinite(weight) || weight < 0.0f) {
        throw std::invalid_argument("Particle weight must be finite and non-negative.");
    }
    weight_ = weight;
}