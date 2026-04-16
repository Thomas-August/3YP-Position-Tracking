// ParticleFilter.cpp
// Stub implementation of a particle filter for position and yaw estimation

#include "Drone.hpp"
#include "ParticleFilter.hpp"

#include <cmath>
#include <stdexcept>

#include <Eigen/Geometry>

namespace {

constexpr float kDefaultResamplePositionNoiseStdDev = 0.02f;
constexpr float kDefaultResampleYawNoiseStdDev = 0.02f;

bool isValidBounds(const ParticleFilterBounds& bounds) {
    return bounds.minPosition.x() <= bounds.maxPosition.x() &&
           bounds.minPosition.y() <= bounds.maxPosition.y() &&
           bounds.minPosition.z() <= bounds.maxPosition.z() &&
           bounds.minYaw <= bounds.maxYaw;
}

float wrapYaw(float yaw) {
    return std::atan2(std::sin(yaw), std::cos(yaw));
}

Eigen::Isometry3f particleToPose(const Particle& particle) {
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = Eigen::AngleAxisf(particle.getYaw(), Eigen::Vector3f::UnitZ()).toRotationMatrix();
    pose.translation() = particle.getPosition();
    return pose;
}

}  // namespace

ParticleFilter::ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev)
    : ParticleFilter(virtualDrone, numParticles, weightStdDev, std::random_device{}(),
                     kDefaultResamplePositionNoiseStdDev, kDefaultResampleYawNoiseStdDev) {}

ParticleFilter::ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev, std::uint32_t seed)
    : ParticleFilter(virtualDrone, numParticles, weightStdDev, seed,
                     kDefaultResamplePositionNoiseStdDev, kDefaultResampleYawNoiseStdDev) {}

ParticleFilter::ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev,
                               float resamplePositionNoiseStdDev, float resampleYawNoiseStdDev)
    : ParticleFilter(virtualDrone, numParticles, weightStdDev, std::random_device{}(),
                     resamplePositionNoiseStdDev, resampleYawNoiseStdDev) {}

ParticleFilter::ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev, std::uint32_t seed,
                               float resamplePositionNoiseStdDev, float resampleYawNoiseStdDev)
    : virtualDrone_(virtualDrone),
      particles_(numParticles),
      weightStdDev_(0.0f),
      resamplePositionNoiseStdDev_(0.0f),
      resampleYawNoiseStdDev_(0.0f),
      rng_(seed) {
    setWeightStdDev(weightStdDev);
    setResamplePositionNoiseStdDev(resamplePositionNoiseStdDev);
    setResampleYawNoiseStdDev(resampleYawNoiseStdDev);
}

void ParticleFilter::initializeUniform(const ParticleFilterBounds& bounds) {
    if (particles_.empty()) {
        throw std::invalid_argument("Particle filter must contain at least one particle.");
    }

    if (!isValidBounds(bounds)) {
        throw std::invalid_argument("Particle filter bounds are invalid.");
    }

    std::uniform_real_distribution<float> xDist(bounds.minPosition.x(), bounds.maxPosition.x());
    std::uniform_real_distribution<float> yDist(bounds.minPosition.y(), bounds.maxPosition.y());
    std::uniform_real_distribution<float> zDist(bounds.minPosition.z(), bounds.maxPosition.z());
    std::uniform_real_distribution<float> yawDist(bounds.minYaw, bounds.maxYaw);

    const float uniformWeight = 1.0f / static_cast<float>(particles_.size());

    for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
        particle->setPosition(Eigen::Vector3f(xDist(rng_), yDist(rng_), zDist(rng_)));
        particle->setYaw(yawDist(rng_));
        particle->setWeight(uniformWeight);
    }
}

void ParticleFilter::updateWeights(const Eigen::VectorXf& measurement) {
    if (particles_.empty()) {
        return;
    }

    for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
        Eigen::VectorXf predictedMeasurement = predictMeasurement(*particle);
        particle->setWeight(computeUnnormalizedWeight(measurement, predictedMeasurement));
    }
}

void ParticleFilter::normalizeWeights() {
    if (particles_.empty()) {
        return;
    }

    float weightSum = 0.0f;
    for (std::vector<Particle>::const_iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
        const float weight = particle->getWeight();
        if (!std::isfinite(weight) || weight < 0.0f) {
            throw std::invalid_argument("Particle weights must be finite and non-negative.");
        }
        weightSum += weight;
    }

    const float uniformWeight = 1.0f / static_cast<float>(particles_.size());

    if (!std::isfinite(weightSum) || weightSum <= 0.0f) {
        for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
            particle->setWeight(uniformWeight);
        }
        return;
    }

    for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
        particle->setWeight(particle->getWeight() / weightSum);
    }
}

void ParticleFilter::resample() {
    if (particles_.empty()) {
        return;
    }

    float weightSum = 0.0f;
    for (std::vector<Particle>::const_iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
        const float weight = particle->getWeight();
        if (!std::isfinite(weight) || weight < 0.0f) {
            throw std::invalid_argument("Particle weights must be finite and non-negative.");
        }
        weightSum += weight;
    }

    const float uniformWeight = 1.0f / static_cast<float>(particles_.size());
    if (!std::isfinite(weightSum) || weightSum <= 0.0f) {
        for (std::vector<Particle>::iterator particle = particles_.begin(); particle != particles_.end(); ++particle) {
            particle->setWeight(uniformWeight);
        }
        return;
    }

    std::vector<Particle> resampledParticles;
    resampledParticles.reserve(particles_.size());

    std::uniform_real_distribution<float> startDist(0.0f, 1.0f / static_cast<float>(particles_.size()));
    std::normal_distribution<float> positionNoise(0.0f, resamplePositionNoiseStdDev_);
    std::normal_distribution<float> yawNoise(0.0f, resampleYawNoiseStdDev_);
    const float step = 1.0f / static_cast<float>(particles_.size());
    float target = startDist(rng_);

    std::size_t particleIndex = 0;
    float cumulativeWeight = particles_.front().getWeight() / weightSum;

    for (std::size_t i = 0; i < particles_.size(); ++i) {
        while (target > cumulativeWeight && particleIndex + 1 < particles_.size()) {
            ++particleIndex;
            cumulativeWeight += particles_[particleIndex].getWeight() / weightSum;
        }

        Particle selectedParticle = particles_[particleIndex];
        Eigen::Vector3f noisyPosition = selectedParticle.getPosition();
        noisyPosition.x() += positionNoise(rng_);
        noisyPosition.y() += positionNoise(rng_);
        noisyPosition.z() += positionNoise(rng_);
        selectedParticle.setPosition(noisyPosition);
        selectedParticle.setYaw(wrapYaw(selectedParticle.getYaw() + yawNoise(rng_)));
        selectedParticle.setWeight(uniformWeight);
        resampledParticles.push_back(selectedParticle);
        target += step;
    }

    particles_ = resampledParticles;
}

Eigen::Vector4f ParticleFilter::estimatePose() const {
    if (particles_.empty()) {
        return Eigen::Vector4f::Zero();
    }

    // Compute weighted average position
    Eigen::Vector3f avgPosition = Eigen::Vector3f::Zero();
    float avgSinYaw = 0.0f;
    float avgCosYaw = 0.0f;

    for (const auto& particle : particles_) {
        const float weight = particle.getWeight();
        avgPosition += weight * particle.getPosition();
        avgSinYaw += weight * std::sin(particle.getYaw());
        avgCosYaw += weight * std::cos(particle.getYaw());
    }

    // Compute circular mean for yaw angle
    float estimatedYaw = std::atan2(avgSinYaw, avgCosYaw);

    return Eigen::Vector4f(avgPosition.x(), avgPosition.y(), avgPosition.z(), estimatedYaw);
}

const std::vector<Particle>& ParticleFilter::getParticles() const {
    return particles_;
}

void ParticleFilter::setParticles(const std::vector<Particle>& particles) {
    particles_ = particles;
}

float ParticleFilter::getWeightStdDev() const {
    return weightStdDev_;
}

void ParticleFilter::setWeightStdDev(float weightStdDev) {
    if (!std::isfinite(weightStdDev) || weightStdDev <= 0.0f) {
        throw std::invalid_argument("Weight standard deviation must be finite and positive.");
    }
    weightStdDev_ = weightStdDev;
}

float ParticleFilter::getResamplePositionNoiseStdDev() const {
    return resamplePositionNoiseStdDev_;
}

void ParticleFilter::setResamplePositionNoiseStdDev(float resamplePositionNoiseStdDev) {
    if (!std::isfinite(resamplePositionNoiseStdDev) || resamplePositionNoiseStdDev < 0.0f) {
        throw std::invalid_argument("Resample position noise standard deviation must be finite and non-negative.");
    }
    resamplePositionNoiseStdDev_ = resamplePositionNoiseStdDev;
}

float ParticleFilter::getResampleYawNoiseStdDev() const {
    return resampleYawNoiseStdDev_;
}

void ParticleFilter::setResampleYawNoiseStdDev(float resampleYawNoiseStdDev) {
    if (!std::isfinite(resampleYawNoiseStdDev) || resampleYawNoiseStdDev < 0.0f) {
        throw std::invalid_argument("Resample yaw noise standard deviation must be finite and non-negative.");
    }
    resampleYawNoiseStdDev_ = resampleYawNoiseStdDev;
}

Eigen::VectorXf ParticleFilter::predictMeasurement(const Particle& particle) {
    const Eigen::Isometry3f originalPose = virtualDrone_.getPose();
    virtualDrone_.setPose(particleToPose(particle));

    const Eigen::VectorXf predictedMeasurement = virtualDrone_.readSensors();

    virtualDrone_.setPose(originalPose);

    return predictedMeasurement;
}

float ParticleFilter::computeUnnormalizedWeight(const Eigen::VectorXf& measurement,
                                                const Eigen::VectorXf& predictedMeasurement) const {
    if (measurement.size() != predictedMeasurement.size()) {
        throw std::invalid_argument("Measurement and prediction sizes must match.");
    }

    if (!std::isfinite(weightStdDev_) || weightStdDev_ <= 0.0f) {
        throw std::invalid_argument("Weight standard deviation must be finite and positive.");
    }

    const float squaredError = (measurement - predictedMeasurement).squaredNorm();
    const float variance = weightStdDev_ * weightStdDev_;
    return std::exp(-0.5f * squaredError / variance);
}