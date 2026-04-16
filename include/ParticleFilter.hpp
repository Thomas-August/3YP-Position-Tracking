// ParticleFilter.hpp
// A class to estimate drone position and yaw using a particle filter

#pragma once

#include <Eigen/Dense>

#include <cstdint>
#include <cstddef>
#include <random>
#include <vector>

#include "Particle.hpp"

class Drone;

struct ParticleFilterBounds {
    Eigen::Vector3f minPosition;
    Eigen::Vector3f maxPosition;
    float minYaw;
    float maxYaw;
};

class ParticleFilter {
    public:
        ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev);
        ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev, std::uint32_t seed);
        ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev,
                       float resamplePositionNoiseStdDev, float resampleYawNoiseStdDev);
        ParticleFilter(Drone& virtualDrone, std::size_t numParticles, float weightStdDev, std::uint32_t seed,
                       float resamplePositionNoiseStdDev, float resampleYawNoiseStdDev);

        void initializeUniform(const ParticleFilterBounds& bounds);
        void updateWeights(const Eigen::VectorXf& measurement);
        void normalizeWeights();
        void resample();

        Eigen::Vector4f estimatePose() const;

        const std::vector<Particle>& getParticles() const;
        void setParticles(const std::vector<Particle>& particles);

        float getWeightStdDev() const;
        void setWeightStdDev(float weightStdDev);

        float getResamplePositionNoiseStdDev() const;
        void setResamplePositionNoiseStdDev(float resamplePositionNoiseStdDev);

        float getResampleYawNoiseStdDev() const;
        void setResampleYawNoiseStdDev(float resampleYawNoiseStdDev);

    private:
        Eigen::VectorXf predictMeasurement(const Particle& particle);
        float computeUnnormalizedWeight(const Eigen::VectorXf& measurement,
                                        const Eigen::VectorXf& predictedMeasurement) const;

        Drone& virtualDrone_;
        std::vector<Particle> particles_;
        float weightStdDev_;
        float resamplePositionNoiseStdDev_;
        float resampleYawNoiseStdDev_;
        std::mt19937 rng_;
};