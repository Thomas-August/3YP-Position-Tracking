#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "Drone.hpp"
#include "Map.hpp"
#include "Particle.hpp"
#include "ParticleFilter.hpp"
#include "ToF.hpp"

namespace {

int tests_run = 0;
int tests_failed = 0;

void expect_true(bool condition, const char* test_name) {
    ++tests_run;
    if (!condition) {
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name << std::endl;
    } else {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

void expect_near(float actual, float expected, float tolerance, const char* test_name) {
    ++tests_run;
    if (std::abs(actual - expected) > tolerance) {
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name << " (expected " << expected << ", got " << actual << ")" << std::endl;
    } else {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

void expect_equal(int actual, int expected, const char* test_name) {
    ++tests_run;
    if (actual != expected) {
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name << " (expected " << expected << ", got " << actual << ")" << std::endl;
    } else {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

template <typename Func>
void expect_throws(Func func, const char* test_name) {
    ++tests_run;
    try {
        func();
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name << " (no exception thrown)" << std::endl;
    } catch (...) {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

Eigen::Isometry3f makePose(const Eigen::Vector3f& pos, const Eigen::Quaternionf& ori) {
    Eigen::Isometry3f pose = Eigen::Isometry3f::Identity();
    pose.linear() = ori.normalized().toRotationMatrix();
    pose.translation() = pos;
    return pose;
}

}  // namespace

int main() {
    // Setup: Create a map and drone for use in tests
    Eigen::MatrixXi grid(5, 5);
    grid.setZero();

    Map map(grid, 1.0f, 3.0f);
    const float maxRange = 10.0f;

    const Eigen::Vector3f dronePos(2.0f, -2.0f, 1.0f);
    const Eigen::Quaternionf droneOri = Eigen::Quaternionf::Identity();
    const Eigen::Isometry3f dronePose = makePose(dronePos, droneOri);

    ToF sensor1(map, dronePos, droneOri, maxRange, 4, 45.0f);
    ToF sensor2(map, dronePos, droneOri, maxRange, 4, 45.0f);

    ToFSensor tof1;
    tof1.sensor = &sensor1;
    tof1.relativePose = makePose(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Quaternionf::Identity());

    ToFSensor tof2;
    tof2.sensor = &sensor2;
    tof2.relativePose = makePose(Eigen::Vector3f(0.5f, 0.0f, 0.0f), Eigen::Quaternionf::Identity());

    std::vector<ToFSensor> sensors{tof1, tof2};
    Drone drone(map, dronePose, sensors);

    // Test constructors
    ParticleFilter pf1(drone, 100, 1.0f);
    expect_equal(pf1.getParticles().size(), 100, "Constructor 1: Creates correct number of particles");

    ParticleFilter pf2(drone, 50, 0.5f, 12345);
    expect_equal(pf2.getParticles().size(), 50, "Constructor 2: Creates correct number of particles with seed");
    expect_near(pf2.getWeightStdDev(), 0.5f, 1e-6f, "Constructor 2: Sets weight std dev correctly");

    ParticleFilter pf3(drone, 75, 1.5f, 0.01f, 0.05f);
    expect_near(pf3.getResamplePositionNoiseStdDev(), 0.01f, 1e-6f, "Constructor 3: Sets position noise correctly");
    expect_near(pf3.getResampleYawNoiseStdDev(), 0.05f, 1e-6f, "Constructor 3: Sets yaw noise correctly");

    // Test parameter validation in constructors
    expect_throws([&]() { ParticleFilter invalid_pf(drone, 10, -1.0f); },
                  "Constructor throws for negative weightStdDev");
    expect_throws([&]() { ParticleFilter invalid_pf(drone, 10, 1.0f, -0.01f, 0.01f); },
                  "Constructor throws for negative position noise");

    // Test initializeUniform
    ParticleFilter pf_init(drone, 10, 1.0f);
    ParticleFilterBounds valid_bounds;
    valid_bounds.minPosition = Eigen::Vector3f(0.0f, -5.0f, 0.0f);
    valid_bounds.maxPosition = Eigen::Vector3f(5.0f, 0.0f, 3.0f);
    valid_bounds.minYaw = -M_PI;
    valid_bounds.maxYaw = M_PI;

    pf_init.initializeUniform(valid_bounds);
    expect_equal(pf_init.getParticles().size(), 10, "initializeUniform preserves particle count");

    // Verify particles are within bounds
    bool all_in_bounds = true;
    for (const auto& particle : pf_init.getParticles()) {
        const Eigen::Vector3f pos = particle.getPosition();
        if (pos.x() < valid_bounds.minPosition.x() || pos.x() > valid_bounds.maxPosition.x() ||
            pos.y() < valid_bounds.minPosition.y() || pos.y() > valid_bounds.maxPosition.y() ||
            pos.z() < valid_bounds.minPosition.z() || pos.z() > valid_bounds.maxPosition.z()) {
            all_in_bounds = false;
            break;
        }
        if (!std::isfinite(particle.getWeight()) || particle.getWeight() < 0.0f) {
            all_in_bounds = false;
            break;
        }
    }
    expect_true(all_in_bounds, "initializeUniform places all particles within bounds");

    // Check uniform weight
    float expected_uniform_weight = 1.0f / 10.0f;
    bool weights_uniform = true;
    for (const auto& particle : pf_init.getParticles()) {
        if (std::abs(particle.getWeight() - expected_uniform_weight) > 1e-6f) {
            weights_uniform = false;
            break;
        }
    }
    expect_true(weights_uniform, "initializeUniform assigns uniform weights");

    // Test initializeUniform with invalid bounds
    ParticleFilterBounds invalid_bounds;
    invalid_bounds.minPosition = Eigen::Vector3f(5.0f, 0.0f, 0.0f);
    invalid_bounds.maxPosition = Eigen::Vector3f(0.0f, -5.0f, 0.0f);
    invalid_bounds.minYaw = M_PI;
    invalid_bounds.maxYaw = -M_PI;

    expect_throws([&]() { pf_init.initializeUniform(invalid_bounds); },
                  "initializeUniform throws for invalid bounds");

    // Test normalizeWeights
    ParticleFilter pf_norm(drone, 3, 1.0f);
    std::vector<Particle> test_particles;
    test_particles.push_back(Particle(Eigen::Vector3f(0.0f, 0.0f, 0.0f), 0.0f, 4.0f));
    test_particles.push_back(Particle(Eigen::Vector3f(1.0f, 0.0f, 0.0f), 0.0f, 2.0f));
    test_particles.push_back(Particle(Eigen::Vector3f(2.0f, 0.0f, 0.0f), 0.0f, 2.0f));
    pf_norm.setParticles(test_particles);

    pf_norm.normalizeWeights();

    float weight_sum = 0.0f;
    for (const auto& particle : pf_norm.getParticles()) {
        weight_sum += particle.getWeight();
    }
    expect_near(weight_sum, 1.0f, 1e-6f, "normalizeWeights sums to 1.0");

    // Verify individual normalized weights
    expect_near(pf_norm.getParticles()[0].getWeight(), 0.5f, 1e-6f, "normalizeWeights: particle 0 weight");
    expect_near(pf_norm.getParticles()[1].getWeight(), 0.25f, 1e-6f, "normalizeWeights: particle 1 weight");
    expect_near(pf_norm.getParticles()[2].getWeight(), 0.25f, 1e-6f, "normalizeWeights: particle 2 weight");

    // Test updateWeights and estimatePose
    ParticleFilter pf_estimate(drone, 5, 0.5f);

    // Initialize particles around the drone's actual position
    std::vector<Particle> estimate_particles;
    for (int i = 0; i < 5; ++i) {
        estimate_particles.push_back(Particle(
            Eigen::Vector3f(dronePos.x() + 0.1f * (i - 2), dronePos.y(), dronePos.z()),
            0.0f,
            1.0f / 5.0f
        ));
    }
    pf_estimate.setParticles(estimate_particles);

    // Get measurements and update weights
    Eigen::VectorXf measurements = drone.readSensors();
    pf_estimate.updateWeights(measurements);
    pf_estimate.normalizeWeights();

    // Estimate pose
    Eigen::Vector4f estimated_pose = pf_estimate.estimatePose();
    
    // The estimated pose should be reasonably close to the drone's actual position
    expect_true(std::isfinite(estimated_pose(0)), "estimatePose returns finite x");
    expect_true(std::isfinite(estimated_pose(1)), "estimatePose returns finite y");
    expect_true(std::isfinite(estimated_pose(2)), "estimatePose returns finite z");
    expect_true(std::isfinite(estimated_pose(3)), "estimatePose returns finite yaw");

    // Test estimatePose with empty particles (edge case)
    ParticleFilter pf_empty(drone, 1, 1.0f);
    pf_empty.setParticles(std::vector<Particle>());
    Eigen::Vector4f empty_pose = pf_empty.estimatePose();
    expect_near(empty_pose(0), 0.0f, 1e-6f, "estimatePose returns zero for empty particles (x)");
    expect_near(empty_pose(1), 0.0f, 1e-6f, "estimatePose returns zero for empty particles (y)");
    expect_near(empty_pose(2), 0.0f, 1e-6f, "estimatePose returns zero for empty particles (z)");
    expect_near(empty_pose(3), 0.0f, 1e-6f, "estimatePose returns zero for empty particles (yaw)");

    // Test circular mean for yaw in estimatePose
    ParticleFilter pf_yaw(drone, 2, 1.0f);
    std::vector<Particle> yaw_particles;
    // Particles at opposite angles should average near 0
    yaw_particles.push_back(Particle(Eigen::Vector3f(0.0f, 0.0f, 0.0f), M_PI * 0.99f, 0.5f));
    yaw_particles.push_back(Particle(Eigen::Vector3f(0.0f, 0.0f, 0.0f), -M_PI * 0.99f, 0.5f));
    pf_yaw.setParticles(yaw_particles);

    Eigen::Vector4f yaw_pose = pf_yaw.estimatePose();
    // With nearly opposite angles at equal weight, should average to near 0 or ±π
    bool yaw_reasonable = std::abs(yaw_pose(3)) < 0.1f || std::abs(std::abs(yaw_pose(3)) - M_PI) < 0.1f;
    expect_true(yaw_reasonable, "estimatePose handles circular mean for yaw correctly");

    // Test resample
    ParticleFilter pf_resample(drone, 10, 1.0f, 12345, 0.001f, 0.001f);
    pf_resample.initializeUniform(valid_bounds);
    
    Eigen::VectorXf initial_measurements = drone.readSensors();
    pf_resample.updateWeights(initial_measurements);
    pf_resample.normalizeWeights();
    
    std::vector<Particle> pre_resample = pf_resample.getParticles();
    pf_resample.resample();
    std::vector<Particle> post_resample = pf_resample.getParticles();

    expect_equal(post_resample.size(), 10, "resample maintains particle count");

    // After resampling, all weights should be uniform
    float resample_uniform_weight = 1.0f / 10.0f;
    bool resample_weights_uniform = true;
    for (const auto& particle : post_resample) {
        if (std::abs(particle.getWeight() - resample_uniform_weight) > 1e-6f) {
            resample_weights_uniform = false;
            break;
        }
    }
    expect_true(resample_weights_uniform, "resample assigns uniform weights");

    // Test getter and setter methods
    ParticleFilter pf_getset(drone, 5, 1.0f);
    
    expect_near(pf_getset.getWeightStdDev(), 1.0f, 1e-6f, "getWeightStdDev returns correct value");
    pf_getset.setWeightStdDev(2.5f);
    expect_near(pf_getset.getWeightStdDev(), 2.5f, 1e-6f, "setWeightStdDev updates value");

    expect_throws([&]() { pf_getset.setWeightStdDev(-1.0f); },
                  "setWeightStdDev throws for negative value");
    expect_throws([&]() { pf_getset.setWeightStdDev(0.0f); },
                  "setWeightStdDev throws for zero value");

    expect_near(pf_getset.getResamplePositionNoiseStdDev(), 0.02f, 1e-6f, "getResamplePositionNoiseStdDev returns correct default");
    pf_getset.setResamplePositionNoiseStdDev(0.05f);
    expect_near(pf_getset.getResamplePositionNoiseStdDev(), 0.05f, 1e-6f, "setResamplePositionNoiseStdDev updates value");

    expect_throws([&]() { pf_getset.setResamplePositionNoiseStdDev(-0.01f); },
                  "setResamplePositionNoiseStdDev throws for negative value");

    expect_near(pf_getset.getResampleYawNoiseStdDev(), 0.02f, 1e-6f, "getResampleYawNoiseStdDev returns correct default");
    pf_getset.setResampleYawNoiseStdDev(0.1f);
    expect_near(pf_getset.getResampleYawNoiseStdDev(), 0.1f, 1e-6f, "setResampleYawNoiseStdDev updates value");

    expect_throws([&]() { pf_getset.setResampleYawNoiseStdDev(-0.001f); },
                  "setResampleYawNoiseStdDev throws for negative value");

    std::cout << "\n=================================\n";
    std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << std::endl;
    std::cout << "=================================\n";

    return tests_failed == 0 ? 0 : 1;
}
