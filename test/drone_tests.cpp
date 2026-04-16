#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "Drone.hpp"
#include "Map.hpp"
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
        std::cerr << "[FAIL] " << test_name
                  << " (expected " << expected << ", got " << actual << ")" << std::endl;
    } else {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

void expect_equal(int actual, int expected, const char* test_name) {
    ++tests_run;
    if (actual != expected) {
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name
                  << " (expected " << expected << ", got " << actual << ")" << std::endl;
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
    Eigen::MatrixXi grid(5, 5);
    grid.setZero();

    Map map(grid, 1.0f, 3.0f);
    const float maxRange = 10.0f;

    const Eigen::Vector3f dronePos(2.0f, -2.0f, 1.0f);
    const Eigen::Quaternionf droneOri(1.0f, 0.0f, 0.0f, 0.0f);
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
    expect_true(true, "Drone constructor accepts valid sensors");

    Eigen::Isometry3f returnedPose = drone.getPose();
    expect_near(returnedPose.translation().x(), dronePos.x(), 1e-5f, "getPose returns correct x");
    expect_near(returnedPose.translation().y(), dronePos.y(), 1e-5f, "getPose returns correct y");
    expect_near(returnedPose.translation().z(), dronePos.z(), 1e-5f, "getPose returns correct z");

    const Eigen::Vector3f newPos(1.0f, -1.0f, 2.0f);
    const Eigen::Isometry3f newPose = makePose(newPos, droneOri);
    drone.setPose(newPose);
    returnedPose = drone.getPose();
    expect_near(returnedPose.translation().x(), newPos.x(), 1e-5f, "setPose updates x");
    expect_near(returnedPose.translation().y(), newPos.y(), 1e-5f, "setPose updates y");
    expect_near(returnedPose.translation().z(), newPos.z(), 1e-5f, "setPose updates z");

    Eigen::VectorXf readings = drone.readSensors();
    expect_equal(readings.size(), 32, "readSensors returns 32 readings for 2 sensors");

    const Eigen::Vector3f expectedSensor1Pos = newPos;
    const Eigen::Vector3f expectedSensor2Pos = newPos + Eigen::Vector3f(0.5f, 0.0f, 0.0f);

    expect_near(sensor1.getPos().x(), expectedSensor1Pos.x(), 1e-5f, "sensor1 world x updated");
    expect_near(sensor1.getPos().y(), expectedSensor1Pos.y(), 1e-5f, "sensor1 world y updated");
    expect_near(sensor1.getPos().z(), expectedSensor1Pos.z(), 1e-5f, "sensor1 world z updated");

    expect_near(sensor2.getPos().x(), expectedSensor2Pos.x(), 1e-5f, "sensor2 world x updated");
    expect_near(sensor2.getPos().y(), expectedSensor2Pos.y(), 1e-5f, "sensor2 world y updated");
    expect_near(sensor2.getPos().z(), expectedSensor2Pos.z(), 1e-5f, "sensor2 world z updated");

    bool allValid = true;
    for (int i = 0; i < readings.size(); ++i) {
        if (!std::isfinite(readings(i)) || readings(i) < 0.0f || readings(i) > maxRange + 1e-5f) {
            allValid = false;
            break;
        }
    }
    expect_true(allValid, "readSensors returns valid distances");

    ToFSensor badSensor;
    badSensor.sensor = nullptr;
    badSensor.relativePose = Eigen::Isometry3f::Identity();

    expect_throws([&]() {
        std::vector<ToFSensor> invalidSensors{badSensor};
        Drone badDrone(map, dronePose, invalidSensors);
        (void)badDrone;
    }, "Drone constructor throws for null sensor pointer");

    std::cout << "\n=================================\n";
    std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << std::endl;
    std::cout << "=================================\n";

    return tests_failed == 0 ? 0 : 1;
}