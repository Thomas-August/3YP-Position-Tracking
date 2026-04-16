#include <Eigen/Dense>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "Particle.hpp"

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

}  // namespace

int main() {
    Particle p_default;
    expect_near(p_default.getPosition().x(), 0.0f, 1e-6f, "Default particle x is 0");
    expect_near(p_default.getPosition().y(), 0.0f, 1e-6f, "Default particle y is 0");
    expect_near(p_default.getPosition().z(), 0.0f, 1e-6f, "Default particle z is 0");
    expect_near(p_default.getYaw(), 0.0f, 1e-6f, "Default particle yaw is 0");
    expect_near(p_default.getWeight(), 1.0f, 1e-6f, "Default particle weight is 1");

    Particle p_custom(Eigen::Vector3f(1.0f, -2.0f, 3.5f), 1.2f, 0.25f);
    expect_near(p_custom.getPosition().x(), 1.0f, 1e-6f, "Custom particle x");
    expect_near(p_custom.getPosition().y(), -2.0f, 1e-6f, "Custom particle y");
    expect_near(p_custom.getPosition().z(), 3.5f, 1e-6f, "Custom particle z");
    expect_near(p_custom.getYaw(), 1.2f, 1e-6f, "Custom particle yaw");
    expect_near(p_custom.getWeight(), 0.25f, 1e-6f, "Custom particle weight");

    p_custom.setPosition(Eigen::Vector3f(4.0f, -1.0f, 2.0f));
    expect_near(p_custom.getPosition().x(), 4.0f, 1e-6f, "setPosition updates x");
    expect_near(p_custom.getPosition().y(), -1.0f, 1e-6f, "setPosition updates y");
    expect_near(p_custom.getPosition().z(), 2.0f, 1e-6f, "setPosition updates z");

    p_custom.setYaw(-0.75f);
    expect_near(p_custom.getYaw(), -0.75f, 1e-6f, "setYaw updates yaw");

    p_custom.setWeight(0.75f);
    expect_near(p_custom.getWeight(), 0.75f, 1e-6f, "setWeight updates weight");

    expect_throws([&]() { p_custom.setYaw(std::numeric_limits<float>::infinity()); },
                  "setYaw throws on non-finite value");
    expect_throws([&]() { p_custom.setWeight(-0.1f); }, "setWeight throws on negative value");

    std::cout << "\n=================================\n";
    std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << std::endl;
    std::cout << "=================================\n";

    return tests_failed == 0 ? 0 : 1;
}