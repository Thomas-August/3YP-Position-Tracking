#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "Map.hpp"

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

}  // namespace

int main() {
    Eigen::MatrixXi grid(3, 3);
    grid.setZero();

    Map map(grid, 1.0f);

    Eigen::Vector3f position(1.0f, 1.0f, 0.0f);
    Eigen::Vector3f direction(1.0f, 0.0f, 0.0f);

    float distance = map.raycast(position, direction);
    expect_true(std::isfinite(distance), "raycast returns finite distance");
    expect_true(distance >= 0.0f, "raycast returns non-negative distance");

    std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << std::endl;
    return tests_failed == 0 ? 0 : 1;
}