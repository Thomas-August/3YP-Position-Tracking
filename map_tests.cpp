#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>

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

void expect_near(float actual, float expected, float tolerance, const char* test_name) {
    ++tests_run;
    if (std::abs(actual - expected) > tolerance) {
        ++tests_failed;
        std::cerr << "[FAIL] " << test_name << " (expected " << expected << ", got " << actual << ")" << std::endl;
    } else {
        std::cout << "[PASS] " << test_name << std::endl;
    }
}

void expect_throws(auto func, const char* test_name) {
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
    // Create a simple 3x3 grid with no walls
    Eigen::MatrixXi grid(3, 3);
    grid.setZero();

    float gridSize = 1.0f;
    float mapHeight = 3.0f;
    float maxRayDist = 10.0f;

    Map map(grid, gridSize, mapHeight, maxRayDist);

    // Test 1: Basic horizontal raycast
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(1.0f, 0.0f, 0.0f);
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "basic horizontal raycast returns finite distance");
        expect_true(distance >= 0.0f, "basic horizontal raycast returns non-negative distance");
        expect_true(distance <= maxRayDist, "basic horizontal raycast respects max distance");
    }

    // Test 2: Zero direction vector should throw exception
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f zero_direction(0.0f, 0.0f, 0.0f);
        expect_throws([&]() { map.raycast(position, zero_direction); }, 
                      "zero direction vector throws exception");
    }

    // Test 3: Vertical raycast (floor collision)
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(0.0f, 0.0f, -1.0f);
        float distance = map.raycast(position, direction);
        expect_near(distance, 1.5f, 0.01f, "vertical raycast hits floor");
    }

    // Test 4: Vertical raycast (ceiling collision)
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(0.0f, 0.0f, 1.0f);
        float distance = map.raycast(position, direction);
        expect_near(distance, 1.5f, 0.01f, "vertical raycast hits ceiling");
    }

    // Test 5: Diagonal raycast in xy plane
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(1.0f, 1.0f, 0.0f);
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "diagonal xy raycast returns finite distance");
        expect_true(distance >= 0.0f, "diagonal xy raycast returns non-negative distance");
    }

    // Test 6: Raycast with all components
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(1.0f, 0.5f, 0.2f);
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "3D raycast returns finite distance");
        expect_true(distance >= 0.0f, "3D raycast returns non-negative distance");
        expect_true(distance <= maxRayDist, "3D raycast respects max distance");
    }

    // Test 7: Create a map with walls
    Eigen::MatrixXi grid_with_walls(3, 3);
    grid_with_walls.setZero();
    grid_with_walls(1, 1) = 2; // West wall in center cell
    Map map_with_walls(grid_with_walls, gridSize, mapHeight, maxRayDist);

    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(1.0f, 0.0f, 0.0f);
        float distance = map_with_walls.raycast(position, direction);
        expect_true(std::isfinite(distance), "raycast with walls returns finite distance");
        expect_true(distance >= 0.0f, "raycast with walls returns non-negative distance");
    }

    // Test 8: Negative direction components
    {
        Eigen::Vector3f position(1.5f, -1.5f, 1.5f);
        Eigen::Vector3f direction(-1.0f, 0.0f, 0.0f);
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "negative x direction returns finite distance");
        expect_true(distance >= 0.0f, "negative x direction returns non-negative distance");
    }

    {
        Eigen::Vector3f position(1.5f, -1.5f, 1.5f);
        Eigen::Vector3f direction(0.0f, -1.0f, 0.0f);
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "negative y direction returns finite distance");
        expect_true(distance >= 0.0f, "negative y direction returns non-negative distance");
    }

    // Test 9: Non-normalized direction vector (should be normalized internally)
    {
        Eigen::Vector3f position(0.5f, -0.5f, 1.5f);
        Eigen::Vector3f direction(5.0f, 0.0f, 0.0f); // Not normalized
        float distance = map.raycast(position, direction);
        expect_true(std::isfinite(distance), "non-normalized direction returns finite distance");
        expect_true(distance >= 0.0f, "non-normalized direction returns non-negative distance");
    }

    std::cout << "\n=================================\n";
    std::cout << "Tests run: " << tests_run << ", failed: " << tests_failed << std::endl;
    std::cout << "=================================\n";
    return tests_failed == 0 ? 0 : 1;
}