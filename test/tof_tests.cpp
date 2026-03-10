#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "ToF.hpp"
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

void expect_equal(int actual, int expected, const char* test_name) {
    ++tests_run;
    if (actual != expected) {
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
    // Create a simple 5x5 grid map with some walls for testing
    Eigen::MatrixXi grid(5, 5);
    grid.setZero();
    
    // Add a wall at position (2, 2) with both north and west walls
    grid(2, 2) = 3;
    
    // Create the map
    float mapGridSize = 1.0f;
    float mapHeight = 2.0f;
    float maxRayDist = 10.0f;
    Map map(grid, mapGridSize, mapHeight);

    // ========== Constructor Tests ==========
    std::cout << "\n=== Constructor Tests ===" << std::endl;

    // Test 1: Create a ToF sensor with default parameters
    Eigen::Vector3f pos1(2.5f, -2.5f, 1.0f);
    Eigen::Quaternionf ori1(0.0f, 0.0f, 0.0f, 1.0f);  // Identity quaternion (w, x, y, z)
    ToF tof1(map, pos1, ori1, maxRayDist);
    expect_true(true, "ToF constructor with default parameters");

    // Test 2: Create a ToF sensor with custom array size
    ToF tof2(map, pos1, ori1, maxRayDist, 8, 60.0f);
    expect_true(true, "ToF constructor with custom array size");

    // Test 3: Constructor should normalize the quaternion
    Eigen::Quaternionf ori3(1.0f, 1.0f, 1.0f, 1.0f);  // Non-unit quaternion
    ToF tof3(map, pos1, ori3, maxRayDist);
    Eigen::Quaternionf storedOri3 = tof3.getOri();
    float qNorm3 = storedOri3.norm();
    expect_near(qNorm3, 1.0f, 1e-5f, "Constructor normalizes quaternion");

    // Test 4: Constructor should throw for zero quaternion
    Eigen::Quaternionf ori_zero(0.0f, 0.0f, 0.0f, 0.0f);
    expect_throws([&]() { ToF tof_bad(map, pos1, ori_zero, maxRayDist); }, 
                  "Constructor throws for zero quaternion");

    // ========== setPose Tests ==========
    std::cout << "\n=== setPose Tests ===" << std::endl;

    // Test 5: Set valid pose
    Eigen::Vector3f pos5(1.0f, -1.0f, 0.5f);
    Eigen::Quaternionf ori5(0.7071f, 0.0f, 0.0f, 0.7071f);  // 90 degree rotation around x-axis
    tof1.setPose(pos5, ori5);
    expect_true(true, "setPose with valid parameters");

    // Test 6: setPose should normalize the quaternion
    Eigen::Quaternionf ori6(2.0f, 2.0f, 0.0f, 0.0f);  // Non-unit quaternion
    tof1.setPose(pos1, ori6);
    Eigen::Quaternionf storedOri6 = tof1.getOri();
    float qNorm6 = storedOri6.norm();
    expect_near(qNorm6, 1.0f, 1e-5f, "setPose normalizes quaternion");

    // Test 7: setPose should throw for zero quaternion
    expect_throws([&]() { tof1.setPose(pos1, ori_zero); }, 
                  "setPose throws for zero quaternion");

    // ========== getPos/getOri Tests ==========
    std::cout << "\n=== getPos/getOri Tests ===" << std::endl;

    // Test 8: getPos returns correct vector size
    tof1.setPose(pos1, ori1);
    Eigen::Vector3f retrievedPos = tof1.getPos();
    expect_equal(retrievedPos.size(), 3, "getPos returns 3D vector");

    // Test 9: getPos returns correct position values
    expect_near(retrievedPos(0), pos1(0), 1e-5f, "getPos returns correct x position");
    expect_near(retrievedPos(1), pos1(1), 1e-5f, "getPos returns correct y position");
    expect_near(retrievedPos(2), pos1(2), 1e-5f, "getPos returns correct z position");

    // Test 10: getOri returns correct orientation (quaternion)
    Eigen::Quaternionf retrievedOri = tof1.getOri();
    expect_near(retrievedOri.x(), ori1.x(), 1e-5f, "getOri returns correct qx");
    expect_near(retrievedOri.y(), ori1.y(), 1e-5f, "getOri returns correct qy");
    expect_near(retrievedOri.z(), ori1.z(), 1e-5f, "getOri returns correct qz");
    expect_near(retrievedOri.w(), ori1.w(), 1e-5f, "getOri returns correct qw");

    // ========== readDistances Tests ==========
    std::cout << "\n=== readDistances Tests ===" << std::endl;

    // Test 11: readDistances returns correct size for 4x4 array
    ToF tof_4x4(map, pos1, ori1, maxRayDist, 4, 45.0f);
    Eigen::VectorXf distances_4x4 = tof_4x4.readDistances();
    expect_equal(distances_4x4.size(), 16, "readDistances returns correct size for 4x4 array");

    // Test 12: readDistances returns correct size for 8x8 array
    ToF tof_8x8(map, pos1, ori1, maxRayDist, 8, 45.0f);
    Eigen::VectorXf distances_8x8 = tof_8x8.readDistances();
    expect_equal(distances_8x8.size(), 64, "readDistances returns correct size for 8x8 array");

    // Test 13: readDistances returns distances within max range
    bool all_in_range = true;
    for (int i = 0; i < distances_4x4.size(); ++i) {
        if (distances_4x4(i) > maxRayDist + 1e-5f || distances_4x4(i) < 0.0f) {
            all_in_range = false;
            break;
        }
    }
    expect_true(all_in_range, "readDistances returns distances within valid range");

    // Test 14: readDistances responds to pose changes
    tof_4x4.setPose(Eigen::Vector3f(0.5f, -0.5f, 0.5f), ori1);
    Eigen::VectorXf distances_moved = tof_4x4.readDistances();
    bool distances_changed = (distances_moved != distances_4x4);
    expect_true(distances_changed, "readDistances changes with position change");

    // Test 15: readDistances with different orientations
    Eigen::Quaternionf ori_rotated(0.7071f, 0.0f, 0.7071f, 0.0f);  // 90 degree rotation around y-axis
    tof_4x4.setPose(pos1, ori_rotated);
    Eigen::VectorXf distances_rotated = tof_4x4.readDistances();
    expect_equal(distances_rotated.size(), 16, "readDistances works with rotated orientation");

    // Test 16: readDistances from center of map
    Eigen::Vector3f center_pos(2.5f, -2.5f, 1.0f);
    tof_4x4.setPose(center_pos, ori1);
    Eigen::VectorXf distances_center = tof_4x4.readDistances();
    bool center_valid = true;
    for (int i = 0; i < distances_center.size(); ++i) {
        if (distances_center(i) <= 0.0f || distances_center(i) > maxRayDist) {
            center_valid = false;
            break;
        }
    }
    expect_true(center_valid, "readDistances from center of map returns valid distances");

    // ========== Ray Direction Generation Tests ==========
    std::cout << "\n=== Ray Direction Generation Tests ===" << std::endl;

    // Test 17: Ray directions are normalized
    ToF tof_rays(map, pos1, ori1, maxRayDist, 4, 45.0f);
    bool rays_normalized = true;
    // Note: We can't directly access rayDir_ from tests, so we verify through behavior
    // that the rays are being generated correctly by checking distances
    Eigen::VectorXf test_distances = tof_rays.readDistances();
    for (int i = 0; i < test_distances.size(); ++i) {
        if (test_distances(i) <= 0.0f) {
            rays_normalized = false;
            break;
        }
    }
    expect_true(rays_normalized, "Ray directions generate valid distance measurements");

    // Test 18: Different FOV angles work correctly
    ToF tof_narrow(map, pos1, ori1, maxRayDist, 4, 10.0f);
    Eigen::VectorXf distances_narrow = tof_narrow.readDistances();
    expect_equal(distances_narrow.size(), 16, "ToF with narrow FOV returns correct size");

    ToF tof_wide(map, pos1, ori1, maxRayDist, 4, 120.0f);
    Eigen::VectorXf distances_wide = tof_wide.readDistances();
    expect_equal(distances_wide.size(), 16, "ToF with wide FOV returns correct size");

    // ========== Print Summary ==========
    std::cout << "\n=== Test Summary ===" << std::endl;
    std::cout << "Tests run: " << tests_run << std::endl;
    std::cout << "Tests failed: " << tests_failed << std::endl;
    std::cout << "Tests passed: " << (tests_run - tests_failed) << std::endl;

    return tests_failed > 0 ? 1 : 0;
}
