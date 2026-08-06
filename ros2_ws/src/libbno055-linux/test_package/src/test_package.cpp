#include <iostream>
#include "libbno055-linux/bno055.hpp"

int main() {
    // Verify that the library can be linked and headers are accessible.
    // We don't attempt to open a real device, just verify the API is available.
    std::cout << "libbno055-linux package test: OK" << std::endl;
    std::cout << "  - BNO055 class available" << std::endl;
    std::cout << "  - Vector3 struct available" << std::endl;
    std::cout << "  - Quaternion struct available" << std::endl;

    // Verify the toEulerDegrees utility function
    bno055lib::Quaternion identity{1.0, 0.0, 0.0, 0.0};
    auto euler = bno055lib::toEulerDegrees(identity);
    std::cout << "  - toEulerDegrees(identity) = (" << euler.x << ", " << euler.y << ", " << euler.z
              << ")" << std::endl;

    return 0;
}
