#include "imu_bno055/bno055.hpp"

#include <array>
#include <iostream>

int main()
{
  BNO055 imu("/dev/i2c-1", 0x28);

  std::cout << "init start" << std::endl;

  if (!imu.init()) {
    std::cerr << "BNO055 init failed" << std::endl;
    return 1;
  }

  std::cout << "init success" << std::endl;

  std::array<float, 3> acc{};
  std::array<float, 3> gyr{};
  std::array<float, 4> quat{};

  if (imu.readAcceleration(acc)) {
    std::cout << "acc: "
              << acc[0] << ", "
              << acc[1] << ", "
              << acc[2] << std::endl;
  }

  if (imu.readGyroscope(gyr)) {
    std::cout << "gyro: "
              << gyr[0] << ", "
              << gyr[1] << ", "
              << gyr[2] << std::endl;
  }


  if (imu.readQuaternion(quat)) {
    std::cout << "quat: "
              << quat[0] << ", "
              << quat[1] << ", "
              << quat[2] << ", "
              << quat[3] << std::endl;
  }
  return 0;
}
