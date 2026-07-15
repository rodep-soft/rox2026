#pragma once

#include <cstdint>
#include <array>
#include <string>
#include <iostream>
#include <thread>
#include <chrono>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>

#include "imu_bno055/bno055_data.hpp"
#include "imu_bno055/bno055_reg.hpp"


class BNO055 
{
public:

  BNO055(std::string dev, uint8_t addr);

  bool init(); 
  bool readAcceleration(std::array<float, 3>& accValue);
  bool readGyroscope(std::array<float, 3>& gyrValue);
  bool readQuaternion(std::array<float, 4>& quatValue);
  bool readEuler(std::array<float, 3>& eulValue); 
 
  bool writeCalibration(const CalibrationData &calibData);
  bool readCalibration(CalibrationData &calibData);

  bool isCalib();

private:

  template <typename T>
  constexpr uint8_t toUint8(T value)
{
  return static_cast<uint8_t>(value);
}

  bool writeReg(uint8_t reg, uint8_t value);

  bool readReg(BNO055Reg reg, uint8_t& outValue);
 
  bool readRegs(BNO055Reg reg, uint8_t* data, size_t length);

  std::string dev_;

  uint8_t addr_;

  int fd_;

  bool setOprMode(BNO055Mode mode);
 
  bool readOprMode();

  bool readInt16(BNO055Reg lsbReg, int16_t& outValue);

  bool writeInt16(uint8_t lsbReg, int16_t rawValue);

  bool writeOffset(const std::array<int16_t, 3>& offsetArray, BNO055Reg baseReg);

  bool expectSetUnit();//そのうちinitに追加

  bool setUnit();
  
  uint8_t toUnit();
  
  bool expectChipID();
  
  bool readBytes(uint8_t* data, size_t length);

  int16_t combineInt16(uint8_t lsb, uint8_t msb);

  template<size_t N>
  bool readInt16Array(
    BNO055Reg startReg,
    std::array<int16_t, N>& values)
  {
    uint8_t buffer[N * 2];

    if (!readRegs(startReg, buffer, N * 2)) {
        return false;
    }

    for (size_t i = 0; i < N; i++) {
        values[i] = combineInt16(
            buffer[i * 2],
            buffer[i * 2 + 1]
        );
    }

    return true;
  }
};








