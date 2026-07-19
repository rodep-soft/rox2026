#include <fcntl.h>
#include <sys/ioctl.h>
#include "imu_bno055/bno055_data.hpp"
#include "imu_bno055/bno055.hpp"
#include "imu_bno055/bno055_reg.hpp"
#include <thread>
#include <chrono>
#include <array>
#include <cstdint>
#include <iostream>

BNO055::BNO055(std::string dev, uint8_t addr)
: dev_(dev), addr_(addr), fd_(-1)
{}

bool BNO055::init()
{
  fd_ = open(dev_.c_str(), O_RDWR);

  if (fd_ == -1) {
    perror("open failed");
    return false;
  }

  int ret = ioctl(fd_, I2C_SLAVE, addr_);

  if (ret == -1) {
    perror("ioctl failed");
    close(fd_);
    return false;
  }


  if (!expectChipID()) {
    std::cerr << "CHIP_ID mismatch" << std::endl;
    close(fd_);
    return false;
  }

  if (!setUnit()) {
    std::cerr << "SET UNIT failed" << std::endl;
    close(fd_);
    return false;
  }

  if (!expectSetUnit()) {
    std::cerr << "UNIT_SEL verification failed" << std::endl;
    close(fd_);
    return false;
  }

  if (!setOprMode(BNO055Mode::NDOF)) {//センサーをNDOFmodeにする
    close(fd_);
    return false;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  return true;

}

bool BNO055::readReg(BNO055Reg reg, uint8_t & outValue)
{
  return readRegs(reg, &outValue, 1);
}


bool BNO055::writeReg(uint8_t reg, uint8_t value)
{
  ssize_t ret;

  std::array<uint8_t, 2> buf;

  buf[0] = reg;
  buf[1] = value;

  ret = write(fd_, buf.data(), 2);   //2bit 書く

  if (ret != 2) {
    perror("writeReg failed");
    return false;
  }

  return true;

}


bool BNO055::setUnit()
{
  uint8_t unit = toUnit();

  return writeReg(toUint8(BNO055Reg::UNIT_SEL), unit);

}

bool BNO055::expectSetUnit()
{
  uint8_t current;

  if (!readReg(BNO055Reg::UNIT_SEL, current)) {return false;}

  uint8_t expected = toUnit();

  uint8_t mask =
    ACC_UNIT_MASK |
    GYR_UNIT_MASK |
    EUL_UNIT_MASK;

  return (current & mask) == (expected & mask);

}

uint8_t BNO055::toUnit()
{
  return
    toUint8(BNO055Unit::EUL_UNIT_RADIANS) |
    toUint8(BNO055Unit::GYR_UNIT_RPS) |
    toUint8(BNO055Unit::ACC_UNIT_METER_PER_SECOND_PER_SECOND);
}

bool BNO055::writeInt16(uint8_t lsbReg, int16_t rawValue)
{
  uint16_t value = static_cast<uint16_t>(rawValue);

  uint8_t lsbValue, msbValue;

  lsbValue = value & 0xFF;
  msbValue = (value >> 8) & 0xFF;

  uint8_t lsbAddr = lsbReg;
  uint8_t msbAddr = lsbAddr + 1;

  if (!writeReg(lsbAddr, lsbValue)) {return false;}

  if (!writeReg(msbAddr, msbValue)) {return false;}

  return true;

}

int16_t BNO055::combineInt16(uint8_t lsb, uint8_t msb)
{
  return static_cast<int16_t>(
    (static_cast<uint16_t>(msb) << 8) |
    static_cast<uint16_t>(lsb)
  );
}

// m/s^2 == /100.0f , mg == /1.0f
bool BNO055::readAcceleration(std::array<float, 3> & accValue)
{

  constexpr float ACC_SCALE = 1.0f / 100.0f;

  std::array<int16_t, 3> raw;

  if (!readInt16Array(BNO055Reg::ACC_DATA_X_LSB, raw)) {return false;}

  accValue[0] = static_cast<float>(raw[0]) * ACC_SCALE;
  accValue[1] = static_cast<float>(raw[1]) * ACC_SCALE;
  accValue[2] = static_cast<float>(raw[2]) * ACC_SCALE;

  return true;
}

bool BNO055::readBytes(uint8_t * data, size_t length)
{
  ssize_t ret = read(fd_, data, length);

  if (ret != static_cast<ssize_t>(length)) {
    perror("read reg");
    return false;
  }

  return true;
}

bool BNO055::readRegs(BNO055Reg reg, uint8_t * data, size_t length)
{

  uint8_t rawReg = toUint8(reg);

  ssize_t writeRet = write(fd_, &rawReg, 1);

  if (writeRet != 1) {
    perror("write reg");
    return false;
  }

  if (!readBytes(data, length)) {
    return false;
  }

  return true;
}


//Dps == /16.0f, Rps == /900.0f
bool BNO055::readGyroscope(std::array<float, 3> & gyrValue)
{
  constexpr float GYR_SCALE = 1.0f / 900.0f;

  std::array<int16_t, 3> raw;

  if (!readInt16Array(BNO055Reg::GYR_DATA_X_LSB, raw)) {return false;}

  gyrValue[0] = static_cast<float>(raw[0]) * GYR_SCALE;
  gyrValue[1] = static_cast<float>(raw[1]) * GYR_SCALE;
  gyrValue[2] = static_cast<float>(raw[2]) * GYR_SCALE;

  return true;
}

// Qua /2^14
bool BNO055::readQuaternion(std::array<float, 4> & quatValue)
{
  constexpr float QUAT_SCALE = 1.0f / static_cast<float>(1 << 14);

  std::array<int16_t, 4> raw;

  if (!readInt16Array(BNO055Reg::QUAT_DATA_W_LSB, raw)) {return false;}

  quatValue[0] = static_cast<float>(raw[0]) * QUAT_SCALE;
  quatValue[1] = static_cast<float>(raw[1]) * QUAT_SCALE;
  quatValue[2] = static_cast<float>(raw[2]) * QUAT_SCALE;
  quatValue[3] = static_cast<float>(raw[3]) * QUAT_SCALE;

  return true;
}

// degree /16.0f   radians /900.0f
bool BNO055::readEuler(std::array<float, 3> & eulValue)
{
  constexpr float EUL_SCALE = 1.0f / 900.0f;
  //ここの上ymlで追い出すなら使用知らんけど変数名の差異で変更させるだけで行ければ熱そう

  std::array<int16_t, 3> raw;

  if (!readInt16Array(BNO055Reg::EUL_DATA_X_LSB, raw)) {return false;}

  eulValue[0] = static_cast<float>(raw[0]) * EUL_SCALE;
  eulValue[1] = static_cast<float>(raw[1]) * EUL_SCALE;
  eulValue[2] = static_cast<float>(raw[2]) * EUL_SCALE;

  return true;
}


bool BNO055::expectChipID()
{
  uint8_t chipID;

  if (!readReg(BNO055Reg::CHIP_ID, chipID)) {return false;}

  return chipID == EXPECT_CHIP_ID;
}

bool BNO055::setOprMode(BNO055Mode mode)
{
  constexpr int MAX_RETRY = 3;

  for (int i = 0; i < MAX_RETRY; i++) {
    if (!writeReg(toUint8(BNO055Reg::OPR_MODE), toUint8(mode))) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(20));

    uint8_t currentMode;

    if (!readReg(BNO055Reg::OPR_MODE, currentMode)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    if ((currentMode & 0x0F) == toUint8(mode)) {
      return true;
    }
  }

  std::cerr << "Failed to set operation mode: "
            << static_cast<int>(toUint8(mode))
            << std::endl;

  return false;
}

bool BNO055::writeCalibration(const CalibrationData & calibData)
{

  if (!setOprMode(BNO055Mode::CONFIG)) {
    return false;
  }

  if (!writeOffset(calibData.accOffset, BNO055Reg::ACC_OFFSET_X_LSB)) {
    std::cerr << "Failed to write accOffset" << std::endl;
    return false;
  }

  if (!writeOffset(calibData.gyrOffset, BNO055Reg::GYR_OFFSET_X_LSB)) {
    std::cerr << "Failed to write gyrOffset" << std::endl;
    return false;
  }

  if (!writeOffset(calibData.magOffset, BNO055Reg::MAG_OFFSET_X_LSB)) {
    std::cerr << "Failed to write magOffset" << std::endl;
    return false;
  }

  if (!writeInt16(toUint8(BNO055Reg::ACC_RADIUS_LSB), calibData.accRadius)) {
    std::cerr << "Failed to write accmeter radius" << std::endl;
    return false;
  }

  if (!writeInt16(toUint8(BNO055Reg::MAG_RADIUS_LSB), calibData.magRadius)) {
    std::cerr << "Failed to write gyrmeter radius" << std::endl;
    return false;
  }

  if (!setOprMode(BNO055Mode::NDOF)) {
    return false;
  }

  std::cout << "Succsess write to Calibration" << std::endl;

  return true;
}

bool BNO055::writeOffset(const std::array<int16_t, 3> & offsetArray, BNO055Reg baseReg)
{

  uint8_t xLsbReg = toUint8(baseReg);
  uint8_t yLsbReg = xLsbReg + 2;
  uint8_t zLsbReg = yLsbReg + 2;

  if (!writeInt16(xLsbReg, offsetArray[0])) {return false;}
  if (!writeInt16(yLsbReg, offsetArray[1])) {return false;}
  if (!writeInt16(zLsbReg, offsetArray[2])) {return false;}

  return true;
}


bool BNO055::readCalibration(CalibrationData & calibData)
{

  if (!readInt16Array(BNO055Reg::ACC_OFFSET_X_LSB, calibData.accOffset)) {return false;}

  if (!readInt16Array(BNO055Reg::GYR_OFFSET_X_LSB, calibData.gyrOffset)) {return false;}

  if (!readInt16Array(BNO055Reg::MAG_OFFSET_X_LSB, calibData.magOffset)) {return false;}

  std::array<int16_t, 1> raw;

  if (!readInt16Array(BNO055Reg::ACC_RADIUS_LSB, raw)) {return false;}

  calibData.accRadius = raw[0];

  if (!readInt16Array(BNO055Reg::MAG_RADIUS_LSB, raw)) {return false;}

  calibData.magRadius = raw[0];

  return true;
}

bool BNO055::isCalib()
{

  bool ret = true;
  //ここはキャリブレーションできたか有無を知りたいのでretを使用
  //あとaccとかも追加必須

  uint8_t accCalibStatus, gyrCalibStatus;

  uint8_t calibData;

  if (!readReg(BNO055Reg::CALIB_STAT, calibData)) {return false;}

  accCalibStatus = (calibData >> ACC_CALIB_BIT_SHIFT) & CALIB_MASK;  //2bit抽出
  gyrCalibStatus = (calibData >> GYR_CALIB_BIT_SHIFT) & CALIB_MASK;

  if (accCalibStatus != CALIB_APPROPRIATE) {
    std::cerr << "accCalibStatus is inappropriate" << std::endl;
    ret = false;
  }

  if (gyrCalibStatus != CALIB_APPROPRIATE) {
    std::cerr << "gyrCalibStatus is inappropriate" << std::endl;
    ret = false;
  }

  return ret;
}
