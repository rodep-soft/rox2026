#pragma once
#include <cstdint>
#include <iostream>

constexpr uint8_t EXPECT_CHIP_ID = 0xA0;//yml追い出し


enum class BNO055Mode : uint8_t
{
  //ここに必要なレジスタ番号を記入
  CONFIG = 0X00,
  NDOF = 0x0C,
};

enum class BNO055Unit : uint8_t
{

  //EUL_Unit
  EUL_UNIT_DEGREES = 0,
  EUL_UNIT_RADIANS = 1,

  //GYR_Unit
  GYR_UNIT_DPS = 0,
  GYR_UNIT_RPS = 1,

  //ACC_Unit
  ACC_UNIT_METER_PER_SECOND_PER_SECOND = 0,
  ACC_UNIT_MG = 1,

};


//acc, gyr, qua, eul
enum class BNO055Reg : uint8_t
{

  CHIP_ID = 0x00,
  UNIT_SEL = 0X3B,
  OPR_MODE = 0x3D,

  //Acceleration data
  ACC_DATA_X_LSB = 0x08,
  ACC_DATA_Y_LSB = 0x0A,
  //ACC_DATA_Z_LSB = 0x0C,

  //Gyroscope data
  GYR_DATA_X_LSB = 0x14,
  GYR_DATA_Y_LSB = 0x16,
  GYR_DATA_Z_LSB = 0x18,

  //quaternion data
  QUAT_DATA_W_LSB = 0x20,
  QUAT_DATA_X_LSB = 0x22,
  QUAT_DATA_Y_LSB = 0x24,
  QUAT_DATA_Z_LSB = 0x26,

  //euler data
  EUL_DATA_X_LSB = 0x1A,
  EUL_DATA_Y_LSB = 0x1C,
  EUL_DATA_Z_LSB = 0x1E,

  //Acceleration offset
  ACC_OFFSET_X_LSB = 0x55,
  ACC_OFFSET_Y_LSB = 0x57,
  ACC_OFFSET_Z_LSB = 0x59,

  //gyroscope offset
  GYR_OFFSET_X_LSB = 0x61,
  GYR_OFFSET_Y_LSB = 0x63,
  GYR_OFFSET_Z_LSB = 0x65,

  //mag offset
  MAG_OFFSET_X_LSB = 0x5B,
  MAG_OFFSET_Y_LSB = 0x5D,
  MAG_OFFSET_Z_LSB = 0x5F,

  // acc & mag radius
  ACC_RADIUS_LSB = 0x67,
  MAG_RADIUS_LSB = 0x69,

  //Calibration reg
  CALIB_STAT = 0x35,
};

//角速度、加速度

namespace
{

// unit
constexpr uint8_t ACC_UNIT_MASK = 0x01;
constexpr uint8_t GYR_UNIT_MASK = 0x02;
constexpr uint8_t EUL_UNIT_MASK = 0x04;

// calibration

constexpr uint8_t CALIB_MASK = 0x03;
constexpr uint8_t CALIB_APPROPRIATE = 3;  //CALIB_DONE より　こっちの方がいいのでは？
constexpr uint8_t ACC_CALIB_BIT_SHIFT = 2;
constexpr uint8_t GYR_CALIB_BIT_SHIFT = 4;
}
