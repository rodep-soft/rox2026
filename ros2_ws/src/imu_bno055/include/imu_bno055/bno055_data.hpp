#pragma once

#include <cstdint>
#include <array>

struct CalibrationData
{

  std::array<int16_t, 3> accOffset;
  std::array<int16_t, 3> gyrOffset;
  std::array<int16_t, 3> magOffset;

//int16_t accRadius;
  int16_t accRadius;
  int16_t magRadius;

};
