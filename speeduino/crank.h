#pragma once

#include "uq.h"

#include <stdint.h>

class Crank
{
  uint32_t revolutionTime;
  UQ24X8_t microsPerDegree;
  UQ1X15_t degreesPerMicro;

  bool setRevolutionTime(uint32_t const new_revolution_time);
};

