#pragma once

#include "uq.h"

#include <stdint.h>

class Crank
{
public:
  //The time in uS that one revolution would take at current speed
  //(The time tooth 1 was last seen, minus the time it was seen prior to that)
  uint32_t revolutionTime;

  UQ24X8_t microsPerDegree;

  UQ1X15_t degreesPerMicro;

  bool setRevolutionTime(uint32_t const new_revolution_time);
};

extern Crank crank;
