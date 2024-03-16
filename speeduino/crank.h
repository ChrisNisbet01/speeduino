#pragma once

#include "uq.h"

#include <stdint.h>

typedef uint16_t UQ1X15_t;

class Crank
{
public:
  //The time in uS that one revolution would take at current speed
  //(The time tooth 1 was last seen, minus the time it was seen prior to that)
  uint32_t revolutionTime;

  UQ24X8_t microsPerDegree;
  static uint8_t const microsPerDegree_Shift = UQ24X8_Shift;

  bool setRevolutionTime(uint32_t new_revolution_time);

  /**
   * @name Converts a time interval in microseconds to the equivalent degrees of angular (crank)
   * rotation at current RPM.
   *
   * @param time_us Time interval in uS
   * @param degrees_per_us Amount of crank angle expected per microsecond
   * @return Angle in degrees
   */
  ///@{
  /** @brief Converts based on the the interval on time one degree of rotation takes
   *
   * Inverse of angleToTimeMicroSecPerDegree
  */
  uint16_t timeToAngleDegPerMicroSec(uint32_t time_us);

private:
    /** @brief Degrees per uS in UQ1.15 fixed point.
   *
   * Ranges from 8 (0.000246) at MIN_RPM to 3542 (0.108) at MAX_RPM
   */
  UQ1X15_t degreesPerMicro;

};

extern Crank crank;
