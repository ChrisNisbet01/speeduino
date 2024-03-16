#include "crank.h"
#include "maths.h"
#include "src/decoders/decoders.h"

static constexpr uint8_t UQ1X15_Shift = 15U;
static constexpr uint8_t const degreesPerMicro_Shift = UQ1X15_Shift;

constexpr uint8_t UQ24X8_Shift = 8U;

static uint8_t const microsPerDegree_Shift = UQ24X8_Shift;

Crank crank;

bool Crank::setRevolutionTime(uint32_t const new_revolution_time)
{
  bool const revolution_time_changed = new_revolution_time != revolutionTime;

  if (revolution_time_changed)
  {
    revolutionTime = new_revolution_time;
    microsPerDegree = div360(revolutionTime << microsPerDegree_Shift);
    degreesPerMicro = UDIV_ROUND_CLOSEST(UINT32_C(360) << degreesPerMicro_Shift, revolutionTime, uint32_t);
  }

  return revolution_time_changed;
}

uint16_t Crank::timeToAngleDegPerMicroSec(uint32_t const time_us)
{
  uint32_t const degFixed = time_us * degreesPerMicro;

  return RSHIFT_ROUND(degFixed, degreesPerMicro_Shift);
}

uint32_t Crank::angleToTimeMicroSecPerDegree(uint16_t angle)
{
  UQ24X8_t micros = (uint32_t)angle * microsPerDegree;
  return RSHIFT_ROUND(micros, microsPerDegree_Shift);
}


