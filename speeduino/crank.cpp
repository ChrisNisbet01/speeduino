#include "crank.h"
#include "maths.h"
#include "src/decoders/decoders.h"

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

