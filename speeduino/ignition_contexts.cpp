#include "ignition_contexts.h"
#include "schedule_calcs.h"

bool ignition_context_st::
adjustCrankAngle(int const crankAngle, uint16_t const currentTooth)
{
  bool adjusted;

  if (currentTooth == endTooth)
  {
    ::adjustCrankAngle(*ignitionSchedule, endAngle, crankAngle);
    adjusted = true;
  }
  else
  {
    adjusted = false;
  }

  return adjusted;
}

void ignition_context_st::
calculateIgnitionAngle(
  int const dwellAngle, uint16_t const channelIgnDegrees, int8_t const advance)
{
  ::calculateIgnitionAngle(dwellAngle, channelIgnDegrees, advance, &endAngle, &startAngle);
}

void ignition_context_st::
calculateIgnitionAngle(int const dwellAngle, int8_t const advance)
{
  ::calculateIgnitionAngle(dwellAngle, ignDegrees, advance, &endAngle, &startAngle);
}

uint32_t ignition_context_st::
calculateIgnitionTimeout(int crankAngle)
{
  return ::calculateIgnitionTimeout(*ignitionSchedule, startAngle, ignDegrees, crankAngle);
}


/* Ignitions context methods. */
void ignition_contexts_st::adjustCrankAngle(int16_t const crankAngle, uint16_t const currentTooth)
{
  for (size_t i = 0; i < ignChannelCount; i++)
  {
    if (ignitions[i].adjustCrankAngle(crankAngle, currentTooth))
    {
      break;
    }
  }
}

void ignition_contexts_st::adjustStartAngle(int const adjustment)
{
  for (size_t i = 0; i < ignChannelCount; i++)
  {
    ignitions[i].startAngle += adjustment;
  }
}

void ignition_contexts_st::resetEndAngle(void)
{
  for (size_t i = 0; i < ignChannelCount; i++)
  {
    ignition_context_st &ignition = ignitions[i];

    ignition.endAngle = 0;
  }
}

ignition_contexts_st ignitions;

