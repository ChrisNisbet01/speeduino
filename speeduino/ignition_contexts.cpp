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

void ignition_context_st::
setIgnitionSchedule(unsigned long const timeout, unsigned long const durationMicrosecs)
{
  ::setIgnitionSchedule(*ignitionSchedule, timeout, durationMicrosecs);
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

void ignition_contexts_st::setMaxIgnitions(byte const maxOutputs)
{
  this->maxOutputs = maxOutputs;
  this->maxOutputMask = ((uint16_t)1 << maxOutputs) - 1;
}

void ignition_contexts_st::setAllOn(void)
{
  channelsOn = maxOutputMask;
}

void ignition_contexts_st::setAllOff(void)
{
  channelsOn = 0;
}

void ignition_contexts_st::setOn(ignitionChannelID_t ign)
{
  BIT_SET(channelsOn, ign);
}

void ignition_contexts_st::setOff(ignitionChannelID_t ign)
{
  BIT_CLEAR(channelsOn, ign);
}

bool ignition_contexts_st::isOperational(ignitionChannelID_t ign)
{
  return ((1 << ign) & maxOutputMask & channelsOn) != 0;
}

byte ignition_contexts_st::channelsOnMask(void)
{
  return channelsOn;
}

void ignition_contexts_st::setChannelsOnMask(uint8_t const mask)
{
  channelsOn = mask;
}

void ignition_contexts_st::
applyIgnitionControl(ignitionChannelID_t const ign, int const crankAngle)
{
  if (isOperational(ign))
  {
    ignition_context_st &ignition = ignitions[ign];
    uint32_t const timeOut = ignition.calculateIgnitionTimeout(crankAngle);

    if (timeOut > 0U)
    {
      ignition.setIgnitionSchedule(timeOut, currentStatus.dwell + fixedCrankingOverride);
    }
  }
}

ignition_contexts_st ignitions;

