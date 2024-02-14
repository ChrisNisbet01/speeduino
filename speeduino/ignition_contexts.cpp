#include "ignition_contexts.h"

#include "ignition_schedule.h"
#include "ignition_control.h"
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

void ignition_context_st::
applyOverDwellCheck(uint32_t targetOverdwellTime)
{
  //Check first whether each spark output is currently on. Only check it's dwell time if it is
  if (ignitionSchedule->Status == RUNNING && ignitionSchedule->startTime < targetOverdwellTime)
  {
    ignitionSchedule->end.pCallback(ignitionSchedule->end.coil_ids[0], ignitionSchedule->end.coil_ids[1]);
    ignitionSchedule->Status = OFF;
  }
}

void ignition_context_st::
reset(void)
{
  startAngle = 0;
  endAngle = 0;
  ignDegrees = 0;
  ignitionSchedule->reset();
}

void ignition_context_st::
configure_coil_schedule(ignition_id_t const id)
{
  configure_ignition_coil_schedule(*ignitionSchedule, id);
}

void ignition_context_st::
configure_coil_schedule(ignition_id_t const id1, ignition_id_t const id2)
{
  configure_ignition_coil_schedule(*ignitionSchedule, id1, id2);
}

void ignition_context_st::
inhibit_coil_schedule(void)
{
  ignitionSchedule->start.pCallback = nullCallback;
  ignitionSchedule->end.pCallback = nullCallback;
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

void ignition_contexts_st::
configure_coil_schedule(ignitionChannelID_t const ign, ignition_id_t const id)
{
  ignitions[ign].configure_coil_schedule(id);
}

void ignition_contexts_st::
configure_coil_schedule(ignitionChannelID_t const ign, ignition_id_t const id1, ignition_id_t const id2)
{
  ignitions[ign].configure_coil_schedule(id1, id2);
}

void ignition_contexts_st::
inhibit_coil_schedule(ignitionChannelID_t const ign)
{
  ignitions[ign].inhibit_coil_schedule();
}

void ignition_contexts_st::
configure_rotary_fc_trailing_coil_schedules(void)
{
  ::configure_rotary_fc_trailing_coil_schedules(
    *ignitions[ignition_id_3].ignitionSchedule, *ignitions[ignition_id_4].ignitionSchedule);
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
