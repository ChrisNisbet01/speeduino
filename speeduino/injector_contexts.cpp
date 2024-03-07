#include "injector_contexts.h"
#include "injector_control.h"
#include "utilities.h"
#include "bit_macros.h"

injector_context_st injector_contexts[injChannelCount];
injectors_state_st injectors;

void injector_context_st::applyFuelTrimToPW(trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM)
{
  PW = ::applyFuelTrimToPW(pTrimTable, fuelLoad, RPM, PW);
}

uint16_t injector_context_st::calculateInjectorStartAngle(uint16_t pwDegrees, uint16_t injAngle)
{
  return ::calculateInjectorStartAngle(pwDegrees, channelInjDegrees, injAngle);
}

void injector_context_st::applyInjectorControl(uint16_t injOpenTime, uint16_t openAngle, int crankAngle)
{
  if (PW >= injOpenTime)
  {
    uint32_t const timeout = calculateInjectorTimeout(
        *fuelSchedule, channelInjDegrees, openAngle, crankAngle);

    if (timeout > 0)
    {
      setFuelSchedule(*fuelSchedule, timeout, (unsigned long)PW);
    }
  }
}

void injector_context_st::
reset(void)
{
  channelInjDegrees = 0;
  fuelSchedule->reset();
}

void injectors_state_st::setMaxInjectors(byte const maxOutputs)
{
  this->maxOutputs = maxOutputs;
  this->maxOutputMask = ((uint16_t)1 << maxOutputs) - 1;
}

void injectors_state_st::setAllOn(void)
{
  channelsOn = maxOutputMask;
}

void injectors_state_st::setAllOff(void)
{
  channelsOn = 0;
}

void injectors_state_st::setOn(injectorChannelID_t inj)
{
  BIT_SET(channelsOn, inj);
}

void injectors_state_st::setOff(injectorChannelID_t inj)
{
  BIT_CLEAR(channelsOn, inj);
}

bool injectors_state_st::isOperational(injectorChannelID_t inj)
{
  return (BIT(inj) & maxOutputMask & channelsOn) != 0;
}

byte injectors_state_st::channelsOnMask(void)
{
  return channelsOn;
}

