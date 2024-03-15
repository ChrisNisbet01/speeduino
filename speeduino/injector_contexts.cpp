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
      setFuelSchedule(*fuelSchedule, timeout, (uint32_t)PW);
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
  this->maxOutputMask = ((uint8_t)1 << maxOutputs) - 1;
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

static void initialiseFuelSchedules(void)
{
  for (size_t i = 0; i < injChannelCount; i++)
  {
    injector_contexts[i].fuelSchedule = &fuelSchedules[i];
  }
}

/** Perform the injector priming pulses.
 * Set these to run at an arbitrary time in the future (100us).
 * The prime pulse value is in ms*10, so need to multiply by 100 to get to uS
 */
void beginInjectorPriming(void)
{
  static uint32_t const priming_delay_us = 100;
  uint32_t primingValue =
    table2D_getValue(&PrimingPulseTable, currentStatus.coolant + CALIBRATION_TEMPERATURE_OFFSET);

  if (primingValue > 0 && currentStatus.TPS < configPage4.floodClear)
  {
    // To achieve long enough priming pulses, the values in tuner studio are divided by 0.5 instead of 0.1,
    // so multiplier of 5 is required.
    // XXX - Should that be _multiplied_ by 0.5, which also means the value should be multiplied by 2?
    static unsigned config_multiplier = 5;
    primingValue = MS_TIMES_10_TO_US(primingValue * config_multiplier);

    for (size_t i = 0; i < injectors.maxOutputs; i++)
    {
      setFuelSchedule(fuelSchedules[i], priming_delay_us, primingValue);
    }
  }
}

void initialiseAndResetFuelSchedules(void)
{
  initialiseFuelSchedules();
  for (size_t i = injChannel1; i < injChannelCount; i++)
  {
    injector_context_st &injector = injector_contexts[i];

    injector.reset();
  }
}

