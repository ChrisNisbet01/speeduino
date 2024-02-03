#include "injector_contexts.hpp"
#include "injector_schedule.h"

injectors_context_st injectors;

static void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id)
{
  fuelSchedule.start.pCallback = openSingleInjector;
  fuelSchedule.start.args[0] = injector_id;
  fuelSchedule.end.pCallback = closeSingleInjector;
  fuelSchedule.end.args[0] = injector_id;
}

static void
configure_injector_schedule(FuelSchedule &fuelSchedule, injector_id_t injector_id1, injector_id_t injector_id2)
{
  fuelSchedule.start.pCallback = openTwoInjectors;
  fuelSchedule.start.args[0] = injector_id1;
  fuelSchedule.start.args[1] = injector_id2;
  fuelSchedule.end.pCallback = closeTwoInjectors;
  fuelSchedule.end.args[0] = injector_id1;
  fuelSchedule.end.args[1] = injector_id2;
}

void injector_context_st::scheduleFuel(uint32_t const timeout)
{
  if (timeout > 0)
  {
    setFuelSchedule(*fuelSchedule, timeout, (unsigned long)PW);
  }
}

void injector_context_st::applyFuelTrimToPW(trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM)
{
  PW = ::applyFuelTrimToPW(pTrimTable, fuelLoad, RPM, PW);
}

uint16_t injector_context_st::calculateInjectorStartAngle(uint16_t pwDegrees, uint16_t injAngle)
{
  return ::calculateInjectorStartAngle(pwDegrees, channelInjDegrees, injAngle);
}


void injector_context_st::configure_injector_schedule(injector_id_t injector_id)
{
  ::configure_injector_schedule(*fuelSchedule, injector_id);
}

void injector_context_st::configure_injector_schedule(injector_id_t injector_id1, injector_id_t injector_id2)
{
  ::configure_injector_schedule(*fuelSchedule, injector_id1, injector_id2);
}

void injector_context_st::applyInjectorControl(uint16_t injOpenTime, uint16_t openAngle, int crankAngle)
{
  if (PW >= injOpenTime)
  {
    uint32_t const timeOut = calculateInjectorTimeout(
        *fuelSchedule, channelInjDegrees, openAngle, crankAngle);

    scheduleFuel(timeOut);
  }
}


void injectors_context_st::setMaxInjectors(byte const maxOutputs)
{
  this->maxOutputs = maxOutputs;
  this->maxOutputMask = ((uint16_t)1 << maxOutputs) - 1;
}

void injectors_context_st::applyFuelTrimToPW(injectorChannelID_t inj, trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM)
{
  injector_context_st &injector = injectors[inj];

  injector.applyFuelTrimToPW(pTrimTable, fuelLoad, RPM);
}

uint16_t injectors_context_st::calculateInjectorStartAngle(injectorChannelID_t inj, uint16_t pwDegrees, uint16_t injAngle)
{
  injector_context_st &injector = injectors[inj];

  return injector.calculateInjectorStartAngle(pwDegrees, injAngle);
}

void injectors_context_st::setAllOn(void)
{
  channelsOn = maxOutputMask;
}

void injectors_context_st::setAllOff(void)
{
  channelsOn = 0;
}

void injectors_context_st::setOn(injectorChannelID_t inj)
{
  BIT_SET(channelsOn, inj);
}

void injectors_context_st::setOff(injectorChannelID_t inj)
{
  BIT_CLEAR(channelsOn, inj);
}

bool injectors_context_st::isOperational(injectorChannelID_t inj)
{
  return ((1 << inj) & maxOutputMask & channelsOn) != 0;
}

byte injectors_context_st::channelsOnMask(void)
{
  return channelsOn;
}

void injectors_context_st::configure_injector_schedule(injectorChannelID_t inj, injector_id_t injector_id)
{
  injector_context_st &injector = injectors[inj];

  injector.configure_injector_schedule(injector_id);
}

void injectors_context_st::configure_injector_schedule(injectorChannelID_t inj, injector_id_t injector_id1, injector_id_t injector_id2)
{
  injector_context_st &injector = injectors[inj];

  injector.configure_injector_schedule(injector_id1, injector_id2);
}

void
injectors_context_st::configure_sequential_injector_schedules(size_t const count)
{
  for (size_t i = 0; i < MIN(count, (size_t)injChannelCount); i++)
  {
    injector_context_st &injector = injectors[i];

    injector.configure_injector_schedule((injector_id_t)(injector_id_1 + i));
  }
}

void injectors_context_st::applyInjectorControl(injectorChannelID_t inj, uint16_t injOpenTime, uint16_t openAngle, int crankAngle)
{
  injector_context_st &injector = injectors[inj];

  if (isOperational(inj))
  {
    injector.applyInjectorControl(injOpenTime, openAngle, crankAngle);
  }
}

