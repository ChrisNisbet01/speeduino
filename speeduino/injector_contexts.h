#pragma once

#include "fuel_scheduler.h"
#include "scheduler.h"
#include "schedule_calcs.h"

#include <stdint.h>

typedef struct injector_context_st
{
  FuelSchedule * fuelSchedule = nullptr;
  unsigned int PW = 0;
  /* The number of crank degrees until the associated cylinder is at TDC */
  int channelInjDegrees = 0;

  void reset(void);

  void applyFuelTrimToPW(trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM);

  uint16_t calculateInjectorStartAngle(uint16_t pwDegrees, uint16_t injAngle);

  void applyInjectorControl(uint16_t injOpenTime, uint16_t openAngle, int crankAngle);
} injector_context_st;

typedef struct injectors_state_st
{
public:
  byte maxOutputs = 1; /**< Number of injection outputs being used by the current tune configuration */
  byte channelsOn = 0;

  void setMaxInjectors(byte const maxOutputs);

  void setAllOn(void);

  void setAllOff(void);

  void setOn(injectorChannelID_t inj);

  void setOff(injectorChannelID_t inj);

  bool isOperational(injectorChannelID_t inj);

  byte channelsOnMask(void);

private:
  byte maxOutputMask = 0x01;
} injectors_state_st;

void beginInjectorPriming(void);

void initialiseAndResetFuelSchedules(void);

extern injector_context_st injector_contexts[injChannelCount];
extern injectors_state_st injectors;
