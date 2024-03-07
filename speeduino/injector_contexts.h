#pragma once

#include "scheduler.h"
#include "schedule_calcs.h"

#include <stdint.h>

typedef enum
{
  injChannel1,
  injChannel2,
  injChannel3,
  injChannel4,
#if INJ_CHANNELS >= 5
  injChannel5,
#endif
#if INJ_CHANNELS >= 6
  injChannel6,
#endif
#if INJ_CHANNELS >= 7
  injChannel7,
#endif
#if INJ_CHANNELS >= 8
  injChannel8,
#endif
  injChannelCount,
} injectorChannelID_t;

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

extern injector_context_st injector_contexts[injChannelCount];
extern FuelSchedule fuelSchedules[injChannelCount];
extern injectors_state_st injectors;
