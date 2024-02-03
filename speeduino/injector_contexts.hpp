#pragma once

#include "scheduler.h"
#include "schedule_calcs.h"
#include "injector_id.h"

#include <stdint.h>

typedef struct injector_context_st
{
  FuelSchedule * fuelSchedule;
  unsigned int PW;
  /* The number of crank degrees until the associated cylinder is at TDC */
  int channelInjDegrees;

  void scheduleFuel(uint32_t const timeout);

  void applyFuelTrimToPW(trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM);

  uint16_t calculateInjectorStartAngle(uint16_t pwDegrees, uint16_t injAngle);

  void configure_injector_schedule(injector_id_t injector_id);

  void configure_injector_schedule(injector_id_t injector_id1, injector_id_t injector_id2);

  void applyInjectorControl(uint16_t injOpenTime, uint16_t openAngle, int crankAngle);

} injector_context_st;

typedef struct injectors_context_st
{
public:
  byte maxOutputs = 1; /**< Number of injection outputs being used by the current tune configuration */
  byte channelsOn;

  injector_context_st injectors[INJ_CHANNELS];

  void setMaxInjectors(byte const maxOutputs);

  void applyFuelTrimToPW(byte inj, trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM);

  uint16_t calculateInjectorStartAngle(byte inj, uint16_t pwDegrees, uint16_t injAngle);

  void setAllOn(void);

  void setAllOff(void);

  void setOn(byte inj);

  void setOff(byte inj);

  bool isOperational(byte inj);

  byte channelsOnMask(void);

  void configure_injector_schedule(byte inj, injector_id_t injector_id);

  void configure_injector_schedule(byte inj, injector_id_t injector_id1, injector_id_t injector_id2);

  void configure_sequential_injector_schedules(size_t const count);

  void applyInjectorControl(byte inj, uint16_t injOpenTime, uint16_t openAngle, int crankAngle);

private:
  byte maxOutputMask = 0x01;

} injectors_context_st;

extern injectors_context_st injectors_context;

