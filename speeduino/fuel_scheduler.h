#pragma once

#include "fuel_schedule_class.h"
#include "globals.h"
#include "maths.h"

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

//The ARM cores use separate functions for their ISRs
#if defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY)
void fuelSchedule1Interrupt(void);
void fuelSchedule2Interrupt(void);
void fuelSchedule3Interrupt(void);
void fuelSchedule4Interrupt(void);
#if (INJ_CHANNELS >= 5)
void fuelSchedule5Interrupt(void);
#endif
#if (INJ_CHANNELS >= 6)
void fuelSchedule6Interrupt(void);
#endif
#if (INJ_CHANNELS >= 7)
void fuelSchedule7Interrupt(void);
#endif
#if (INJ_CHANNELS >= 8)
void fuelSchedule8Interrupt(void);
#endif
#endif

void _setFuelScheduleRunning(FuelSchedule &schedule, unsigned long timeout, unsigned long duration);
void _setFuelScheduleNext(FuelSchedule &schedule, unsigned long timeout, unsigned long duration);
void disablePendingFuelSchedule(byte channel);

inline __attribute__((always_inline)) void
setFuelSchedule(FuelSchedule &schedule, unsigned long timeout, unsigned long duration)
{
    if(schedule.Status != RUNNING)
    {
      //Check that we're not already part way through a schedule
      _setFuelScheduleRunning(schedule, timeout, duration);
    }
    else if(timeout < MAX_TIMER_PERIOD)
    {
      _setFuelScheduleNext(schedule, timeout, duration);
    }
}

static inline uint16_t
applyFuelTrimToPW(trimTable3d * pTrimTable, int16_t fuelLoad, int16_t RPM, uint16_t currentPW)
{
  uint8_t pw1percent = 100U + get3DTableValue(pTrimTable, fuelLoad, RPM) - OFFSET_FUELTRIM;
  return percentage(pw1percent, currentPW);
}

extern FuelSchedule fuelSchedules[injChannelCount];

