#pragma once

#include "globals.h"
#include "maths.h"
#include "schedule_status.h"

typedef void (*injectorCallback_fn)(void);

/** Fuel injection schedule.
* Fuel schedules don't use the callback pointers, or the startTime/endScheduleSetByDecoder variables.
* They are removed in this struct to save RAM.
*/
struct FuelSchedule {

  // Deduce the real types of the counter and compare registers.
  // COMPARE_TYPE is NOT the same - it's just an integer type wide enough to
  // store 16-bit counter/compare calculation results.
  using counter_t = decltype(FUEL1_COUNTER);
  using compare_t = decltype(FUEL1_COMPARE);

  FuelSchedule(counter_t &counter, compare_t &compare,
            void (&_pTimerDisable)(), void (&_pTimerEnable)())
  : counter(counter)
  , compare(compare)
  , pTimerDisable(_pTimerDisable)
  , pTimerEnable(_pTimerEnable)
  {
  }

  volatile uint32_t duration = 0;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status = OFF; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  volatile COMPARE_TYPE startCompare = 0; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare = 0;   ///< The counter value of the timer when this will end
  struct
  {
    injectorCallback_fn pCallback = nullptr;
  } start;
  struct
  {
    injectorCallback_fn pCallback = nullptr;
  } end;
  COMPARE_TYPE nextStartCompare = 0;
  COMPARE_TYPE nextEndCompare = 0;
  volatile bool hasNextSchedule = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function

  void reset(void);
};

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

void _setFuelScheduleRunning(FuelSchedule &schedule, uint32_t timeout, uint32_t duration);
void _setFuelScheduleNext(FuelSchedule &schedule, uint32_t timeout, uint32_t duration);
void disablePendingFuelSchedule(byte channel);
void nullInjCallback(void);

inline __attribute__((always_inline)) void
setFuelSchedule(FuelSchedule &schedule, uint32_t timeout, uint32_t duration)
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

