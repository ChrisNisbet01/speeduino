#pragma once

#include "schedule_status.h"
#include "globals.h"

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

  volatile unsigned long duration = 0;///< Scheduled duration (uS ?)
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

