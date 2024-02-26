#pragma once

#include "injector_id.h"
#include "schedule_status.h"
#include "globals.h"

typedef void (*injectorCallback_fn)(injector_id_t coil_id1, injector_id_t coil_id2);

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
    injector_id_t injector_ids[2] = {injector_id_1, injector_id_1};
  } start;
  struct
  {
    injectorCallback_fn pCallback = nullptr;
    injector_id_t injector_ids[2] = {injector_id_1, injector_id_1};
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

extern FuelSchedule fuelSchedule1;
extern FuelSchedule fuelSchedule2;
extern FuelSchedule fuelSchedule3;
extern FuelSchedule fuelSchedule4;
#if INJ_CHANNELS >= 5
extern FuelSchedule fuelSchedule5;
#endif
#if INJ_CHANNELS >= 6
extern FuelSchedule fuelSchedule6;
#endif
#if INJ_CHANNELS >= 7
extern FuelSchedule fuelSchedule7;
#endif
#if INJ_CHANNELS >= 8
extern FuelSchedule fuelSchedule8;
#endif
