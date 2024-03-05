#pragma once

#include "ignition_id.h"
#include "schedule_status.h"
#include "globals.h"

typedef void (*coilCallback_fn)(void);

/** Ignition schedule.
 */
struct IgnitionSchedule {

  // Deduce the real types of the counter and compare registers.
  // COMPARE_TYPE is NOT the same - it's just an integer type wide enough to
  // store 16-bit counter/compare calculation results.
  using counter_t = decltype(IGN1_COUNTER);
  using compare_t = decltype(IGN1_COMPARE);

  IgnitionSchedule( counter_t &counter, compare_t &compare,
            void (&_pTimerDisable)(), void (&_pTimerEnable)())
  : counter(counter)
  , compare(compare)
  , pTimerDisable(_pTimerDisable)
  , pTimerEnable(_pTimerEnable)
  {
  }

  volatile unsigned long duration = 0;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status = OFF; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  struct
  {
    coilCallback_fn pCallback = nullptr;
  } start;
  struct
  {
    coilCallback_fn pCallback = nullptr;
  } end;
  volatile unsigned long startTime = 0; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
  volatile COMPARE_TYPE startCompare = 0; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare = 0;   ///< The counter value of the timer when this will end

  COMPARE_TYPE nextStartCompare = 0;      ///< Planned start of next schedule (when current schedule is RUNNING)
  COMPARE_TYPE nextEndCompare = 0;        ///< Planned end of next schedule (when current schedule is RUNNING)
  volatile bool hasNextSchedule = false; ///< Enable flag for planned next schedule (when current schedule is RUNNING)
  volatile bool endScheduleSetByDecoder = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function

  void reset(void);
};
