#pragma once

#include "ignition_id.h"
#include "schedule_status.h"
#include "globals.h"

typedef void (*coilCallback_fn)(ignition_id_t coil_id1, ignition_id_t coil_id2);

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

  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  struct
  {
    coilCallback_fn pCallback;
    ignition_id_t coil_ids[2];
  } start;
  struct
  {
    coilCallback_fn pCallback;
    ignition_id_t coil_ids[2];
  } end;
  volatile unsigned long startTime; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end

  COMPARE_TYPE nextStartCompare;      ///< Planned start of next schedule (when current schedule is RUNNING)
  COMPARE_TYPE nextEndCompare;        ///< Planned end of next schedule (when current schedule is RUNNING)
  volatile bool hasNextSchedule = false; ///< Enable flag for planned next schedule (when current schedule is RUNNING)
  volatile bool endScheduleSetByDecoder = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function

  void reset(void);
};

