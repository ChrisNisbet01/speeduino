#pragma once

/** Schedule statuses.
 * - OFF - Schedule turned off and there is no scheduled plan
 * - PENDING - There's a scheduled plan, but is has not started to run yet
 * - STAGED - (???, Not used)
 * - RUNNING - Schedule is currently running
 */
enum ScheduleStatus
{
  OFF,
  PENDING,
  STAGED,
  RUNNING
}; //The statuses that a schedule can have

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

  FuelSchedule( counter_t &counter, compare_t &compare,
            void (&_pTimerDisable)(), void (&_pTimerEnable)())
  : counter(counter)
  , compare(compare)
  , pTimerDisable(_pTimerDisable)
  , pTimerEnable(_pTimerEnable)
  {
  }

  volatile unsigned long duration;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  volatile COMPARE_TYPE startCompare; ///< The counter value of the timer when this will start
  volatile COMPARE_TYPE endCompare;   ///< The counter value of the timer when this will end
  struct
  {
    void (*pCallback)(uint8_t arg1, uint8_t arg2);
    uint8_t args[2];
  } start;
  struct
  {
    void (*pCallback)(uint8_t arg1, uint8_t arg2);
    uint8_t args[2];
  } end;
  COMPARE_TYPE nextStartCompare;
  COMPARE_TYPE nextEndCompare;
  volatile bool hasNextSchedule = false;

  counter_t &counter;  // Reference to the counter register. E.g. TCNT3
  compare_t &compare;  // Reference to the compare register. E.g. OCR3A
  void (&pTimerDisable)();    // Reference to the timer disable function
  void (&pTimerEnable)();     // Reference to the timer enable function
};

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
    void (*pCallback)(uint8_t arg1, uint8_t arg2);
    uint8_t args[2];
  } start;
  struct
  {
    void (*pCallback)(uint8_t arg1, uint8_t arg2);
    uint8_t args[2];
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
};

