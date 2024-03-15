#pragma once

#include "globals.h"
#include "schedule_status.h"

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

  volatile uint32_t duration = 0;///< Scheduled duration (uS ?)
  volatile ScheduleStatus Status = OFF; ///< Schedule status: OFF, PENDING, STAGED, RUNNING
  struct
  {
    coilCallback_fn pCallback = nullptr;
  } start;
  struct
  {
    coilCallback_fn pCallback = nullptr;
  } end;
  volatile uint32_t startTime = 0; /**< The system time (in uS) that the schedule started, used by the overdwell protection in timers.ino */
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

typedef enum
{
  ignChannel1,
  ignChannel2,
  ignChannel3,
  ignChannel4,
#if IGN_CHANNELS >= 5
  ignChannel5,
#endif
#if IGN_CHANNELS >= 6
  ignChannel6,
#endif
#if IGN_CHANNELS >= 7
  ignChannel7,
#endif
#if IGN_CHANNELS >= 8
  ignChannel8,
#endif
  ignChannelCount,
} ignitionChannelID_t;

#define USE_IGN_REFRESH
//Time in uS that the refresh functions will check to ensure there is enough
//time before changing the end compare
#define IGNITION_REFRESH_THRESHOLD  30

void disablePendingIgnSchedule(byte channel);

void refreshIgnitionSchedule1(uint32_t timeToEnd);

//The ARM cores use separate functions for their ISRs
#if defined(ARDUINO_ARCH_STM32) || defined(CORE_TEENSY)

#if (IGN_CHANNELS >= 1)
  void ignitionSchedule1Interrupt(void);
#endif
#if (IGN_CHANNELS >= 2)
  void ignitionSchedule2Interrupt(void);
#endif
#if (IGN_CHANNELS >= 3)
  void ignitionSchedule3Interrupt(void);
#endif
#if (IGN_CHANNELS >= 4)
  void ignitionSchedule4Interrupt(void);
#endif
#if (IGN_CHANNELS >= 5)
  void ignitionSchedule5Interrupt(void);
#endif
#if (IGN_CHANNELS >= 6)
  void ignitionSchedule6Interrupt(void);
#endif
#if (IGN_CHANNELS >= 7)
  void ignitionSchedule7Interrupt(void);
#endif
#if (IGN_CHANNELS >= 8)
  void ignitionSchedule8Interrupt(void);
#endif

#endif

void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, uint32_t timeout, uint32_t duration);
void _setIgnitionScheduleNext(IgnitionSchedule &schedule, uint32_t timeout, uint32_t duration);
void nullIgnCallback(void);

static inline __attribute__((always_inline))
void setIgnitionSchedule(
  IgnitionSchedule &schedule, uint32_t timeout, uint32_t durationMicrosecs)
{
  //Check that we're not already part way through a schedule.
  if(schedule.Status != RUNNING)
  {
    _setIgnitionScheduleRunning(schedule, timeout, durationMicrosecs);
  }
  // Check whether timeout exceeds the maximum future time. This can potentially
  // occur on sequential setups when below ~115rpm.
  else if(timeout < MAX_TIMER_PERIOD)
  {
    _setIgnitionScheduleNext(schedule, timeout, durationMicrosecs);
  }
  else
  {
    /* Do nothing. */
  }
}

extern IgnitionSchedule ignitionSchedules[ignChannelCount];

void initialiseAndResetIgnitionSchedules(void);

