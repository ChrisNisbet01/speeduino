#pragma once

#include "ignition_schedule_class.h"
#include "globals.h"

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

void refreshIgnitionSchedule1(unsigned long timeToEnd);

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

void _setIgnitionScheduleRunning(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration);
void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long duration);

static inline __attribute__((always_inline))
void setIgnitionSchedule(
  IgnitionSchedule &schedule, unsigned long timeout, unsigned long durationMicrosecs)
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

