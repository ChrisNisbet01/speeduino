/*
Speeduino - Simple engine management for the Arduino Mega 2560 platform
Copyright (C) Josh Stewart
A full copy of the license may be found in the project's root directory
*/
/** @file
 * Injector and Ignition (on/off) scheduling (functions).
 * There is usually 8 functions for cylinders 1-8 with same naming pattern.
 *
 * ## Scheduling structures
 *
 * Structures @ref FuelSchedule and @ref Schedule describe (from scheduler.h) describe the scheduling info for Fuel and Ignition respectively.
 * They contain duration, current activity status, start timing, end timing, callbacks to carry out action, etc.
 *
 * ## Scheduling Functions
 *
 * For Injection:
 * - setFuelSchedule*(tout,dur) - **Setup** schedule for (next) injection on the channel
 * - inj*StartFunction() - Execute **start** of injection (Interrupt handler)
 * - inj*EndFunction() - Execute **end** of injection (interrupt handler)
 *
 * For Ignition (has more complex schedule setup):
 * - setIgnitionSchedule*(cb_st,tout,dur,cb_end) - **Setup** schedule for (next) ignition on the channel
 * - ign*StartFunction() - Execute **start** of ignition (Interrupt handler)
 * - ign*EndFunction() - Execute **end** of ignition (Interrupt handler)
 */
#include "ignition_scheduler.h"
#include "globals.h"
#include "timers.h"

#define DWELL_SMOOTHED_ALPHA 30
#define DWELL_SMOOTHED(current_dwell, input) ((((long)(input) * (256 - DWELL_SMOOTHED_ALPHA) + ((long)(current_dwell) * DWELL_SMOOTHED_ALPHA))) >> 8)
//#define DWELL_SMOOTHED(current_dwell, input) (current_dwell) //Can be use to disable the above for testing

IgnitionSchedule ignitionSchedules[ignChannelCount] =
{
  IgnitionSchedule(IGN1_COUNTER, IGN1_COMPARE, IGN1_TIMER_DISABLE, IGN1_TIMER_ENABLE),
  IgnitionSchedule(IGN2_COUNTER, IGN2_COMPARE, IGN2_TIMER_DISABLE, IGN2_TIMER_ENABLE),
  IgnitionSchedule(IGN3_COUNTER, IGN3_COMPARE, IGN3_TIMER_DISABLE, IGN3_TIMER_ENABLE),
  IgnitionSchedule(IGN4_COUNTER, IGN4_COMPARE, IGN4_TIMER_DISABLE, IGN4_TIMER_ENABLE),
#if IGN_CHANNELS >= 5
  IgnitionSchedule(IGN5_COUNTER, IGN5_COMPARE, IGN5_TIMER_DISABLE, IGN5_TIMER_ENABLE),
#endif
#if IGN_CHANNELS >= 6
  IgnitionSchedule(IGN6_COUNTER, IGN6_COMPARE, IGN6_TIMER_DISABLE, IGN6_TIMER_ENABLE),
#endif
#if IGN_CHANNELS >= 7
  IgnitionSchedule(IGN7_COUNTER, IGN7_COMPARE, IGN7_TIMER_DISABLE, IGN7_TIMER_ENABLE),
#endif
#if IGN_CHANNELS >= 8
  IgnitionSchedule(IGN8_COUNTER, IGN8_COMPARE, IGN8_TIMER_DISABLE, IGN8_TIMER_ENABLE),
#endif
};

void _setIgnitionScheduleRunning(
  IgnitionSchedule &schedule, unsigned long timeout, unsigned long durationMicrosecs)
{
  schedule.duration = durationMicrosecs;

  //Need to check that the timeout doesn't exceed the overflow
  COMPARE_TYPE timeout_timer_compare;
  // If the timeout is >4x (Each tick represents 4uS) the maximum allowed
  // value of unsigned int (65535), the timer compare value will overflow when
  // applied causing erratic behaviour such as erroneous sparking.
  if (timeout > MAX_TIMER_PERIOD)
  {
    timeout_timer_compare = uS_TO_TIMER_COMPARE(MAX_TIMER_PERIOD - 1);
  }
  else //Normal case
  {
    timeout_timer_compare = uS_TO_TIMER_COMPARE(timeout);
  }

  noInterrupts();

  schedule.startCompare = schedule.counter + timeout_timer_compare;

  //The .endCompare value may be set by the per-tooth timing in decoders.cpp.
  //The check here is to ensure that per-tooth control is not overridden.
  if (!schedule.endScheduleSetByDecoder)
  {
    schedule.endCompare = schedule.startCompare + uS_TO_TIMER_COMPARE(durationMicrosecs);
  }

  SET_COMPARE(schedule.compare, schedule.startCompare);
  schedule.Status = PENDING; //Turn this schedule on

  schedule.pTimerEnable();

  interrupts();
}

void _setIgnitionScheduleNext(IgnitionSchedule &schedule, unsigned long timeout, unsigned long durationMicrosecs)
{
  //If the schedule is already running, we can set the next schedule so it is ready to go
  //This is required in cases of high rpm and high DC where there otherwise would not be enough time to set the schedule
  schedule.nextStartCompare = schedule.counter + uS_TO_TIMER_COMPARE(timeout);
  schedule.nextEndCompare = schedule.nextStartCompare + uS_TO_TIMER_COMPARE(durationMicrosecs);
  schedule.hasNextSchedule = true;
}


void refreshIgnitionSchedule1(unsigned long timeToEnd)
{
  //Must have the threshold check here otherwise it can cause a condition where
  //the compare fires twice, once after the other, both for the end
  IgnitionSchedule &ignition1 = ignitionSchedules[ignChannel1];

  noInterrupts();

  if (ignition1.Status == RUNNING && timeToEnd < ignition1.duration)
  {
    ignition1.endCompare = IGN1_COUNTER + uS_TO_TIMER_COMPARE(timeToEnd);
    SET_COMPARE(IGN1_COMPARE, ignition1.endCompare);
  }

  interrupts();
}

// Shared ISR function for all ignition timers.
static void ignitionScheduleISR(IgnitionSchedule &schedule)
{
  if (schedule.Status == PENDING) //Check to see if this schedule is turn on
  {
    schedule.start.pCallback();
    //Set the status to be in progress (ie The start callback has been called,
    //but not the end callback)
    schedule.Status = RUNNING;
    schedule.startTime = micros();
    if(schedule.endScheduleSetByDecoder)
    {
        SET_COMPARE(schedule.compare, schedule.endCompare);
    }
    else
    {
        SET_COMPARE(schedule.compare,
                    schedule.counter + uS_TO_TIMER_COMPARE(schedule.duration));
    } //Doing this here prevents a potential overflow on restarts
  }
  else if (schedule.Status == RUNNING)
  {
    schedule.end.pCallback();
    schedule.endScheduleSetByDecoder = false;
    ignitionCount = ignitionCount + 1; //Increment the ignition counter
    currentStatus.actualDwell =
      DWELL_SMOOTHED(currentStatus.actualDwell, micros() - schedule.startTime);

    //If there is a next schedule queued up, activate it
    if (schedule.hasNextSchedule)
    {
      SET_COMPARE(schedule.compare, schedule.nextStartCompare);
      schedule.Status = PENDING;
      schedule.hasNextSchedule = false;
    }
    else
    {
      schedule.Status = OFF; //Turn off the schedule
      schedule.pTimerDisable();
    }
  }
  else if (schedule.Status == OFF)
  {
    //Catch any spurious interrupts. This really shouldn't ever be called, but there as a safety
    schedule.pTimerDisable();
  }
}

#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule1Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel1]);
  }

#if IGN_CHANNELS >= 2
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule2Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel2]);
  }
#endif

#if IGN_CHANNELS >= 3
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER5_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule3Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel3]);
  }
#endif

#if IGN_CHANNELS >= 4
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPA_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule4Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel4]);
  }
#endif

#if IGN_CHANNELS >= 5
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule5Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel5]);
  }
#endif

#if IGN_CHANNELS >= 6
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER4_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule6Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel6]);
  }
#endif

#if IGN_CHANNELS >= 7
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPC_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule7Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel7]);
  }
#endif

#if IGN_CHANNELS >= 8
#if defined(CORE_AVR) //AVR chips use the ISR for this
ISR(TIMER3_COMPB_vect) //cppcheck-suppress misra-c2012-8.2
#else
void ignitionSchedule8Interrupt(void) //Most ARM chips can simply call a function
#endif
  {
    ignitionScheduleISR(ignitionSchedules[ignChannel8]);
  }
#endif

void disablePendingIgnSchedule(byte const channel)
{
  if (channel < ignChannelCount)
  {
    noInterrupts();

    if(ignitionSchedules[channel].Status == PENDING)
    {
      ignitionSchedules[channel].Status = OFF;
    }

    interrupts();
  }
}

